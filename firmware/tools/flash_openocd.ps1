<#
    Flash the GD32F303VET6 control board over CMSIS-DAP / SWD with OpenOCD,
    and optionally dump the key peripheral registers right after flashing.

    Default target is the EIDE build output (build-eide\Debug), which is what
    tools\build_eide.ps1 and the VS Code "EIDE Build Debug" task produce.
    Use -UseCmake to flash the CMake output (build\debug) instead, or -Firmware
    to point at an explicit .elf/.hex/.bin file.

    Usage (from the project root):
        powershell -ExecutionPolicy Bypass -File tools\flash_openocd.ps1
        powershell -ExecutionPolicy Bypass -File tools\flash_openocd.ps1 -Registers
        powershell -ExecutionPolicy Bypass -File tools\flash_openocd.ps1 -UseCmake -Registers
        powershell -ExecutionPolicy Bypass -File tools\flash_openocd.ps1 -Probe
        powershell -ExecutionPolicy Bypass -File tools\flash_openocd.ps1 -Firmware path\to\file.elf

    For a live register view while the firmware runs, use
    tools\openocd_regs.ps1 instead (it keeps OpenOCD open and polls the target).
#>

param(
    [string]$Firmware = '',
    [switch]$UseCmake,
    [switch]$Probe,
    [switch]$Registers,
    [string]$Interface = 'interface/cmsis-dap.cfg',
    [string]$Target    = 'target/gd32f30x.cfg',
    [int]$AdapterSpeed = 1000
)

$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

$openocd  = 'E:\DevTools\OpenOCD\bin\openocd.exe'
$scripts  = 'E:\DevTools\OpenOCD\openocd\scripts'

if (-not (Test-Path $openocd)) { throw "OpenOCD not found: $openocd" }

$commonArgs = @(
    '-s', $scripts
    '-f', $Interface
    '-f', $Target
    '-c', 'transport select swd'
    '-c', "adapter speed $AdapterSpeed"
)

# --- Probe only: verify the debug link without touching the flash ------------
if ($Probe) {
    Write-Host 'Probing target (no flash write)...' -ForegroundColor Cyan
    & $openocd @commonArgs -c 'init; reset halt'
    exit $LASTEXITCODE
}

# --- Resolve the firmware file ----------------------------------------------
if ('' -eq $Firmware) {
    $dir = if ($UseCmake) { 'build\debug' } else { 'build-eide\Debug' }
    $Firmware = Join-Path $root (Join-Path $dir 'GD32F303VET6_MotorControl.elf')
    if (-not (Test-Path $Firmware)) {
        $bin = [System.IO.Path]::ChangeExtension($Firmware, '.bin')
        if ((Test-Path $bin) -and -not (Test-Path $Firmware)) {
            Write-Host "ELF not found, falling back to binary: $bin" -ForegroundColor Yellow
            $Firmware = $bin
        }
    }
}

if (-not (Test-Path $Firmware)) {
    throw @"
Firmware not found: $Firmware

Build it first, either:
  powershell -ExecutionPolicy Bypass -File tools\build_eide.ps1      (EIDE layout)
  cmake --build --preset debug                                       (CMake layout, use -UseCmake)
or press Ctrl+Shift+B in VS Code and pick the "EIDE Build Debug" task.
"@
}

# Relative paths are what OpenOCD expects when run from the project root.
$firmwareArg = $Firmware.Replace($root + '\', '').Replace('\', '/')

# --- Program ----------------------------------------------------------------
Write-Host "Flashing: $firmwareArg" -ForegroundColor Cyan

& $openocd @commonArgs -c "program $firmwareArg verify reset exit"
$code = $LASTEXITCODE

if ($code -eq 0) {
    Write-Host ''
    Write-Host 'Flash OK - expected output: Programming Finished / Verified OK / shutdown command invoked' -ForegroundColor Green
} else {
    Write-Host ''
    Write-Host "OpenOCD exited with code $code" -ForegroundColor Red
    Write-Host 'Common causes:'
    Write-Host '  - CMSIS-DAP not connected / board unpowered (check SWDIO, SWCLK, GND, VTref)'
    Write-Host '  - target held by a running debug session (stop it with Shift+F5 in VS Code)'
    Write-Host '  - wrong interface/target cfg (use -Interface / -Target to override)'
    exit $code
}

# --- Optional register dump --------------------------------------------------
# The firmware now runs freely (program ... reset). Attach again, halt briefly,
# read the registers, then resume so the application keeps running.
if ($Registers) {
    $regsCfg = Join-Path (Split-Path $Firmware) 'openocd_regs_dump.cfg'
    $dumpScript = Join-Path $PSScriptRoot 'openocd_regs.cfg'

    if (-not (Test-Path $dumpScript)) { throw "Register dump script not found: $dumpScript" }

    Copy-Item $dumpScript $regsCfg -Force

    Write-Host ''
    Write-Host 'Reading registers (target is halted for a few milliseconds)...' -ForegroundColor Cyan

    & $openocd @commonArgs -f $regsCfg -c 'init; halt' -c "source [find $($regsCfg.Replace('\','/'))]" -c 'resume; exit'
    $regCode = $LASTEXITCODE

    # The helper cfg is only there for `source`; keep the tree clean.
    Remove-Item $regsCfg -Force -ErrorAction SilentlyContinue

    if ($regCode -ne 0) {
        Write-Host 'Register dump failed (flash itself succeeded).' -ForegroundColor Yellow
    }
}

exit $code
