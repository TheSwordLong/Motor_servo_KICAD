<#
    Standalone EIDE-equivalent build for GD32F303VET6_MotorControl.

    Purpose: build the firmware with exactly the same compiler/linker options
    and the same output layout that the EIDE extension uses, so the result can
    be flashed directly from build-eide\Debug without opening the EIDE GUI.

    Source list is read from .eide/eide.yml, so the script and the EIDE project
    always agree on which files are in the build.

    Usage (from the project root):
        powershell -ExecutionPolicy Bypass -File tools\build_eide.ps1
        powershell -ExecutionPolicy Bypass -File tools\build_eide.ps1 -Rebuild
#>

param(
    [switch]$Rebuild
)

$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

# --- Toolchain (same as .vscode/settings.json -> EIDE.ARM.GCC.InstallDirectory) ---
$toolchain = 'E:\DevTools\arm-gnu-toolchain-15.3.rel1-mingw-w64-i686-arm-none-eabi\bin'
$gcc     = Join-Path $toolchain 'arm-none-eabi-gcc.exe'
$objcopy = Join-Path $toolchain 'arm-none-eabi-objcopy.exe'
$size    = Join-Path $toolchain 'arm-none-eabi-size.exe'

if (-not (Test-Path $gcc)) { throw "Toolchain not found: $gcc" }

$outDir  = Join-Path $root 'build-eide\Debug'
$objDir  = Join-Path $outDir '.obj'

if ($Rebuild) {
    Remove-Item $objDir -Recurse -Force -ErrorAction SilentlyContinue
}
New-Item -ItemType Directory -Force -Path $objDir | Out-Null

# --- Source list: single source of truth is .eide/eide.yml -------------------
$eideYml = Join-Path $root '.eide\eide.yml'
if (-not (Test-Path $eideYml)) { throw "EIDE project file not found: $eideYml" }

$sources = @()
foreach ($line in Get-Content $eideYml) {
    if ($line -match '^\s*-\s*path:\s*(\S+)\s*$') { $sources += $Matches[1] }
}
if ($sources.Count -eq 0) { throw "No source files listed in .eide\eide.yml (virtualFolder is empty)" }

# --- Flags: copied verbatim from the EIDE build (compile_commands.json / .lnp) ---
$includes = @(
    '-ICore/Inc'
    '-IBSP'
    '-IDrivers/CMSIS'
    '-IDrivers/GD32F30x_standard_peripheral/Include'
    '-IApp/USB/Inc'
    '-IDrivers/USB_Device/Core/Include'
    '-IDrivers/USB_Device/LL/Include'
    '-IDrivers/USB_Device/Class/CDC/Include'
)

$mcuFlags = @('-mcpu=cortex-m4', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard', '-mthumb')

$cFlags = @(
    '-std=gnu11'
    '-DGD32F30X_HD'
    '-Og'
    '-Wall'
    '-g'
    '-ffunction-sections'
    '-fdata-sections'
    '-funsigned-char'
    '--specs=nano.specs'
    '-fno-common'
)

$asmFlags = @('-DGD32F30X_HD', '-g', '-funsigned-char', '--specs=nano.specs', '-fno-common')

$linkFlags = $mcuFlags + @(
    "-T$root/linker/gd32f303xE_flash.ld"
    '-Wl,--gc-sections'
    '-Wl,--print-memory-usage'
    '--specs=nano.specs'
    '-fno-common'
    "-Wl,-Map=./build-eide/Debug/GD32F303VET6_MotorControl.map"
)

# --- Compile -----------------------------------------------------------------
$objects = @()
$failed  = 0

foreach ($src in $sources) {
    if (-not (Test-Path -LiteralPath $src)) { Write-Host "SKIP (missing): $src" -ForegroundColor Yellow; continue }

    $obj = Join-Path $objDir (($src -replace '/', '\') -replace '\.(c|S)$', '.o')
    New-Item -ItemType Directory -Force -Path (Split-Path $obj) | Out-Null

    if ($src -like '*.S') {
        & $gcc @mcuFlags @asmFlags @includes -x assembler-with-cpp -c $src -o $obj
    } else {
        & $gcc @mcuFlags @cFlags @includes -c $src -o $obj
    }

    if ($LASTEXITCODE -ne 0) {
        Write-Host "FAILED: $src" -ForegroundColor Red
        $failed++
    } else {
        $objects += $obj
    }
}

if ($failed -gt 0) { throw "$failed source file(s) failed to compile" }
Write-Host "Compiled $($objects.Count) objects." -ForegroundColor Green

# --- Link --------------------------------------------------------------------
$elf = Join-Path $outDir 'GD32F303VET6_MotorControl.elf'
& $gcc @objects @linkFlags -o $elf
if ($LASTEXITCODE -ne 0) { throw 'Link failed' }

# --- Post-build artifacts ----------------------------------------------------
& $objcopy -O ihex   $elf (Join-Path $outDir 'GD32F303VET6_MotorControl.hex')
& $objcopy -O binary $elf (Join-Path $outDir 'GD32F303VET6_MotorControl.bin')

Write-Host ''
Write-Host 'Build succeeded (EIDE layout):' -ForegroundColor Green
& $size $elf
Write-Host ''
Write-Host 'Flash with:'
Write-Host '  powershell -ExecutionPolicy Bypass -File tools\flash_openocd.ps1'
