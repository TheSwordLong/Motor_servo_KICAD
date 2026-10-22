# Standalone build script for GD32F303VET6_MotorControl.
#
# Use this when the normal incremental build hangs at
# "[0/2] Re-checking globbed directories..." (CMake's glob re-check can stall).
#
# It compiles every source file with the exact flags from the project's
# compile_commands.json, then links the firmware, and finally produces the
# .elf / .hex / .bin / .map artifacts in build/debug.
#
# Usage (from the project root):
#     powershell -ExecutionPolicy Bypass -File tools\build_verify.ps1

$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

$toolchain = 'E:\DevTools\arm-gnu-toolchain-15.3.rel1-mingw-w64-i686-arm-none-eabi\bin'
$gcc     = Join-Path $toolchain 'arm-none-eabi-gcc.exe'
$objcopy = Join-Path $toolchain 'arm-none-eabi-objcopy.exe'
$size    = Join-Path $toolchain 'arm-none-eabi-size.exe'

$outDir = Join-Path $root 'build\debug'
$tmpDir = Join-Path $root 'build\verify'

if (-not (Test-Path $gcc)) { throw "Toolchain not found: $gcc" }

# --- Source list (keep in sync with CMakeLists.txt) -----------------------
$sources = @(
    'Core/Src/main.c'
    'Core/Src/gd32f30x_it.c'
    'Core/Src/systick.c'

    'BSP/button.c'
    'BSP/can_bus.c'
    'BSP/capture.c'
    'BSP/led.c'
    'BSP/motor_io.c'
    'BSP/pwm.c'
    'BSP/robot.c'
    'BSP/robot_protocol.c'
    'BSP/robot_rtu.c'
    'BSP/robot_store.c'
    'BSP/robot_port.c'
    'BSP/rs485.c'
    'BSP/usart0.c'
    'BSP/usart2.c'

    'Drivers/CMSIS/system_gd32f30x.c'
    'startup/startup_gd32f30x_hd.S'
    'Middlewares/Newlib/syscalls.c'
)

$sources += (Get-ChildItem 'Drivers/GD32F30x_standard_peripheral/Source' -Filter *.c |
             ForEach-Object { "Drivers/GD32F30x_standard_peripheral/Source/$($_.Name)" })

# --- Flags (keep in sync with CMakeLists.txt) -----------------------------
$includes = @(
    '-ICore/Inc'
    '-IBSP'
    '-IDrivers/CMSIS'
    '-IDrivers/GD32F30x_standard_peripheral/Include'
)

$cFlags = @(
    '-DGD32F30X_HD'
    '-std=gnu11'
    '-mcpu=cortex-m4'
    '-mthumb'
    '-mfpu=fpv4-sp-d16'
    '-mfloat-abi=hard'
    '-Og'
    '-g3'
    '-ffunction-sections'
    '-fdata-sections'
    '-fno-common'
    '-Wall'
    '-Wextra'
)

$mcuFlags = @('-mcpu=cortex-m4', '-mthumb', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard')

$linkFlags = $mcuFlags + @(
    "-T$root/linker/gd32f303xE_flash.ld"
    '-Wl,--gc-sections'
    "-Wl,-Map=$outDir/GD32F303VET6_MotorControl.map"
    '--specs=nano.specs'
    '-g'
)

# --- Compile --------------------------------------------------------------
Remove-Item $tmpDir -Recurse -Force -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force -Path $tmpDir | Out-Null

$objects = @()
$failed = 0

foreach ($src in $sources) {
    if (-not (Test-Path $src)) { Write-Host "SKIP (missing): $src"; continue }

    $obj = Join-Path $tmpDir (($src -replace '/', '\') -replace '\.(c|S)$', '.obj')
    New-Item -ItemType Directory -Force -Path (Split-Path $obj) | Out-Null

    $srcFlags = if ($src -like '*.S') { $mcuFlags } else { $cFlags }

    & $gcc @srcFlags @includes -o $obj -c $src
    if ($LASTEXITCODE -ne 0) {
        Write-Host "FAILED: $src" -ForegroundColor Red
        $failed++
    } else {
        $objects += $obj
    }
}

if ($failed -gt 0) { throw "$failed source file(s) failed to compile" }

Write-Host "Compiled $($objects.Count) objects."

# --- Link -----------------------------------------------------------------
$elf = Join-Path $outDir 'GD32F303VET6_MotorControl.elf'
& $gcc @objects @linkFlags -o $elf
if ($LASTEXITCODE -ne 0) { throw 'Link failed' }

# --- Post-build artifacts -------------------------------------------------
& $objcopy -O ihex   $elf (Join-Path $outDir 'GD32F303VET6_MotorControl.hex')
& $objcopy -O binary $elf (Join-Path $outDir 'GD32F303VET6_MotorControl.bin')

Write-Host ''
Write-Host 'Build succeeded:' -ForegroundColor Green
& $size $elf
