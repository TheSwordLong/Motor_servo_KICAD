param([string]$Compiler = '')
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root
New-Item -ItemType Directory -Force build/host-tests | Out-Null
if (!$Compiler) { $Compiler = Join-Path $root 'build/test-tools/ziglang/zig.exe' }
if (!(Test-Path -LiteralPath $Compiler)) { throw 'Specify a host C compiler, or install: python -m pip install --target build/test-tools ziglang==0.16.0' }
$arguments = @('-std=c11','-Wall','-Wextra','-Werror','-g','-O1','-IBSP',
    'tests/robot_test.c','BSP/robot.c','BSP/robot_protocol.c','BSP/robot_rtu.c','BSP/robot_store.c',
    '-o','build/host-tests/robot_test.exe')
if ((Split-Path -Leaf $Compiler) -eq 'zig.exe') {
    $env:ZIG_GLOBAL_CACHE_DIR = Join-Path $root 'build/zig-cache'
    $arguments = @('cc','-target','x86_64-windows-gnu') + $arguments
}
& $Compiler @arguments
if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed' }
& ./build/host-tests/robot_test.exe
if ($LASTEXITCODE -ne 0) { throw 'Host tests failed' }
