<#
    Live register viewer for GD32F303VET6 over CMSIS-DAP / SWD.

    Starts OpenOCD and keeps it running, then polls the target and prints the
    key peripheral registers (GPIO / USART / RCU / TIMER / core) as a table.
    Use this when you want to watch register state while the firmware runs,
    without VS Code and without a full GDB session.

    The board keeps running: the script attaches, briefly halts, reads the
    registers, and resumes. Reads are fast (a few ms), but for a firmware with
    hard real-time deadlines prefer watch variables instead.

    Usage (from the project root):
        powershell -ExecutionPolicy Bypass -File tools\openocd_regs.ps1
        powershell -ExecutionPolicy Bypass -File tools\openocd_regs.ps1 -Count 20 -IntervalMs 500
        powershell -ExecutionPolicy Bypass -File tools\openocd_regs.ps1 -Address 0x40004400 -Count 10 -Raw

    Press Ctrl+C to stop (the script shuts OpenOCD down for you).
#>

param(
    [int]$Count       = 10,
    [int]$IntervalMs  = 500,
    [string]$Address  = '',
    [switch]$Raw,
    [switch]$NoDump,
    [string]$Interface = 'interface/cmsis-dap.cfg',
    [string]$Target    = 'target/gd32f30x.cfg',
    [int]$AdapterSpeed = 1000,
    [int]$TelnetPort   = 4444,
    [int]$GdbPort      = 3333
)

$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

$openocd = 'E:\DevTools\OpenOCD\bin\openocd.exe'
$scripts = 'E:\DevTools\OpenOCD\openocd\scripts'

if (-not (Test-Path $openocd)) { throw "OpenOCD not found: $openocd" }

# --- Watch points ------------------------------------------------------------
$watch = @(
    @{ Name = 'GPIOA_OCTL'; Addr = 0x4001080C; Hint = 'bit1=DE(PA1) bit2=TX(PA2)' }
    @{ Name = 'GPIOA_ISTAT'; Addr = 0x40010808; Hint = 'pin levels' }
    @{ Name = 'USART1_STAT0'; Addr = 0x40004400; Hint = 'bit5=RBNE bit6=TC bit7=TBE bit3=ORERR' }
    @{ Name = 'USART1_CTL0'; Addr = 0x4000440C; Hint = 'bit2=REN bit3=TEN bit13=UEN' }
    @{ Name = 'RCU_CFG0'; Addr = 0x40021004; Hint = 'SCSS bits2:3 = 10 -> PLL' }
    @{ Name = 'RCU_APB1EN'; Addr = 0x4002101C; Hint = 'bit17=USART1EN bit2=TIMER3EN' }
)

if ('' -ne $Address) {
    $watch = @(@{ Name = $Address; Addr = [Convert]::ToUInt32($Address, 16); Hint = 'user supplied' })
}

# --- Start OpenOCD -----------------------------------------------------------
# Output goes to its own console window: OpenOCD must not run with a piped
# stdout, because its server loop would block waiting on the pipe.
$args = @(
    '-s', $scripts
    '-f', $Interface
    '-f', $Target
    '-c', 'transport select swd'
    '-c', "adapter speed $AdapterSpeed"
    '-c', 'init'
)
$argLine = ($args | ForEach-Object { if ($_ -match '\s') { '"' + $_ + '"' } else { $_ } }) -join ' '

Write-Host "Starting OpenOCD (telnet $TelnetPort, gdb $GdbPort)..." -ForegroundColor Cyan
$proc = Start-Process -FilePath $openocd -ArgumentList $argLine -PassThru -WindowStyle Minimized

try {
    # --- Wait for the telnet port --------------------------------------------
    $client = $null
    $deadline = (Get-Date).AddSeconds(15)
    while ((Get-Date) -lt $deadline) {
        try {
            $client = New-Object System.Net.Sockets.TcpClient('127.0.0.1', $TelnetPort)
            break
        } catch {
            if ($proc.HasExited) { throw "OpenOCD exited immediately (code $($proc.ExitCode)). Run tools\flash_openocd.ps1 -Probe to see the error." }
            Start-Sleep -Milliseconds 300
        }
    }
    if ($null -eq $client) { throw "Timed out waiting for OpenOCD telnet port $TelnetPort" }

    $stream = $client.GetStream()
    $writer = New-Object System.IO.StreamWriter($stream)
    $writer.AutoFlush = $true
    $reader = New-Object System.IO.StreamReader($stream)

    # Drain the banner, then sync on the prompt.
    Start-Sleep -Milliseconds 400
    while ($stream.DataAvailable) { $null = $reader.Read() }

    function Invoke-OpenOcd([string]$command) {
        $writer.WriteLine($command)
        $sb = New-Object System.Text.StringBuilder
        $idle = (Get-Date).AddSeconds(3)
        while ((Get-Date) -lt $idle) {
            if ($stream.DataAvailable) {
                $sb.Append($reader.ReadToEnd()) | Out-Null
                $idle = (Get-Date).AddMilliseconds(250)
            } else {
                if ($sb.Length -gt 0 -and $sb.ToString().TrimEnd().EndsWith('>')) { break }
                Start-Sleep -Milliseconds 50
            }
        }
        return $sb.ToString()
    }

    # --- Register dump script path (for the one-shot full dump) --------------
    $dumpCfg = (Join-Path $PSScriptRoot 'openocd_regs.cfg').Replace('\', '/')

    Write-Host ''
    Write-Host "Attached. Watching $($watch.Count) register(s), $Count sample(s) every ${IntervalMs} ms." -ForegroundColor Green
    Write-Host 'Press Ctrl+C to stop.' -ForegroundColor DarkGray
    Write-Host ''

    for ($i = 1; $i -le $Count; $i++) {
        # Halt briefly, read, resume: a read while running is not reliable for
        # peripheral registers on this core.
        $null = Invoke-OpenOcd 'halt'

        $stamp = (Get-Date).ToString('HH:mm:ss.fff')

        if ($Raw) {
            $resp = Invoke-OpenOcd ('mdw 0x{0:X8} 1' -f $watch[0].Addr)
            Write-Host ("[{0}] {1}" -f $stamp, ($resp.Trim() -replace "`r?`n", ' '))
        } else {
            $line = "[$stamp] "
            foreach ($w in $watch) {
                $resp = Invoke-OpenOcd ('mdw 0x{0:X8} 1' -f $w.Addr)
                $hex = ([regex]::Match($resp, '0x[0-9a-fA-F]{8}')).Value
                $line += ("{0}={1}  " -f $w.Name, $hex)
            }
            Write-Host $line
        }

        $null = Invoke-OpenOcd 'resume'

        if ($i -lt $Count) { Start-Sleep -Milliseconds $IntervalMs }
    }

    # --- Optional one-shot full dump -----------------------------------------
    if (-not $NoDump -and -not $Raw) {
        Write-Host ''
        Write-Host 'Full register dump:' -ForegroundColor Cyan
        $null = Invoke-OpenOcd 'halt'
        $dump = Invoke-OpenOcd "source [find $dumpCfg]"
        Write-Host $dump
        $null = Invoke-OpenOcd 'resume'
    }

    Write-Host ''
    Write-Host 'Done. Hint: to decode a value, e.g. USART1_STAT0=0x000000E0 means TBE|TC|RBNE are set.' -ForegroundColor DarkGray
}
finally {
    # --- Always clean up -----------------------------------------------------
    if ($null -ne $client) {
        try { $writer.WriteLine('shutdown'); Start-Sleep -Milliseconds 200; $client.Close() } catch { }
    }
    if ($null -ne $proc -and -not $proc.HasExited) {
        try { $proc.Kill() } catch { }
        Write-Host 'OpenOCD stopped.' -ForegroundColor DarkGray
    }
}
