#!/usr/bin/env bash
# Shows the handbrake's serial output (calibration progress, then one
# "(sensor1, sensor2, sensor3, percent)" line every 0.1 s). Ctrl+C to quit.
#
#   ./test.bash              follow the output as it comes
#   ./test.bash --restart    restart code.py first, to watch calibration from the start
#   ./test.bash -t 10        stop after 10 seconds
#   PORT=COM6 ./test.bash    use this serial port instead of looking for the board
#
# On Windows the port is read through PowerShell: CircuitPython only sends
# output once the host raises DTR, which reading the port from bash does not do.
set -euo pipefail

CIRCUITPY_VID=239A

restart=false
seconds=0
while (($#)); do
    case $1 in
    -r | --restart) restart=true ;;
    -t | --time)
        seconds=${2:-}
        [[ $seconds =~ ^[0-9]+$ ]] || {
            echo "$1 needs a number of seconds" >&2
            exit 1
        }
        shift
        ;;
    -h | --help)
        sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'
        exit 0
        ;;
    *)
        echo "Unknown option: $1 (see --help)" >&2
        exit 1
        ;;
    esac
    shift
done

is_windows() {
    case "$(uname -s)" in MINGW* | MSYS* | CYGWIN*) return 0 ;; *) return 1 ;; esac
}

find_port() {
    if is_windows; then
        powershell.exe -NoProfile -Command \
            "Get-CimInstance Win32_PnPEntity | Where-Object { \$_.DeviceID -like 'USB\\VID_$CIRCUITPY_VID*' -and \$_.Name -match '\\((COM\\d+)\\)' } | ForEach-Object { \$Matches[1] } | Select-Object -First 1" |
            tr -d '\r'
        return
    fi
    local tty
    for tty in /sys/class/tty/ttyACM*; do
        if [[ $(cat "$tty/device/../idVendor" 2>/dev/null) == "${CIRCUITPY_VID,,}" ]]; then
            echo "/dev/$(basename "$tty")"
            return
        fi
    done
    ls /dev/cu.usbmodem* 2>/dev/null | head -n 1 || true
}

# Give a board that is still resetting (e.g. right after flash.bash) time to come back.
port=${PORT:-}
for ((i = 0; i < 10 && ${#port} == 0; i++)); do
    ((i)) && sleep 1
    port=$(find_port)
done
if [[ -z $port ]]; then
    echo "CircuitPython serial port not found. Is the board plugged in?" >&2
    exit 1
fi
echo "Reading $port$( ((seconds)) && echo " for $seconds s"), Ctrl+C to quit." >&2

if is_windows; then
    # Reopens the port if the board resets (e.g. right after flash.bash).
    exec powershell.exe -NoProfile -Command "
        [Console]::OutputEncoding = [Text.Encoding]::UTF8
        \$end = if ($seconds -gt 0) { (Get-Date).AddSeconds($seconds) } else { [DateTime]::MaxValue }
        \$restart = \$$restart
        while ((Get-Date) -lt \$end) {
            \$p = New-Object System.IO.Ports.SerialPort '$port', 115200
            \$p.DtrEnable = \$true
            \$p.Encoding = [Text.Encoding]::UTF8
            try { \$p.Open() } catch { Start-Sleep -Milliseconds 250; continue }
            try {
                if (\$restart) {
                    \$p.Write([string][char]3); Start-Sleep -Milliseconds 200
                    \$p.Write([string][char]4); \$restart = \$false
                }
                while ((Get-Date) -lt \$end) {
                    \$text = \$p.ReadExisting()
                    if (\$text) { [Console]::Out.Write(\$text); [Console]::Out.Flush() }
                    else { Start-Sleep -Milliseconds 20 }
                }
            } catch {
                [Console]::Error.WriteLine('[board disconnected, waiting for it...]')
            } finally { \$p.Close() }
        }"
fi

stty -F "$port" 115200 raw -echo 2>/dev/null || stty -f "$port" 115200 raw -echo
exec 3<>"$port"
if $restart; then
    printf '\x03' >&3
    sleep 0.2
    printf '\x04' >&3
fi
if ((seconds)); then
    timeout "$seconds" cat <&3 || true
else
    cat <&3
fi
