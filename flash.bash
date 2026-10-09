#!/usr/bin/env bash
# Flashes the handbrake code to the nRF52840-MDK.
#
#   ./flash.bash             copy src/*.py to the CIRCUITPY drive (changed files only)
#   ./flash.bash --reset     same, then always hard-reset the board
#   ./flash.bash --firmware  first flash CircuitPython + S140 through the DAPLINK
#                            drive, then copy src/*.py
#
# CircuitPython auto-reloads code.py when a file is saved, but boot.py (USB HID
# setup) only runs at power-up, so the board is hard-reset when boot.py changes.
# The reset goes through the DAPLink serial port (a serial break resets the
# target); where that is not available, press the board's RESET button.
set -euo pipefail
cd "$(dirname "$0")"

FIRMWARE=firmware/circuitpython-10.3.1-with-s140.hex
DAPLINK_VID=0D28
# code.py last: saving it triggers the auto-reload, so everything else must be there.
FILES=(src/calibration.py src/boot.py src/code.py)

is_windows() {
    case "$(uname -s)" in MINGW* | MSYS* | CYGWIN*) return 0 ;; *) return 1 ;; esac
}

# Prints the mount point of the drive with the given label, or nothing.
find_drive() {
    local label=$1
    if is_windows; then
        local letter
        letter=$(powershell.exe -NoProfile -Command \
            "(Get-Volume -FileSystemLabel '$label' -ErrorAction SilentlyContinue | Select-Object -First 1).DriveLetter" |
            tr -d '\r')
        [[ -n $letter ]] && cygpath -u "$letter:/"
        return 0
    fi
    local dir
    for dir in "/media/$USER/$label" "/run/media/$USER/$label" "/media/$label" "/Volumes/$label"; do
        if [[ -d $dir ]]; then
            echo "$dir"
            return 0
        fi
    done
}

# Waits up to $2 seconds for the drive to appear; prints its mount point.
wait_for_drive() {
    local label=$1 timeout=$2 drive
    for ((i = 0; i < timeout; i++)); do
        drive=$(find_drive "$label")
        if [[ -n $drive ]]; then
            echo "$drive"
            return 0
        fi
        sleep 1
    done
    return 1
}

# Prints the COM port of the USB device with the given vendor ID (Windows only).
find_com_port() {
    powershell.exe -NoProfile -Command \
        "Get-CimInstance Win32_PnPEntity | Where-Object { \$_.DeviceID -like 'USB\\VID_$1*' -and \$_.Name -match '\\((COM\\d+)\\)' } | ForEach-Object { \$Matches[1] } | Select-Object -First 1" |
        tr -d '\r'
}

reset_board() {
    local port=
    is_windows && port=$(find_com_port "$DAPLINK_VID")
    if [[ -z $port ]]; then
        echo "Press the board's RESET button so boot.py runs again."
        return
    fi
    echo "Resetting the board through DAPLink ($port)..."
    powershell.exe -NoProfile -Command "
        \$p = New-Object System.IO.Ports.SerialPort '$port', 9600
        \$p.Open(); \$p.BreakState = \$true; Start-Sleep -Milliseconds 100
        \$p.BreakState = \$false; \$p.Close()"
}

flash_firmware() {
    local daplink
    daplink=$(find_drive DAPLINK)
    if [[ -z $daplink ]]; then
        echo "DAPLINK drive not found. Is the board plugged in?" >&2
        exit 1
    fi
    echo "Flashing $FIRMWARE through $daplink (takes ~30 s)..."
    cp "$FIRMWARE" "$daplink"
    sync
    # DAPLINK remounts once programming is done; the target then boots
    # CircuitPython, which brings CIRCUITPY back.
    sleep 5
    daplink=$(wait_for_drive DAPLINK 60) || {
        echo "DAPLINK drive did not come back." >&2
        exit 1
    }
    if [[ -f $daplink/FAIL.TXT ]]; then
        echo "DAPLink reported a failure:" >&2
        cat "$daplink/FAIL.TXT" >&2
        exit 1
    fi
    echo "Firmware flashed."
}

firmware=false
force_reset=false
for arg in "$@"; do
    case $arg in
    --firmware) firmware=true ;;
    --reset) force_reset=true ;;
    -h | --help)
        sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'
        exit 0
        ;;
    *)
        echo "Unknown option: $arg (see --help)" >&2
        exit 1
        ;;
    esac
done

$firmware && flash_firmware

circuitpy=$(wait_for_drive CIRCUITPY 30) || {
    echo "CIRCUITPY drive not found. Is the board plugged in and running CircuitPython?" >&2
    echo "(If it is not, run: $0 --firmware)" >&2
    exit 1
}
echo "Board: $(head -n 1 "$circuitpy/boot_out.txt" 2>/dev/null || echo unknown) at $circuitpy"

changed=()
for file in "${FILES[@]}"; do
    target=$circuitpy/$(basename "$file")
    if cmp -s "$file" "$target"; then
        echo "  unchanged  $(basename "$file")"
    else
        cp "$file" "$target"
        echo "  copied     $(basename "$file")"
        changed+=("$(basename "$file")")
    fi
done
sync

if $force_reset || [[ " ${changed[*]} " == *" boot.py "* ]]; then
    reset_board
elif ((${#changed[@]})); then
    echo "code.py reloads by itself."
else
    echo "Nothing changed on the board."
fi
