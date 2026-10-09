# Handbrake, stage 2: Hall sensors -> progressive axis, calibrated at power-up
# (or at any time from ui/index.html).
#
# Wiring (sensors powered from 3.3V, never 5V):
#   sensor 1 signal -> P0.02 (AIN0)
#   sensor 2 signal -> P0.03 (AIN1)
#   sensor 3 signal -> P0.28 (AIN4)
#
# Green LED:
#   blinking -> plugged in, waiting: pull the handbrake to start calibrating
#   solid    -> calibrating for 30 s: pull slowly all the way, hold a second
#               at the end, release fully, wait a second, repeat
#   off      -> ready; lights only while the handbrake is at 100%
#
# Leave the handbrake released when plugging in: the resting position is
# measured while the LED blinks.
#
# Serial protocol, used by ui/index.html (other lines are for humans):
#   in:  "calibrate"  measure rest now and calibrate (at any time)
#   out, every PRINT_PERIOD in each state:
#        waiting <s1> <s2> <s3>
#        progress <seconds left> <phase> <pulls> <s1> <s2> <s3>
#        pos <s1> <s2> <s3> <percent> <button 0/1>
#   out, once: resting | calibrated <summary> | failed <reason>
#   phase: settling, rest, pulling, held, releasing
import math
import struct
import sys
import time

import analogio
import board
import digitalio
import supervisor
import usb_hid

from calibration import Calibrator

SENSOR_PINS = (board.AIN0, board.AIN1, board.AIN4)

SAMPLES = 8              # readings averaged per sensor, nRF ADC is noisy
LOOP_DELAY = 0.005
PRINT_PERIOD = 0.1       # seconds between status lines

BLINK_PERIOD = 0.25
REST_READINGS = 50       # averaged at power-up, handbrake untouched
START_MOVE = 600         # raw distance from rest that starts calibration
LEAD_IN = 20             # readings kept from just before the movement started
CALIBRATION_TIME = 30
HOLD_TIME = 0.6          # seconds on the same node = held still, for the phase
DUMP_CALIBRATION = True  # print every calibration reading, for tuning

AXIS_MAX = 32767
BUTTON_PRESS_AT = 0.6    # button 1 too, for games without axis support
BUTTON_RELEASE_AT = 0.4

sensors = [analogio.AnalogIn(pin) for pin in SENSOR_PINS]

led = digitalio.DigitalInOut(board.LED_GREEN)
led.switch_to_output(value=True)  # LEDs are active low: True = off

handbrake = usb_hid.devices[0]

command_buffer = ""


def set_led(on):
    led.value = not on


def read(sensor):
    total = 0
    for _ in range(SAMPLES):
        total += sensor.value
    return total // SAMPLES


def read_all():
    return [read(s) for s in sensors]


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def send(axis, pressed):
    try:
        handbrake.send_report(struct.pack("<HH", 1 if pressed else 0, axis))
    except OSError:
        pass  # USB not ready yet (e.g. host still enumerating)


def read_command():
    """Returns a complete line received over serial, or None. Never blocks."""
    global command_buffer
    while supervisor.runtime.serial_bytes_available:
        char = sys.stdin.read(1)
        if char in "\r\n":
            command, command_buffer = command_buffer.strip(), ""
            if command:
                return command
        else:
            command_buffer += char
    return None


def measure_rest():
    readings = [read_all() for _ in range(REST_READINGS)]
    return [sum(column) / len(readings) for column in zip(*readings)]


def rest_now():
    """Calibration asked for over serial: the handbrake is released right now."""
    print("resting")
    return measure_rest(), []


def wait_for_start():
    """Blink until the handbrake moves or "calibrate" arrives.

    Returns (rest, readings just before the movement)."""
    time.sleep(0.5)
    rest = measure_rest()
    print("rest:", [int(r) for r in rest], "- pull the handbrake to calibrate")
    recent = []
    blink_at = print_at = time.monotonic()
    while True:
        if read_command() == "calibrate":
            return rest_now()
        values = read_all()
        if distance(values, rest) > START_MOVE:
            return rest, recent
        recent.append(values)
        if len(recent) > LEAD_IN:
            recent.pop(0)
        now = time.monotonic()
        if now >= blink_at:
            led.value = not led.value
            blink_at = now + BLINK_PERIOD
        if now >= print_at:
            print("waiting", values[0], values[1], values[2])
            print_at = now + PRINT_PERIOD
        time.sleep(LOOP_DELAY)


def calibrate(start_now=False):
    while True:
        rest, recent = rest_now() if start_now else wait_for_start()
        start_now = False
        set_led(True)
        print("calibrating")
        calibrator = Calibrator(rest)
        for values in recent:
            calibrator.add(values)

        # Phase tracking, only for the instructions shown by the UI.
        node, node_since = calibrator.node, time.monotonic()
        was_held = False
        pulls = 0

        end = time.monotonic() + CALIBRATION_TIME
        print_at = 0
        while True:
            now = time.monotonic()
            if now >= end:
                break
            values = read_all()
            calibrator.add(values)
            if DUMP_CALIBRATION:
                print("cal", values[0], values[1], values[2])

            if calibrator.node != node:
                node, node_since = calibrator.node, now
            still = now - node_since >= HOLD_TIME
            if distance(values, rest) < START_MOVE:
                if was_held:
                    pulls += 1
                    was_held = False
                phase = "rest" if still else "settling"
            elif still:
                was_held = True
                phase = "held"
            else:
                phase = "releasing" if was_held else "pulling"

            if now >= print_at:
                print("progress", "{:.1f}".format(end - now), phase, pulls,
                      values[0], values[1], values[2])
                print_at = now + PRINT_PERIOD
            time.sleep(LOOP_DELAY)
        set_led(False)
        print("samples:", calibrator.samples)
        try:
            mapping = calibrator.finish()
        except ValueError as error:
            print("failed", error)
            continue
        print("calibrated", mapping.summary())
        return mapping


mapping = calibrate()

sent = None
pressed = False
print_at = 0
while True:
    if read_command() == "calibrate":
        send(0, False)
        set_led(False)
        mapping = calibrate(start_now=True)
        sent = None
        pressed = False
        continue

    values = read_all()
    pos = mapping.position(values)
    axis = int(pos * AXIS_MAX)

    if not pressed and pos > BUTTON_PRESS_AT:
        pressed = True
    elif pressed and pos < BUTTON_RELEASE_AT:
        pressed = False

    # Only send changes: with no game reading the device, each send stalls.
    if (axis, pressed) != sent:
        send(axis, pressed)
        sent = (axis, pressed)
    set_led(pos >= 1.0)

    now = time.monotonic()
    if now >= print_at:
        print("pos", values[0], values[1], values[2], int(pos * 100), int(pressed))
        print_at = now + PRINT_PERIOD

    time.sleep(LOOP_DELAY)
