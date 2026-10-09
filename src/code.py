# Handbrake, stage 2: Hall sensors -> progressive axis, calibrated at power-up.
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
import math
import struct
import time

import analogio
import board
import digitalio
import usb_hid

from calibration import Calibrator

SENSOR_PINS = (board.AIN0, board.AIN1, board.AIN4)

SAMPLES = 8              # readings averaged per sensor, nRF ADC is noisy
LOOP_DELAY = 0.005
PRINT_PERIOD = 0.1       # seconds between serial prints in normal use

BLINK_PERIOD = 0.25
REST_READINGS = 50       # averaged at power-up, handbrake untouched
START_MOVE = 600         # raw distance from rest that starts calibration
LEAD_IN = 20             # readings kept from just before the movement started
CALIBRATION_TIME = 30
DUMP_CALIBRATION = True  # print every calibration reading, for tuning

AXIS_MAX = 32767
BUTTON_PRESS_AT = 0.6    # button 1 too, for games without axis support
BUTTON_RELEASE_AT = 0.4

sensors = [analogio.AnalogIn(pin) for pin in SENSOR_PINS]

led = digitalio.DigitalInOut(board.LED_GREEN)
led.switch_to_output(value=True)  # LEDs are active low: True = off

handbrake = usb_hid.devices[0]


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


def measure_rest():
    readings = [read_all() for _ in range(REST_READINGS)]
    return [sum(column) / len(readings) for column in zip(*readings)]


def wait_for_pull():
    """Blink until the handbrake moves. Returns (rest, readings just before)."""
    time.sleep(0.5)
    rest = measure_rest()
    print("waiting for pull, rest:", [int(r) for r in rest])
    recent = []
    blink_at = time.monotonic()
    while True:
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
        time.sleep(LOOP_DELAY)


def calibrate():
    while True:
        rest, recent = wait_for_pull()
        set_led(True)
        print("calibrating")
        calibrator = Calibrator(rest)
        for values in recent:
            calibrator.add(values)
        end = time.monotonic() + CALIBRATION_TIME
        while time.monotonic() < end:
            values = read_all()
            calibrator.add(values)
            if DUMP_CALIBRATION:
                print("cal", values[0], values[1], values[2])
            time.sleep(LOOP_DELAY)
        set_led(False)
        print("samples:", calibrator.samples)
        try:
            mapping = calibrator.finish()
        except ValueError as error:
            print("calibration failed:", error)
            continue
        print("calibrated:", mapping.summary())
        return mapping


mapping = calibrate()

sent = None
pressed = False
print_at = 0
while True:
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
        # Tuple format so the Mu / Thonny plotter can graph it.
        print((values[0], values[1], values[2], int(pos * 100)))
        print_at = now + PRINT_PERIOD

    time.sleep(LOOP_DELAY)
