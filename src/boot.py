# Runs once at power-up, before USB is set up.
# Replaces the default keyboard/mouse with a single game controller:
# 16 buttons + 1 axis (Z). The axis is reserved for the progressive handbrake.
import usb_hid

REPORT_ID = 4

HANDBRAKE_DESCRIPTOR = bytes((
    0x05, 0x01,              # Usage Page (Generic Desktop)
    0x09, 0x04,              # Usage (Joystick)
    0xA1, 0x01,              # Collection (Application)
    0x85, REPORT_ID,         #   Report ID
    0x05, 0x09,              #   Usage Page (Button)
    0x19, 0x01,              #   Usage Minimum (Button 1)
    0x29, 0x10,              #   Usage Maximum (Button 16)
    0x15, 0x00,              #   Logical Minimum (0)
    0x25, 0x01,              #   Logical Maximum (1)
    0x75, 0x01,              #   Report Size (1)
    0x95, 0x10,              #   Report Count (16)
    0x81, 0x02,              #   Input (Data, Var, Abs)
    0x05, 0x01,              #   Usage Page (Generic Desktop)
    0x09, 0x32,              #   Usage (Z)
    0x15, 0x00,              #   Logical Minimum (0)
    0x26, 0xFF, 0x7F,        #   Logical Maximum (32767)
    0x75, 0x10,              #   Report Size (16)
    0x95, 0x01,              #   Report Count (1)
    0x81, 0x02,              #   Input (Data, Var, Abs)
    0xC0,                    # End Collection
))

handbrake = usb_hid.Device(
    report_descriptor=HANDBRAKE_DESCRIPTOR,
    usage_page=0x01,
    usage=0x04,
    report_ids=(REPORT_ID,),
    in_report_lengths=(4,),   # 2 bytes buttons + 2 bytes axis
    out_report_lengths=(0,),
)

usb_hid.enable((handbrake,))
