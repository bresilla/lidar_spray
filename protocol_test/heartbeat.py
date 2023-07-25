#!/usr/bin/env python

import can
import time

can.rc['interface'] = 'socketcan'
can.rc['channel'] = 'can0'
can.rc['bitrate'] = 500000

from can.interface import Bus
bus = Bus()

def send2can(id, by):
    msg = can.Message(arbitration_id=id, data=by, is_extended_id=True)
    try:
        bus.send(msg)
        print(f"Message sent on {bus.channel_info}")
        print(f"Message content: {msg}")
    except can.CanError:
        print("Message NOT sent")

def main():
    id = 0x14FF8099
    by = 0x01
    while True:
        send2can(id, by)
        time.sleep(0.2)

if __name__ == '__main__':
    main() 