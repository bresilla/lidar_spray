#!/usr/bin/env python

import can
import time
import struct

can.rc['interface'] = 'socketcan'
can.rc['channel'] = 'can0'
can.rc['bitrate'] = 500000

from can.interface import Bus
bus = Bus()

speed = [4, 5]
speed_id = 0x0C019980

nozzles = {
    0: 0x0C218099,
    1: 0x0C218099,
    2: 0x0C218099,
    3: 0x0C218099,
    4: 0x0C218099,
    5: 0x0C218099,
    6: 0x0C218099,
    7: 0x0C218099,
    8: 0x0C228099,
    9: 0x0C228099,
    10: 0x0C228099,
    11: 0x0C228099,
    12: 0x0C228099,
    13: 0x0C228099,
    14: 0x0C228099,
    15: 0x0C228099
}

def send2can(id, by):
    msg = can.Message(arbitration_id=id, data=by, is_extended_id=True)
    try:
        bus.send(msg)
        print(f"Message sent on {bus.channel_info}")
        print(f"Message content: {msg}")
    except can.CanError:
        print("Message NOT sent")


def message_listener(id):
    while True:
        message = bus.recv()
        if message.arbitration_id == id:
            return message

def main():
    id = 0x0C228099
    by = [0, 25, 0, 34, 3, 155, 4, 241]
    while True:
        send2can(id, by)
        # time.sleep(1)


def spray(nozzle, amount):
    amount = int(amount / 0.4)
    id = nozzles[nozzle]
    by = [0, 0, 0, 0, 0, 0, 0, 0]
    nozzle_fix = nozzle if nozzle < 8 else nozzle - 8
    ammount_fix = amount if amount < 255 else 255
    by[nozzle_fix] = ammount_fix
    send2can(id, by)

def gps_position():
    id = 0x0C029980
    message = message_listener(id)
    print(message)
    lat = (message.data[7] << 24) | (message.data[6] << 16) | (message.data[5] << 8) | message.data[4]
    print(lat)
    lon = (message.data[3] << 24) | (message.data[2] << 16) | (message.data[1] << 8) | message.data[0]
    print(lon)

def actual_speed():
    id = 0x0C019980
    message = message_listener(id)
    print(message)
    value = (message.data[5] << 8) | message.data[4]
    print(value)

def machine_speed():
    id = 0x0C019980
    message = message_listener(id)
    print(message)
    value = (message.data[5] << 8) | message.data[4]
    print(value)

if __name__ == '__main__':
    # main()
    while True:
        spray(3, 90)
        time.sleep(0.1)


