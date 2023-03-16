import sys
import time

print("Initializing pymata4...\n")
from pymata4 import pymata4

"""
Tests fully assembled robot. 
"""

board = pymata4.Pymata4()
center = 135
scale = 180/270

try:
    print('setting servos...')
    board.set_pin_mode_servo(2)       # L0
    board.servo_write(2, int((0 + center) * scale))
    board.set_pin_mode_servo(3)       # L1
    board.servo_write(3, int((45 + center) * scale))
    board.set_pin_mode_servo(4)       # L2
    board.servo_write(4, int((90 + center) * scale))
    board.set_pin_mode_servo(5)       # R0
    board.servo_write(5, int((0 + center) * scale))
    board.set_pin_mode_servo(6)
    board.servo_write(6, int((-45 + center) * scale))
    board.set_pin_mode_servo(7)
    board.servo_write(7, int((-90 + center) * scale))

    while(True):
        pin = 0
        num = input("select servo>")
        if num=="2":
            pin = 2
        elif num=="3":
            pin = 3
        elif num=="4":
            pin = 4
        elif num=="5":
            pin = 5
        elif num=="6":
            pin = 6
        elif num=="7":
            pin = 7
        else:
            print("Invalid servo num.")
            continue
        
        offset=int(input("set offset>"))
        value = int(input("set value>"))
        board.servo_write(pin, int((value + center + offset) * scale))
        print("...\n")

except KeyboardInterrupt:
    board.shutdown()
    sys.exit(0)
