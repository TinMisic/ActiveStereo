import sys
import time

print("Initializing pymata4...\n")
from pymata4 import pymata4

"""
This test is used to determine the offsets of the two shoulder sergos
"""


def servo(my_board, pin):
    """
    Set a pin to servo mode and then adjust
    its position.
    :param my_board: pymata4
    :param pin: pin to be controlled
    """

    # set the pin mode
    my_board.set_pin_mode_servo(pin)
    values = [0,90,180]
    scale=180/270
    offset=int(input("set offset>"))
    value = int(input("value>"))
    #values.reverse()

    while(True):
        my_board.servo_write(pin, int(value*scale + offset))
        print(value)
        #time.sleep(3)
        value = int(input("value>"))


board = pymata4.Pymata4()
center = 135
scale = 180/270

try:
    print('setting servos...')
##    board.set_pin_mode_servo(2)
##    board.set_pin_mode_servo(3)
##    board.set_pin_mode_servo(4)
##    board.set_pin_mode_servo(5)
##    board.set_pin_mode_servo(6)
##    board.set_pin_mode_servo(7)
    
    for i in range(2,8):
        board.set_pin_mode_servo(i)
        board.servo_write(i, int((0 + center) * scale))

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