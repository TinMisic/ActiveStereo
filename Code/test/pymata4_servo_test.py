import sys
import time

print("Initializing pymata4...\n")
from pymata4 import pymata4

"""
This example will set a servo to 0, 90 and 180 degree
positions.
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

try:
    servo(board, 3)
    #print('setting servos...')
    #board.set_pin_mode_servo(2)
    #board.set_pin_mode_servo(3)
    #print('writing...')
    #board.servo_write(2, 180)
    #board.servo_write(3, 180)
except KeyboardInterrupt:
    board.shutdown()
    sys.exit(0)