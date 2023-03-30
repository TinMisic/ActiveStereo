import pyfirmata

scaling = 180/270
offset = 135
offsets = [x+offset for x in [2, 7, -4, -4]]

board = pyfirmata.Arduino('/dev/ttyUSB0')

# for servo in range(2,6):
#     board.servo_config(servo,min_pulse=544,max_pulse=2400,angle=offset*scaling)

l1 = board.get_pin('d:2:s')
l1.write(offsets[0]*scaling)
l2 = board.get_pin('d:3:s')
l2.write(offsets[1]*scaling)
r1 = board.get_pin('d:4:s')
r1.write(offsets[2]*scaling)
r2 = board.get_pin('d:5:s')
r2.write(offsets[3]*scaling)

servos = [l1, l2, r1, r2]

while True:
    i = int(input("Servo index:"))
    angle = float(input("Angle:"))

    servos[i].write((angle+offsets[i])*scaling)

