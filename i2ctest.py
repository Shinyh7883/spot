
from adafruit_servokit import ServoKit
import board
import busio
import time

print("Initializing Servos")
i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
print("Initializing ServoKit")

kit = list()
kit.append(ServoKit(channels=16, i2c=i2c_bus0, address=0x40))

print("Done initializing")

val_list = [30, 60, 150, 120, 170, 170, 10, 10, 90, 90, 90, 90]
for i in range(len(val_list)):
            kit[0].servo[i].set_pulse_width_range(550,2600)

if __name__ == '__main__':
    for x in range(len(val_list)):
        print(x)
        kit[0].servo[x].angle = val_list[x]

    while True:
        # num is index of motor to rotate
        num=int(input("Enter Servo to rotate (0-11): "))
        
        # new angle to be written on selected motor
        angle=int(input("Enter new angles (0-180): "))
        prev_angle = val_list[num]
        sweep = range(prev_angle, angle, 1) if (prev_angle < angle) else range(prev_angle, angle, -1)
        for degree in sweep:
            kit[0].servo[num].angle=degree
            time.sleep(0.01)

        val_list[num] = angle