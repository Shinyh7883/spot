from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import board
import busio
import time

print("Initializing Servos")
i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
print("Initializing ServoKit")

pca = list()
pca.append(PCA9685(i2c_bus0, address=0x40))

print("Done initializing")

# [0]~[2] : FL // [3]~[5] : FR // [6]~[8] : RL // [9]~[11] : RR
val_list = [90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90]

servos = list()
for i in range(len(val_list)):
    servos.append(servo.Servo(pca[0].channels[i], min_pulse=1000, max_pulse=2000))
    # servos.append(servo.Servo(pca[int(i/6)].channels[int(i%6)], min_pulse=771, max_pulse=2740))


if __name__ == '__main__':
    for i in range(len(val_list)):
        servos[i].angle = val_list[i]

    while True:
        # num is index of motor to rotate
        num=int(input("Enter Servo to rotate (0-11): "))
        
        # new angle to be written on selected motor
        angle=int(input("Enter new angles (0-180): "))
        prev_angle = val_list[num]
        
        # increase(decrease) prev_angle to angle by 1 degree
        sweep = range(prev_angle, angle, 1) if (prev_angle < angle) else range(prev_angle, angle, -1)
        for degree in sweep:
            servos[num].angle=degree
            time.sleep(0.01)

        val_list[num] = angle