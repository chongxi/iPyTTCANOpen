from bitstring import main
from motorController import motorController
import numpy as np
import time

if __name__ == "__main__":
    motor1 = motorController(port='COM10', canid=1)
    # motor1.setZeroPosition()
    motor1.enterMotorMode()
    motor1.set(position=0, velocity=0, torque=0, kp=8, kd=0.5)
    time.sleep(2)
    for theta in np.arange(0.01, 2*np.pi/4, np.pi/360/10):
        target_pos = theta  # np.pi/k*i
        print('target', target_pos)
        for j in range(1):
            pos, speed, torque = motor1.set(position=target_pos+6e-3, velocity=0, torque=0, kp=8, kd=0.5)
        print('pos:{}, speed:{}, torque:{}'.format(pos, speed, torque))
        print(end='\n')
    time.sleep(0.5)
    motor1.position = 0
    time.sleep(0.5)
    motor1.exitMotorMode()
    motor1.close()
    print(end='\n')
