from bitstring import main
from motorController import serialCAN
import numpy as np
import time

if __name__ == "__main__":
    can = serialCAN(port='COM10', canid=1) 
    # can.setZeroPosition()
    can.enterMotorMode()
    can.set(position=0, velocity=0, torque=0, kp=100, kd=1)
    time.sleep(2)
    # k = 36
    # for i in range(1, 2*k+1):
    for theta in np.arange(0.01, 2*np.pi/4, np.pi/360/10):
        target_pos = theta  # np.pi/k*i
        print('target', target_pos)
        for j in range(1):
            pos, speed, torque = can.set(position=target_pos+6e-3, velocity=0, torque=0, kp=100, kd=1)
        print('pos:{}, speed:{}, torque:{}'.format(pos, speed, torque))
        print(end='\n')
    # can.position = 1
    time.sleep(0.5)
    can.exitMotorMode()
    can.close()
    print(end='\n')
