import serial
from bitstring import BitArray
from utils import *
import time


class serialCAN():
    """
    Use TTCANOpen protocol to communicate with the MIT cheetah actuator CAN bus (via serial port), using pyserial here
    baudrate: 256000 (256000 serial baudrate ~ 1000kbps CAN baudrate)
    parity: 'O' (odd)
    timeout: 0 (non-blocking)
    """
    def __init__(self, port, canid=1, baudrate=256000, parity='O', timeout=0.05):
        self.port = port
        self.canid = canid
        self.baudrate = baudrate
        self.timeout = timeout
        self.parity = parity
        self.ser = serial.Serial(port, baudrate=baudrate, parity=parity, timeout=timeout)
        self.check_handshake_TTCANOpen() # this is necessary in case the selected serial port is not the TTCANOpen port 

    def check_handshake_TTCANOpen(self):
        '''
        TTCANOpen protocol sends back a long advertisement message whenever baudrate is changed.
        '''
        self.ser.baudrate = 9600
        self.ser.baudrate = self.baudrate
        waitOhneSleep(0.5)
        acq = self.ser.readlines()
        if acq[-2] == b'AA CC 18765432 08 11 22 33 44 55 66 77 88 DD\r\n':
            self.is_connected = True
        else:
            self.is_connected = False
        
    def send(self, message):
        self.ser.write(message)

    @property
    def prefix(self):
        '''
        prefix attached to all commands (TTCANOpen protocol)
        '''
        _prefix = serial.to_bytes([0xAA, 0x55, 0x00, self.canid, 0x08])  # - Header of TTCANOpen protocol for standard CAN command
        self._prefix = BitArray(_prefix)
        return self._prefix

    def enterMotorMode(self):
        cmd = serial.to_bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, # - MIT Chetah CAN command (8 Bytes)
                               0xDD])                                          # - TTCANOpen end flag (0xDD)
        self.cmd = BitArray(cmd)
        self.cmd.hex = self.prefix.hex + self.cmd.hex # order is important here; don;t use +=
        self.send(self.cmd.bytes)
        ack = self.ser.read(16)
        if len(ack)>0:
            return True

    def exitMotorMode(self):
        cmd = serial.to_bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD, 
                               0xDD])
        self.cmd = BitArray(cmd)
        # order is important here; don;t use +=
        self.cmd.hex = self.prefix.hex + self.cmd.hex
        self.send(self.cmd.bytes)
        ack = self.ser.read(16)
        if len(ack) > 0:
            return True

    def setZeroPosition(self):
        cmd = serial.to_bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 
                               0xDD])
        self.cmd = BitArray(cmd)
        self.cmd.hex = self.prefix.hex + self.cmd.hex
        self.send(self.cmd.bytes)

    def set(self, position, velocity=0, torque=0, kp=10, kd=0.5):
        self._position = position
        self._velocity = velocity
        self._kd = kd
        self._kp = kp
        self._torque = torque
        pos = BitArray(uint=float_to_uint(position, PARAMS['P_MIN'], PARAMS['P_MAX'], 16), 
                       length=16)
        vel = BitArray(uint=float_to_uint(velocity, PARAMS['V_MIN'], PARAMS['V_MAX'], 12),length=12)
        tor = BitArray(uint=float_to_uint(torque, PARAMS['T_MIN'], PARAMS['T_MAX'], 12), length=12)
        kp = BitArray(uint=float_to_uint(kp, PARAMS['KP_MIN'], PARAMS['KP_MAX'], 12), length=12)
        kd = BitArray(uint=float_to_uint(kd, PARAMS['KD_MIN'], PARAMS['KD_MAX'], 12), length=12)

        self.cmd.hex = self.prefix.hex + pos.hex + vel.hex + kp.hex + kd.hex + tor.hex + self.cmd.hex[-2:]
        
        # for i in range(5):
        self.send(self.cmd.bytes)
        # waitOhneSleep(0.001)
        self._last_read = BitArray(self.ser.read(16))
        self._read_position_rad = uint_to_float(self._last_read[4*20:4*24].uint, PARAMS['P_MIN'], PARAMS['P_MAX'], 16)
        self._read_speed_rad = uint_to_float(self._last_read[4*24:4*27].uint, PARAMS['V_MIN'], PARAMS['V_MAX'], 12)
        self._read_torque = uint_to_float(self._last_read[4*27:4*30].uint, PARAMS['T_MIN'], PARAMS['T_MAX'], 12)
        return self._read_position_rad, self._read_speed_rad, self._read_torque
    
    def refresh(self):
        '''
        set the last command to read out the status
        '''
        self.set(self._position, self._velocity, self._torque, self._kp, self._kd)

    @property
    def position(self):
        # return uint_to_float(self.cmd.uint[10:22], PARAMS['P_MIN'], PARAMS['P_MAX'], 16)
        return self._position

    @property
    def velocity(self):
        # return uint_to_float(self.cmd.uint[22:34], PARAMS['V_MIN'], PARAMS['V_MAX'], 12)
        return self._velocity
    
    @property
    def kp(self):
        return self._kp

    @property
    def kd(self):
        return self._kd
    
    @property
    def torque(self):
        return self._torque

    @position.setter
    def position(self, value):
        self._position = value
        self.set(position=self._position, velocity=self._velocity,
                 torque=self._torque, kp=self._kp, kd=self._kd)
    
    @velocity.setter
    def velocity(self, value):
        self._velocity = value
        self.set(position=self._position, velocity=self._velocity,
                 torque=self._torque, kp=self._kp, kd=self._kd)
    
    @torque.setter
    def torque(self, value):
        self._torque = value
        self.set(position=self._position, velocity=self._velocity,
                 torque=self._torque, kp=self._kp, kd=self._kd)
    
    @kp.setter
    def kp(self, value):
        self._kp = value
        self.set(position=self._position, velocity=self._velocity,
                 torque=self._torque, kp=self._kp, kd=self._kd)
    
    @kd.setter
    def kd(self, value):
        self._kd = value
        self.set(position=self._position, velocity=self._velocity,
                 torque=self._torque, kp=self._kp, kd=self._kd)

    def close(self):
        self.ser.close()
