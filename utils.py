import numpy as np 
import time

# Working parameters for AK80-6 V1.0 firmware
PARAMS_V1 = {
    "P_MIN": -95.5,
    "P_MAX": 95.5,
    "V_MIN": -45.0,
    "V_MAX": 45.0,
    "KP_MIN": 0.0,
    "KP_MAX": 500,
    "KD_MIN": 0.0,
    "KD_MAX": 5.0,
    "T_MIN": -18.0,
    "T_MAX": 18.0,
    "AXIS_DIRECTION": -1
}

# 16-Bits for Raw Position Values
maxRawPosition = 2**16 - 1
# 12-Bits for Raw Velocity Values
maxRawVelocity = 2**12 - 1
maxRawTorque = 2**12 - 1                        # 12-Bits for Raw Torque Values
maxRawKp = 2**12 - 1                            # 12-Bits for Raw Kp Values
maxRawKd = 2**12 - 1                            # 12-Bits for Raw Kd Values
maxRawCurrent = 2**12 - 1                       # 12-Bits for Raw Current Values
dt_sleep = 0.0001                               # Time before motor sends a reply


def waitOhneSleep(dt):
    startTime = time.time()
    while time.time() - startTime < dt:
        pass


def float_to_uint(x, x_min, x_max, numBits):
    span = x_max - x_min
    offset = x_min
    # Attempt to speedup by using pre-computation. Not used currently.
    if numBits == 16:
        bitRange = maxRawPosition
    elif numBits == 12:
        bitRange = maxRawVelocity
    else:
        bitRange = 2**numBits - 1
    return int(((x - offset) * (bitRange)) / span)


def uint_to_float(x_int, x_min, x_max, numBits):
    span = x_max - x_min
    offset = x_min
    if numBits == 16:
        bitRange = maxRawPosition
    elif numBits == 12:
        bitRange = maxRawVelocity
    else:
        bitRange = 2**numBits - 1
    return ((x_int * span) / (bitRange)) + offset

