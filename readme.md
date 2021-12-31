# What is this project about
This python code for PC to send CAN command to control [MIT Cheetah actuator](https://www.youtube.com/watch?v=ecSQZlNda6g). It is built on top of:
- [TTCANOPEN](https://www.ttcanopen.com/) USB2CAN hardware and protocol
- MIT Cheetah CAN API (check Motor Driver Documentation.pdf)

MIT Cheetah driver uses standard frame format (11-bit ID) not extended. The CAN bus runs at 1MBaud = 1kbps. The MIT driver receives 8 bytes of command within the DATA field of the CAN packet, and sends back 6 bytes of data. 

# Hardware required to run the code
- TTCANOPEN USB2CAN PCB
- MIT Cheetah Actuator (https://store.tmotor.com/category.php?id=97)
- JST-GH 2 pin CAN cable
- 24V DC power supply with 20A current capacity (or more)

# Python dependencies
- Python 3.8
- [PySerial](https://pypi.org/project/pyserial/)
- [bitstring](https://pypi.org/project/bitstring/)

# How the PC communicate to the MIT driver
With TTCANOPEN, the PC can send CAN packet to the MIT driver via `serial port`. Here the `pyserial` is used. The MIT driver will send back the response packet. The PC can then parse the response packet to get the data. 

# How to choose the serial port buadrate
The correnspendonce between serial Baudrate and CAN baudrate is:

| Baudrate | CAN baudrate |
|:--------:|:------------:|
| 9600 | 10 kbps |
| 14400 | 20 kbps |
| 19200 | 50 kbps |
| 38400 | 100 kbps |
| 56000 | 125 kbps |
| 57600 | 250 kbps |
| 115200 | 500 kbps |
| 128000 | 800 kbps |
| 256000 | 1000 kbps (default) |

(256000 serial port baudrate is default for this project since MIT Cheetah actuator CAN bus is running at 1000kbps or 1MBaud)

# What is the packet structure
- send: 
enter motor mode command for example:

    | CAN packet (each section is one byte) |
    |:----------:|
    | start (0xAA)
    | CAN frame type (0x55) |
    | CAN ID high (0x00) |
    | CAN ID low (0x01) |
    | data length (0x08) |
    | data (0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC) |
    | end (0xDD) |

Note: 
- The CAN ID is 11-bit ID. In the PC it takes two bytes, so the ID 01 is `0x0001`. The USB2CAN hardware will send the CAN packet with 11-bit ID since the frame type `0x55` indicates standard frame. 
- The data is 8 bytes (`0x08` data length), it contains the content that will be received by the MIT driver.
- The last byte is always 0xDD.

The full packet in python is represented by a list:
```
[0xAA, 0x55, 0x00, 0x01, 0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC，0xDD]
```

The list needs to be convert to bytes (hex) before sending by the serial port. `serial.to_bytes()` can be used to convert the CAN packet (a python list) to bytes.

<!-- - receive (TODO) -->

# How to construct the 8 byte data field

The MIT driver put 5 variables used in the PID controller loop in the 8 byte data field:
- 16 bit motor position, scaled between `P_MAX` and `P_MIN` in `CAN_COM.cpp`
- 12 bit velocity, scaled between `V_MAX` and `V_MIN` in `CAN_COM.cpp`
- 12 bit KP
- 12 bit KD
- 12 bit feedforward current for exerting torque

16+12+12+12+12 = 64 bits = 8 bytes

In `CAN_COM.cpp`, the data field is constructed as:
```C
 #define P_MIN -95.5f
 #define P_MAX 95.5f
 #define V_MIN -45.0f
 #define V_MAX 45.0f
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define T_MIN -18.0f
 #define T_MAX 18.0f


/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]
void pack_reply(CANMessage *msg, float p, float v, float t){
    int p_int = float_to_uint(p, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v, V_MIN, V_MAX, 12);
    int t_int = float_to_uint(t, -T_MAX, T_MAX, 12);
    msg->data[0] = CAN_ID;
    msg->data[1] = p_int>>8;
    msg->data[2] = p_int&0xFF;
    msg->data[3] = v_int>>4;
    msg->data[4] = ((v_int&0xF)<<4) + (t_int>>8);
    msg->data[5] = t_int&0xFF;
    }
    
/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]
void unpack_cmd(CANMessage msg, ControllerStruct * controller){
        int p_int = (msg.data[0]<<8)|msg.data[1];
        int v_int = (msg.data[2]<<4)|(msg.data[3]>>4);
        int kp_int = ((msg.data[3]&0xF)<<8)|msg.data[4];
        int kd_int = (msg.data[5]<<4)|(msg.data[6]>>4);
        int t_int = ((msg.data[6]&0xF)<<8)|msg.data[7];
        
        controller->p_des = uint_to_float(p_int, P_MIN, P_MAX, 16);
        controller->v_des = uint_to_float(v_int, V_MIN, V_MAX, 12);
        controller->kp = uint_to_float(kp_int, KP_MIN, KP_MAX, 12);
        controller->kd = uint_to_float(kd_int, KD_MIN, KD_MAX, 12);
        controller->t_ff = uint_to_float(t_int, T_MIN, T_MAX, 12);
    //printf("Received   ");
    //printf("%.3f  %.3f  %.3f  %.3f  %.3f   %.3f", controller->p_des, controller->v_des, controller->kp, controller->kd, controller->t_ff, controller->i_q_ref);
    //printf("\n\r");
    }
```

Therefore, in python we should construct the 8 byte data field as:
```python
from bitstring import BitArray

self._p_des_BitArray = BitArray(uint=float_to_uint(0, self.motorParams['P_MIN'],
                                                self.motorParams['P_MAX'], 16), length=16)
self._v_des_BitArray = BitArray(uint=float_to_uint(0, self.motorParams['V_MIN'],
                                                self.motorParams['V_MAX'], 12), length=12)
self._kp_BitArray = BitArray(uint=0, length=12)
self._kd_BitArray = BitArray(uint=0, length=12)
self._tau_BitArray = BitArray(uint=0, length=12)
self._cmd_bytes = BitArray(uint=0, length=64) # 8 bytes
self._recv_bytes = BitArray(uint=0, length=48) # 6 bytes
```

# How to use this project
Just run the code and follow the instruction.
