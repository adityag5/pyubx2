import can
import codecs
import struct
import math
from decimal import Decimal
from can import listener
bus = can.interface.Bus(bustype='kvaser', channel=0, bitrate=500000)

bus.set_filters(filters=[{"can_id":0xaa, "can_mask":0xff}, {"can_id":0x127, "can_mask":0x1ff}])


a = can.BufferedReader()

while True:

    msg = bus.recv()
    a(msg)
    b = a.get_message()
    # # print(msg.timestamp)
    if b.arbitration_id == 0x127:
        # print(f'gearshift:{b.data.hex()}')
        t = b.data.hex()
        r = (int(t,16) >> 44 & 0x7)
        print(f'decoded gearshift is {r}')
        continue

    t = b.data.hex()

    time = b.timestamp

    # print(time)
    # print(t)
    
    # converted_time = hex(struct.unpack('I', struct.pack('<f', time))[0])
    converted_time = hex(round(Decimal(time*1000)) - 1626800000000) 

    converted_time = converted_time[2:]

    chunks2 = [converted_time[i:i+2] for i in range(0, len(converted_time), 2)]

    # print(chunks2)

    # print((int(chunks2[0],16)))


    # n = 4
    # chunks = [t[i:i+n] for i in range(0, len(t), n)]

    # wheel_speed_rl = int(chunks[len(chunks)-2], 16)
    # wheel_speed_rr = int(chunks[len(chunks)-1], 16)
        
    # scale = 0.01
    # valueOffset = -67.669998

    # decoded_wheel_speed_rl = wheel_speed_rl*scale + valueOffset
    # decoded_wheel_speed_rr = wheel_speed_rr*scale + valueOffset

    # decoded_wheel_speed_rl = (decoded_wheel_speed_rl * 5/18) * 1000 # convert to m/s then scale up for ublox units
    # decoded_wheel_speed_rr = (decoded_wheel_speed_rr * 5/18) * 1000 # convert to m/s then scale up for ublox units


    # p = []
    # p.append(0x00)
    # p.append(int(chunks2[0], 16))
    # p.append(int(chunks2[1], 16))
    # p.append(int(chunks2[2], 16))

    #             # print(converted_time)
    # # p.append(int(converted_time,16))
    #             # p.extend([0x00,0x00,0x00,0x00]) # CHANGE THIS TO TAKE TIMESTAMP
    #             # p.append(0x00)
    # p.append((0x00)) # flags
    # p.append((0x08)) # flags
    # p.append((0x00)) # id
    # p.append((0x00)) # id
    # p.extend([0x00,0x00,0x00,0x0b])
    #             # p.append((0x0b)) # data type
    #             # p.append((math.trunc(decoded_wheel_speed_rl)))
    #             # p.append((math.trunc(decoded_wheel_speed_rr)))

    # s = b''
    # for i in p:
        
    #     s += struct.pack('!B',i) # conversion to packed binary data

    # print(s)
a.stop()

