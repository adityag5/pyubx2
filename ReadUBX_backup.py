
"""
Largely borrowed from UBX Examples
"""

from sys import platform
from io import BufferedReader
from threading import Thread
from time import sleep

from pyubx2 import UBXMessage, POLL, SET#, UBX_CONFIG_MESSAGES
from pyubx2.ubxreader import UBXReader
from pyubx2.exceptions import UBXStreamError
from serial import Serial, SerialException, SerialTimeoutException

import pyubx2.exceptions as ube

import signal
import math
import can
import random
import struct

from decimal import Decimal

import ntripclient as nc

class UBXStreamer:
    """
    UBXStreamer class.
    """

    def __init__(self, port, baudrate, timeout=5):
        """
        Constructor.
        """

        self._serial_object = None
        self._serial_thread = None
        self._ubxreader = None
        self._connected = False
        self._reading = False
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._read_can = False
        self._read_rtcm = False

    def __del__(self):
        """
        Destructor.
        """

        self.stop_read_thread()
        self.disconnect()

    def connect(self):
        """
        Open serial connection.
        """

        try:
            self._serial_object = Serial(
                self._port, self._baudrate, timeout=self._timeout
            )
            self._ubxreader = UBXReader(
                BufferedReader(self._serial_object), False)
            self._connected = True
        except (SerialException, SerialTimeoutException) as err:
            print(f"Error connecting to serial port {err}")

    def disconnect(self):
        """
        Close serial connection.
        """

        if self._connected and self._serial_object:
            try:
                self._serial_object.close()
            except (SerialException, SerialTimeoutException) as err:
                print(f"Error disconnecting from serial port {err}")
        self._connected = False

    def start_read_thread(self):
        """
        Start the serial reader thread.
        """

        if self._connected:
            self._reading = True
            self._serial_thread = Thread(target=self._read_thread, daemon=True)
            self._serial_thread.start()

    def start_can_thread(self):

        if self._connected:
            self._read_can = True
            self._serial_thread = Thread(target=self._can_thread, daemon=True)
            self._serial_thread.start()
    
    def start_rtcm_thread(self):

        if self._connected:
            self._read_rtcm = True
            self._serial_thread = Thread(target=self._rtcm_thread, daemon=True)
            self._serial_thread.start()

    def stop_read_thread(self):
        """
        Stop the serial reader thread.
        """

        if self._serial_thread is not None:
            self._reading = False

    def stop_can_thread(self):

        if self._serial_object is not None:
            self._read_can = False
    
    def stop_rtcm_thread(self):

        if self._serial_object is not None:
            self._read_rtcm = False

    def send(self, data):
        """
        Send data to serial connection.
        """
        print(f"Sending this {data}")
        self._serial_object.write(data)

    def flush(self):
        """
        Flush input buffer
        """

        self._serial_object.reset_input_buffer()

    def waiting(self):
        """
        Check if any messages remaining in the input buffer
        """

        return self._serial_object.in_waiting

    def _read_thread(self):
        """
        THREADED PROCESS
        Reads and parses UBX message data from stream
        """

        while self._reading and self._serial_object:
            if self._serial_object.in_waiting:
                try:
                    (raw_data, parsed_data) = self._ubxreader.read()
                    #if raw_data:
                    #    print(raw_data)
                    if parsed_data:
                        print(parsed_data)
                except (
                    ube.UBXStreamError,
                    ube.UBXMessageError,
                    ube.UBXTypeError,
                    ube.UBXParseError,
                ) as err:
                    print(f"Something went wrong {err}")
                    continue

    def _rtcm_thread(self):

        username = 'tr+zen01636701'
        password = '65152416'
        caster = 'ca.smartnetna.com'
        port = 9950
        mountpoint = 'MSM_VRS_ITRF14'
        lat = 37.85917252094228
        lon = -122.29157635549005
        height = 14.0208

        ntripArgs = {}
        ntripArgs['lat'] = lat
        ntripArgs['lon'] = lon 
        ntripArgs['height'] = height
        ntripArgs['user'] = username + ":" + password
        ntripArgs['caster'] = caster
        ntripArgs['port'] = port 
        ntripArgs['mountpoint'] = mountpoint

        if ntripArgs['mountpoint'][0:1] !="/":
            ntripArgs['mountpoint'] = "/"+ntripArgs['mountpoint']

        ntripArgs['V2']= True

        n = nc.NtripClient(**ntripArgs)

        while self._read_rtcm and self._serial_object:
            try:
                data = n.read()

                if data is None:
                    continue
    
                # print(f'send this {data}')
                print(type(data))
                self.send(data)            
            except:
                print("RTCM Read/Write error")
                continue

    def _can_thread(self):

        bus = can.interface.Bus(bustype='kvaser', channel=0, bitrate=500000)

        bus.set_filters(filters=[{"can_id":0xaa, "can_mask":0xff}])#, {"can_id":0x127, "can_mask":0x1ff}])

        a = can.BufferedReader()
        

        while self._read_can and self._serial_object:
            try:
            # Add contents from can_read.py for reading and decoding
                
                msg = bus.recv() # Reading from the bus
                a(msg) # Call listener
                b = a.get_message() # Retrieve latest message
                
                if b.arbitration_id != 0xaa: # To filter out 0x1aa
                    continue
                
                time = b.timestamp # Timestamp (s) in floating point 
                # print(time)
                converted_time = hex(round(Decimal(time*1000)) & 0xFFFFFF) 
                # print(f'corttr {converted_time}')
                converted_time = converted_time[2:]
                chunks2 = [converted_time[i:i+2] for i in range(0, len(converted_time), 2)]

                t = b.data.hex() # Data in hex
                n = 4
                chunks = [t[i:i+n] for i in range(0, len(t), n)]

                wheel_speed_rl = int(chunks[len(chunks)-2], 16)
                wheel_speed_rr = int(chunks[len(chunks)-1], 16)
        
                scale = 0.01
                valueOffset = -67.669998

                decoded_wheel_speed_rl = wheel_speed_rl*scale + valueOffset # in km/h
                decoded_wheel_speed_rr = wheel_speed_rr*scale + valueOffset # in km/h
                # print(f'wheel speed is {decoded_wheel_speed_rl}')

                decoded_wheel_speed_rl = (decoded_wheel_speed_rl * 5/18) * 1000 # convert to m/s then scale up for ublox units
                decoded_wheel_speed_rr = (decoded_wheel_speed_rr * 5/18) * 1000 # convert to m/s then scale up for ublox units
                # print(f'wheel speed is {decoded_wheel_speed_rl}')

                average_wheel_speed = (decoded_wheel_speed_rl+decoded_wheel_speed_rr)/2
                
                p = []
                p.append(int(chunks2[2], 16))
                p.append(int(chunks2[1], 16))
                p.append(int(chunks2[0], 16))
                p.append(0x00)

                p.append((0x00)) # flags
                p.append((0x08)) # flags
                p.append((0x00)) # id
                p.append((0x00)) # id
                
                p.extend(struct.pack('>I', math.trunc(average_wheel_speed))[1:])
                
                p.append((0x0b)) # data type
                
                s = b''
                for i in p:
                    s += struct.pack('!B',i) # conversion to packed binary data
                
                msg = UBXMessage('ESF', 'ESF-MEAS', SET, payload=s)
                
                # print(f"Sending {msg}")
                # print("Sending, ", self.dump(msg.serialize()))
                self.send(msg.serialize())                

            except:
                print("CAN Read/Write error")
                continue
            
        a.stop()

    def dump(self, x):
        return ''.join([type(x).__name__, "('",
        *['\\x'+'{:02x}'.format(i) for i in x], "')"])

    def ubx_proto_only(self):
        """
        Creates a CFG-PRT configuration message and
        sends it to the receiver.
        """
        NMEA = b"\x02\x00"
        UBX = b"\x01\x00"
        BOTH = b"\x03\x00"
        try:
            msg = UBXMessage(
                "CFG",
                "CFG-PRT",
                SET,
                portID=3,
                baudRate=0,
                inProtoMask=b"\x07\x00",
                outProtoMask=UBX,
            )
            print(f"Sending {msg}")
            self.send(msg.serialize())
        except (ube.UBXMessageError, ube.UBXTypeError, ube.UBXParseError) as err:
            print(f"Something went wrong {err}")

    def set_nav(self):
        """
        Set the current Navigation configuration
        """

        transaction = 0 # Immediate application
        layers = 3 # Volatile RAM only.
                   # 2 - Battery Backed
                   # 3 - Flash
        
        cfgData = [
                   #("CFG_SFIMU_IMU_AUTO_MNTALG_ENA", 1), # Enable auto-IMU mounting alignment
                   # Start driving at a minimum speed of 30 km/h and do 
                   # a series of approximately 10 left and right turns (at
                   # least 90 degrees).
                   ("CFG_SFIMU_IMU_AUTO_MNTALG_YAW", 158),
                   ("CFG_SFIMU_IMU_AUTO_MNTALG_PITCH", 86),
                   ("CFG_SFIMU_IMU_AUTO_MNTALG_ROLL", 191),
                   ("CFG_RATE_MEAS", 100), # Measurement rate once every x ms
                   ("CFG_RATE_NAV", 1),    # 1 navigation solution for every measurement
                   ("CFG_RATE_NAV_PRIO", 10), # 30 Hz navigation priority messages
                   ("CFG_MSGOUT_UBX_NAV_STATUS_USB", 1), # Navigation Status
                   ("CFG_MSGOUT_UBX_NAV_PVT_USB", 1), # PVT solution
                   ("CFG_MSGOUT_UBX_NAV_PVT_UART1", 1), # PVT solution
                   ("CFG_MSGOUT_UBX_NAV_HPPOSECEF_USB", 1), # High Precision ECEF position
                   ("CFG_MSGOUT_UBX_NAV_POSECEF_USB", 1), # Regular precision ECEF position
                   ("CFG_MSGOUT_UBX_NAV_VELECEF_USB", 1), # Velocity in ECEF
                   ("CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB", 1), # High precision LLH position
                   ("CFG_MSGOUT_UBX_NAV_POSLLH_USB", 1), # High precision LLH position
                   ("CFG_MSGOUT_UBX_NAV_ATT_USB", 1), # Attitude of the vehicle
                   ("CFG_MSGOUT_UBX_NAV_COV_USB", 1), # Covariance matricies
                   ("CFG_MSGOUT_UBX_NAV_TIMEGPS_USB", 1), # GPS Time more explicitly called out
                   ("CFG_MSGOUT_UBX_ESF_INS_USB", 1), # Compensated angular rate and acceleration
                   ("CFG_MSGOUT_UBX_ESF_STATUS_USB", 1), # External Sensor Fusion status
                   ("CFG_MSGOUT_UBX_ESF_ALG_USB", 1), #Alignment of the sensors
                   ("CFG_MSGOUT_UBX_ESF_MEAS_USB", 1), # Measurements of the sensors
                   
                   ("CFG_MSGOUT_UBX_ESF_RAW_USB", 0),
                #    ("CFG_MSGOUT_UBX_NAV_SIG_UART1", 1),
                #    ("CFG_MSGOUT_UBX_NAV_SIG_USB", 1),
                #    ("CFG_MSGOUT_UBX_NAV_POSLLH_UART1", 1),
                #    ("CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1", 1),
                #    ("CFG_MSGOUT_UBX_NAV_RELPOSNED_USB", 1),
                   ("CFG_MSGOUT_UBX_NAV_STATUS_USB", 1),
                #    ("CFG_MSGOUT_UBX_NAV_STATUS_UART1", 1),

                   # Disable all the other default interfaces so we can actually get our 30Hz!
                   ("CFG_MSGOUT_NMEA_ID_GGA_I2C", 0),
                   ("CFG_MSGOUT_NMEA_ID_GGA_SPI", 0),
                   ("CFG_MSGOUT_NMEA_ID_GGA_UART1", 0),
                   ("CFG_MSGOUT_NMEA_ID_GGA_UART2", 0),
                   ("CFG_MSGOUT_NMEA_ID_GGA_USB", 0),

                   ("CFG_MSGOUT_NMEA_ID_GLL_I2C", 0),
                   ("CFG_MSGOUT_NMEA_ID_GLL_SPI", 0),
                   ("CFG_MSGOUT_NMEA_ID_GLL_UART1", 0),
                   ("CFG_MSGOUT_NMEA_ID_GLL_UART2", 0),
                   ("CFG_MSGOUT_NMEA_ID_GLL_USB", 0),

                   ("CFG_MSGOUT_NMEA_ID_GSA_I2C", 0),
                   ("CFG_MSGOUT_NMEA_ID_GSA_SPI", 0),
                   ("CFG_MSGOUT_NMEA_ID_GSA_UART1", 0),
                   ("CFG_MSGOUT_NMEA_ID_GSA_UART2", 0),
                   ("CFG_MSGOUT_NMEA_ID_GSA_USB", 0),

                   ("CFG_MSGOUT_NMEA_ID_GSV_I2C", 0),
                   ("CFG_MSGOUT_NMEA_ID_GSV_SPI", 0),
                   ("CFG_MSGOUT_NMEA_ID_GSV_UART1", 0),
                   ("CFG_MSGOUT_NMEA_ID_GSV_UART2", 0),
                   ("CFG_MSGOUT_NMEA_ID_GSV_USB", 0),

                   ("CFG_MSGOUT_NMEA_ID_RMC_I2C", 0),
                   ("CFG_MSGOUT_NMEA_ID_RMC_SPI", 0),
                   ("CFG_MSGOUT_NMEA_ID_RMC_UART1", 0),
                   ("CFG_MSGOUT_NMEA_ID_RMC_UART2", 0),
                   ("CFG_MSGOUT_NMEA_ID_RMC_USB", 0),

                   ("CFG_MSGOUT_NMEA_ID_VTG_I2C", 0),
                   ("CFG_MSGOUT_NMEA_ID_VTG_SPI", 0),
                   ("CFG_MSGOUT_NMEA_ID_VTG_UART1", 0),
                   ("CFG_MSGOUT_NMEA_ID_VTG_UART2", 0),
                   ("CFG_MSGOUT_NMEA_ID_VTG_USB", 0),
# 
                #    ("CFG_SFODO_USE_WT_PIN", 0),
                #    ("CFG_SFODO_USE_SPEED", 1),
                #    ("CFG_SFODO_FREQUENCY", 70),
                #    ("CFG_I2COUTPROT_UBX", 0),
                #    ("CFG_I2COUTPROT_NMEA", 0),

                #    ("CFG_MSGOUT_UBX_RXM_RTCM_USB", 1),
                   ("CFG_USBINPROT_RTCM3X",1), 
                   ("CFG_MSGOUT_UBX_MON_COMMS_USB", 1),
                #    ("CFG_MSGOUT_UBX_NAV_SAT_USB", 1)
                   ]
        
        msg = UBXMessage.config_set(layers, transaction, cfgData)
        ubp.send(msg.serialize())

if __name__ == "__main__":

    # set PORT, BAUDRATE and TIMEOUT as appropriate
    PORT = "/dev/ttyACM0"
    BAUDRATE = 912600
    TIMEOUT = 1

    ubp = UBXStreamer(PORT, BAUDRATE, TIMEOUT)
    ubp.connect()
    ubp.start_read_thread()

    # Read out the product ID and information as a basic check
    # msg = UBXMessage("CFG", "CFG-USB", POLL)
    # ubp.send(msg.serialize())
    # sleep(1) # Wait for the response

    #msg = UBXMessage("MON", "MON-VER", POLL)
    #ubp.send(msg.serialize())
    #sleep(1) # Wait for the response

    # Set up navigation the way we'd want. Turned off to not reset it every time when it's in flash.
    ubp.set_nav()
    ubp.ubx_proto_only() # Turn on UBX messages only.

    
    ubp.start_can_thread() # TO TEST
    # ubp.start_rtcm_thread() # TO TEST

    indefinite = True
    if(indefinite == False):
        sleep(20)
    else:
        while True:
            sleep(1)

    # ubp.stop_rtcm_thread() # TO TEST

    ubp.stop_can_thread() # TO TEST

    ubp.stop_read_thread()
    ubp.disconnect()