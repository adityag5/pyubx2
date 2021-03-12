
"""
Largely borrowed from UBX Examples
"""

from sys import platform
from io import BufferedReader
from threading import Thread
from time import sleep

from pyubx2 import UBXMessage, POLL, SET, UBX_CONFIG_MESSAGES
from pyubx2.ubxreader import UBXReader
from pyubx2.exceptions import UBXStreamError
from serial import Serial, SerialException, SerialTimeoutException

import pyubx2.exceptions as ube

import signal

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

    def stop_read_thread(self):
        """
        Stop the serial reader thread.
        """

        if self._serial_thread is not None:
            self._reading = False

    def send(self, data):
        """
        Send data to serial connection.
        """

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
    #ubp.set_nav()
    #ubp.ubx_proto_only() # Turn on UBX messages only.
  
    indefinite = True
    if(indefinite == False):
        sleep(20)
    else:
        while True:
            sleep(1)

    ubp.stop_read_thread()
    ubp.disconnect()
