#!/usr/bin/env -S python3 -u
"""
This is heavily based on the NtripPerlClient program written by BKG.
Then heavily based on a unavco original.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

"""

from os import error
import socket
import sys
import datetime
import base64
import time
import math
import serial
import errno

from optparse import OptionParser


version=0.2
useragent="NTRIP JCMBsoftPythonClient/%.1f" % version

# reconnect parameter (fixed values):
factor=2 # How much the sleep time increases with each failed attempt
maxReconnect=1
maxReconnectTime=1200
sleepTime=1 # So the first one is 1 second



class NtripClient(object):
    def __init__(self,
                 user="",
                 port=2101,
                 caster="",
                 mountpoint="",
                 host=False,
                 lat=46,
                 lon=122,
                 height=1212,
                 ssl=False,
                 V2=False,
                 ):
        self.user = base64.b64encode(bytes(user,'utf-8')).decode("utf-8")
        self.port=port
        self.caster=caster
        self.mountpoint=mountpoint
        self.setPosition(lat, lon)
        self.height=height
        self.ssl=ssl
        self.host=host
        self.V2=V2
        self.found_header = False
        self.sent_header = False
        self.socket=None
        self.last_id = None


    def setPosition(self, lat, lon):
        self.flagN="N"
        self.flagE="E"
        if lon>180:
            lon=(lon-360)*-1
            self.flagE="W"
        elif (lon<0 and lon>= -180):
            lon=lon*-1
            self.flagE="W"
        elif lon<-180:
            lon=lon+360
            self.flagE="E"
        else:
            self.lon=lon
        if lat<0:
            lat=lat*-1
            self.flagN="S"
        self.lonDeg=int(lon)
        self.latDeg=int(lat)
        self.lonMin=(lon-self.lonDeg)*60
        self.latMin=(lat-self.latDeg)*60

    def getMountPointBytes(self):
        mountPointString = "GET %s HTTP/1.1\r\nUser-Agent: %s\r\nAuthorization: Basic %s\r\n" % (self.mountpoint, useragent, self.user)
#        mountPointString = "GET %s HTTP/1.1\r\nUser-Agent: %s\r\n" % (self.mountpoint, useragent)
        if self.host or self.V2:
            hostString = "Host: %s:%i\r\n" % (self.caster,self.port)
            mountPointString+=hostString
        if self.V2:
            print("Version 2")
            mountPointString+="Ntrip-Version: Ntrip/2.0\r\n"
        mountPointString+="\r\n"
        # if self.verbose:
        print (mountPointString)
        return bytes(mountPointString,'ascii')

    def getGGABytes(self):
        now = datetime.datetime.utcnow()
        ggaString= "GPGGA,%02d%02d%04.2f,%02d%011.8f,%1s,%03d%011.8f,%1s,1,05,0.19,+00400,M,%5.3f,M,," % \
            (now.hour,now.minute,now.second,self.latDeg,self.latMin,self.flagN,self.lonDeg,self.lonMin,self.flagE,self.height)
        checksum = self.calcultateCheckSum(ggaString)
        # if self.verbose:
        print  ("$%s*%s\r\n" % (ggaString, checksum))
        return bytes("$%s*%s\r\n" % (ggaString, checksum),'ascii')

    def calcultateCheckSum(self, stringToCheck):
        xsum_calc = 0
        for char in stringToCheck:
            xsum_calc = xsum_calc ^ ord(char)
        return "%02X" % xsum_calc

    def connect(self):
        self.sent_header = False
        self.found_header = False
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            error_indicator = sock.connect_ex((self.caster, self.port))
        except:
            return False
        
        if error_indicator == 0:
            sock.setblocking(0)
            self.socket = sock
            return True
        return False

    def get_ID(self):
        return self.last_id

    def read(self):

        if self.socket is None:
            time.sleep(0.1)
            self.connect()
            return None
        
        if not self.found_header:
            if not self.sent_header:
                self.sent_header = True
                time.sleep(0.1)
                mps = self.getMountPointBytes()

                try:
                    self.socket.sendall(mps)
                except:
                    self.socket = None
                    return None

            try:
                casterResponse = self.socket.recv(4096) #All the data
                # print(casterResponse)
                        # print('I AM HERE')
            except IOError as e:
                if e.errno == errno.EWOULDBLOCK:
                    return None
                self.socket =  None
                casterResponse = ''

            header_lines = casterResponse.decode('utf-8').split("\r\n")
            print(header_lines)
            for line in header_lines:
                if line == "":
                    self.found_header = True
                if line.find("SOURCETABLE") != -1:
                    sys.stderr.write("Mount point does not exist")
                    sys.exit(1)                
                elif line.find("401 Unauthorized") != -1:
                    sys.stderr.write("Unauthorized request\n")
                    sys.exit(1)
                elif line.find("404 Not Found") != -1:
                    sys.stderr.write("Mount Point does not exist\n")
                    sys.exit(2)
                elif line.find("ICY 200 OK")>=0:
                    #Request was valid
                    self.socket.sendall(self.getGGABytes())
                elif line.find("HTTP/1.0 200 OK")>=0:
                    #Request was valid
                    self.socket.sendall(self.getGGABytes())
                elif line.find("HTTP/1.1 200 OK")>=0:
                    #Request was valid
                    self.socket.sendall(self.getGGABytes())
            # print('im here')
            # return None
        
        while True:
            try:
                data = self.socket.recv(2048)
                print(f'Length of message in bytes : {len(data)}')
            except IOError as e:
                if e.errno == errno.EWOULDBLOCK:
                    return None
                self.socket.close()
                self.socket = None
                return None
            except Exception:
                self.socket.close()
                self.socket = None
                return None
            if len(data) == 0:
                self.socket.close()
                self.socket = None
                return None
            # print(f'this is the data {data}')
            return data

    
def main(args):
    username = args[0]
    password = args[1]
    caster = args[2]
    port = args[3]
    mountpoint = args[4]
    lat = args[5]
    lon = args[6]
    height = args[7]

    ntripArgs = {}
    ntripArgs['lat']= lat
    ntripArgs['lon']= lon 
    ntripArgs['height']= height

    ntripArgs['user']= username + ":" + password
    ntripArgs['caster']= caster
    ntripArgs['port']= port 
    ntripArgs['mountpoint']= mountpoint

    if ntripArgs['mountpoint'][0:1] !="/":
        ntripArgs['mountpoint'] = "/"+ntripArgs['mountpoint']

    ntripArgs['V2']= False

    ntripArgs['verbose']= True
    ntripArgs['headerOutput']= False


    if ntripArgs['verbose']:
        print ("Server: " + ntripArgs['caster'])
        print ("Port: " + str(ntripArgs['port']))
        print ("User: " + ntripArgs['user'])
        print ("mountpoint: " +ntripArgs['mountpoint'])
        if ntripArgs['V2']:
            print ("NTRIP: V2")
        else:
            print ("NTRIP: V1")

    n = NtripClient(**ntripArgs)

if __name__ == '__main__':
    import sys
    main(sys.argv[1:])