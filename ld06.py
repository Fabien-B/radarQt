import serial
from enum import Enum
import struct
import time


class RcvState(Enum):
    START = 0
    LEN = 1
    DATA = 2


class LD06:
    def __init__(self, port):
        self.serial = serial.Serial(port, 230400)
        self.state = RcvState.START
        self.nb_expected = 1
        self.len = 0
        self.speed = 0
        self.last_angle = 0

    def start_scan(self):
        while True:
            data = self.serial.read(self.nb_expected)
            if self.state == RcvState.START:
                if data[0] == 0x54:
                    self.state = RcvState.LEN
                    self.nb_expected = 1
            elif self.state == RcvState.LEN:
                self.len = data[0] & 0x0E
                self.state = RcvState.DATA
                self.nb_expected = 9 + 3 * self.len
            elif self.state == RcvState.DATA:
                self.speed, = struct.unpack("<H", data[0:2])
                start_angle, = struct.unpack("<H", data[2:4])
                start_angle /= 100
                mes_data_end = 4+self.len*3
                mes_data = data[4:mes_data_end]
                end_angle, = struct.unpack("<H", data[mes_data_end:mes_data_end+2])
                end_angle /= 100
                timestamp, = struct.unpack("<H", data[mes_data_end+2:mes_data_end+4])
                crc = struct.unpack("<B", data[mes_data_end+4:])
                self.state = RcvState.START
                self.nb_expected = 1
                for i in range(self.len):
                    dist, = struct.unpack("<H", mes_data[3*i:3*i+2])
                    quality, = struct.unpack("<B", mes_data[3*i+2:3*i+3])
                    if end_angle < start_angle:
                        end_angle += 360
                    step = (end_angle-start_angle)/(self.len-1)
                    angle = (start_angle+step*i)
                    if angle >= 360:
                        angle -= 360
                    s = 1 if (angle < self.last_angle) else 0
                    self.last_angle = angle
                    yield angle, quality, dist, s
                self.last_end_angle = end_angle

