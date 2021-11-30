import serial
from enum import Enum
import struct
import time
import socket
import json
from math import radians, cos, sin

QUAL = 0
DIST = 0

addrPort = ("127.0.0.1", 9870)



class RDState(Enum):
    WAIT_RDSTART = 0
    WAIT_RDINDEX= 1
    WAIT_PAYLOAD = 2


class XV11:
    def __init__(self, port):
        self.serial = serial.Serial(port, 115200)
        self.state = RDState.WAIT_RDSTART
        self.index = 0
        self.bytes_left = 2
        self.speed = 0

    @staticmethod
    def process_data(angle, data):
        x = data[0]
        x1 = data[1]
        x2 = data[2]
        x3 = data[3]
        dist_mm = x | ((x1 & 0x3f) << 8)  # distance is coded on 13 bits ? 14 bits ?
        quality = x2 | (x3 << 8)  # quality is on 16 bits
        if x1 & 0x80:
            quality = 0
            dist_mm = 0
        return angle, quality, dist_mm, 1 if angle == 0 else 0

    def start_scan(self):
        while True:
            time.sleep(0.00001)  # do not hog the processor power
            if self.state == RDState.WAIT_RDSTART:
                b = self.serial.read(1)
                # start byte
                if b == bytes([0xFA]):
                    self.state = RDState.WAIT_RDINDEX
                else:
                    self.state = RDState.WAIT_RDSTART
            elif self.state == RDState.WAIT_RDINDEX:
                # position index
                b = self.serial.read(1)
                if bytes([0xA0]) <= b <= bytes([0xF9]):
                    self.index = int.from_bytes(b, byteorder='big') - 0xA0
                    self.state = RDState.WAIT_PAYLOAD
                elif b != bytes([0xFA]):
                    self.state = RDState.WAIT_RDSTART
            elif self.state == RDState.WAIT_PAYLOAD:
                # speed
                b_speed = [b for b in self.serial.read(2)]

                # data
                b_data0 = [b for b in self.serial.read(4)]
                b_data1 = [b for b in self.serial.read(4)]
                b_data2 = [b for b in self.serial.read(4)]
                b_data3 = [b for b in self.serial.read(4)]

                # for the checksum, we need all the data of the packet...
                # this could be collected in a more elegant fashion...
                all_data = [0xFA, self.index + 0xA0] + b_speed + b_data0 + b_data1 + b_data2 + b_data3

                # checksum
                b_checksum = [b for b in self.serial.read(2)]
                incoming_checksum = int(b_checksum[0]) + (int(b_checksum[1]) << 8)

                # verify that the received checksum is equal to the one computed from the data
                if checksum(all_data) == incoming_checksum:
                    # speed_rpm = compute_speed(b_speed)
                    # gui_update_speed(speed_rpm)

                    # motor_control(speed_rpm)
                    self.speed = (b_speed[0] | b_speed[1] << 8)/64
                    yield self.process_data(self.index * 4 + 0, b_data0)
                    yield self.process_data(self.index * 4 + 1, b_data1)
                    yield self.process_data(self.index * 4 + 2, b_data2)
                    yield self.process_data(self.index * 4 + 3, b_data3)

                    #if index == packet_per_cyle:
                    #    cycle = (cycle + 1) % 2
                    #    if cycle == 0:
                    #        lidar_serial.flushInput()

                else:
                    pass
                    # the checksum does not match, something went wrong...
                    # nb_errors += 1
                    # label_errors.text = "errors: " + str(nb_errors)
                    #
                    # # display the samples in an error state
                    # update_view(index * 4 + 0, [0, 0x80, 0, 0])
                    # update_view(index * 4 + 1, [0, 0x80, 0, 0])
                    # update_view(index * 4 + 2, [0, 0x80, 0, 0])
                    # update_view(index * 4 + 3, [0, 0x80, 0, 0])

                self.state = RDState.WAIT_RDSTART  # reset and wait for the next packet




def checksum(data):
    """Compute and return the checksum as an int.

    data -- list of 20 bytes (as ints), in the order they arrived in.
    """
    # group the data by word, little-endian
    data_list = []
    for t in range(10):
        data_list.append(data[2 * t] + (data[2 * t + 1] << 8))

    # compute the checksum on 32 bits
    chk32 = 0
    for d in data_list:
        chk32 = (chk32 << 1) + d

    # return a value wrapped around on 15bits, and truncated to still fit into 15 bits
    checksum = (chk32 & 0x7FFF) + (chk32 >> 15)  # wrap around to fit into 15 bits
    checksum = checksum & 0x7FFF  # truncate to 15 bits
    return int(checksum)
    
    
if __name__ == "__main__":
    lidar = XV11("/dev/ttyUSB0")
    #s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for angle, quality, distance, s in lidar.start_scan():
        if distance != 0:
            print(f"angle: {angle}, quality: {quality}, distance:{distance}")

    
