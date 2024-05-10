#!/usr/bin/python3
import serial
from enum import Enum
import struct
import time
import sys
from generated import lidar_data_pb2  as pbl
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
from ecal.core.subscriber import ProtoSubscriber
import queue

class Ecal:
    def __init__(self):
        ecal_core.initialize(sys.argv, "RadarQt receiver")
        self.lidar_sub = ProtoSubscriber("lidar_data", pbl.Lidar)
        self.queue = queue.Queue()
        self.lidar_sub.set_callback(self.handle_lidar_data)
        self.last_angle = 0
        self.speed = 0

    def handle_lidar_data(self, topic_name, msg, time):
        #for angle, distance in reversed(list(zip(msg.angles, msg.distances))):
        for angle, distance, quality in list(zip(msg.angles, msg.distances, msg.quality)):
            ...
            self.queue.put((360-angle, distance, quality))

    def start_scan(self):
        while True:
            angle, dist, quality = self.queue.get()
            s = 1 if (angle < self.last_angle) else 0
            self.last_angle = angle
            yield angle, quality, dist, s

if __name__ == "__main__":
    lidar = Ecal()
    gen = lidar.start_scan()
    while True:
        ret = next(gen)
        print(ret)
    # for angle, quality, distance, s in lidar.start_scan():
    #     print(f"{angle:.1f}: {distance}")
    #     if s:
    #         print("---------------")
