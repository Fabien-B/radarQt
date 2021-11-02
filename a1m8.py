import serial
from enum import Enum
import struct
import time

HEALTH_STATUS = {0: "Good", 1: "Warning", 2: "Error"}

TIMEOUT = 0.1


class RDState(Enum):
    WAIT_RDSTART1 = 0
    WAIT_RDSTART2 = 1
    WAIT_LEN_MODE = 2
    WAIT_DATA_TYPE = 3


class ResponseCode(Enum):
    INFO = 0x04
    HEALTH = 0x06
    SAMPLE_RATE = 0x15
    CONF = 0x20


class Request(Enum):
    STOP = 0x25
    RESET = 0x40
    SCAN = 0x20
    EXPRESS_SCAN = 0x82
    FORCE_SCAN = 0x21
    GET_INFO = 0x50
    GET_HEALTH = 0x52
    GET_SAMPLERATE = 0x59
    GET_LIDAR_CONF = 0x84


class LidarConfType(Enum):
    RP_LIDAR_SCAN_MODE_COUNT = 0x70
    RP_LIDAR_SCAN_MODE_US_PER_SAMPLE = 0x71
    RP_LIDAR_SCAN_MODE_MAX_DISTANCE = 0x74
    RP_LIDAR_SCAN_MODE_ANS_TYPE = 0x75
    RP_LIDAR_SCAN_MODE_TYPICAL = 0x7C
    RP_LIDAR_SCAN_MODE_NAME = 0x7F


class A1M8:

    REQUEST_START_FLAG = 0XA5
    RDSTART_FLAG1 = 0xA5
    RDSTART_FLAG2 = 0x5A

    def __init__(self, port):
        self.serial = serial.Serial(port, 115200, dsrdtr=True, timeout=TIMEOUT)
        self.bytes_left = 2
        self.serial.dtr = False

    def stop(self):
        self.serial.close()

    def check(self, data):
        chk = 0
        for b in data:
            chk ^= b
        return chk

    def send(self, command: Request, payload=None):
        packet = struct.pack("<BB", A1M8.REQUEST_START_FLAG, command.value)
        if payload is not None:
            packet += struct.pack("<B", len(payload)) + payload
            chk = self.check(packet)
            packet += struct.pack("<B", chk)
        #print(packet.hex())
        self.serial.write(packet)

    def receive_response_descriptor(self):
        """

        :return: length, send_mode, data_type
        """
        state = RDState.WAIT_RDSTART1
        time_start = time.time()
        bytes_to_read = 1

        response_length, send_mode, data_type = -1, -1, -1

        while (time.time() - time_start) < TIMEOUT:
            buffer = self.serial.read(bytes_to_read)
            if len(buffer) != bytes_to_read:
                raise Exception(f"expected {bytes_to_read}, received {len(buffer)}")

            if state == RDState.WAIT_RDSTART1:
                if buffer[0] == A1M8.RDSTART_FLAG1:
                    state = RDState.WAIT_RDSTART2
                    bytes_to_read = 1
            elif state == RDState.WAIT_RDSTART2:
                if buffer[0] == A1M8.RDSTART_FLAG2:
                    state = RDState.WAIT_LEN_MODE
                    bytes_to_read = 4
                else:
                    state = RDState.WAIT_RDSTART2
                    bytes_to_read = 1
            elif state == RDState.WAIT_LEN_MODE:
                send_mode = buffer[3] >> 6
                buf = bytearray(buffer)
                buf[3] &= 0b00111111
                (response_length,) = struct.unpack("<I", buf)
                state = RDState.WAIT_DATA_TYPE
                bytes_to_read = 1
            elif state == RDState.WAIT_DATA_TYPE:
                data_type = buffer[0]
                return response_length, send_mode, data_type
        raise TimeoutError()

    def receive_single_response(self, lenght, data_type):
        def process_info(data):
            model, firmware_minor, firmware_major, hardware = struct.unpack("<BBBB", data[:4])
            serial_nb = data[4:].hex()
            return model, firmware_major, firmware_minor, hardware, serial_nb

        def process_sample_rate(data):
            print(data)
            sr_std, sr_express = struct.unpack("<HH", data)
            print(f"sample rate standard: {sr_std}, express:{sr_express}")

        def process_health(data):
            status, error_code = struct.unpack("<BH", data)
            return status, error_code

        def process_conf(data):
            conf_type, = struct.unpack("<I", data[:4])
            if conf_type == LidarConfType.RP_LIDAR_SCAN_MODE_COUNT.value:
                count, = struct.unpack("<H", data[4:])
                return count
            elif conf_type == LidarConfType.RP_LIDAR_SCAN_MODE_US_PER_SAMPLE.value:
                us_cost, = struct.unpack("<I", data[4:])
                return us_cost / (1 << 8)
            elif conf_type == LidarConfType.RP_LIDAR_SCAN_MODE_MAX_DISTANCE.value:
                max_dist, = struct.unpack("<I", data[4:])
                return max_dist / (1 << 8)
            elif conf_type == LidarConfType.RP_LIDAR_SCAN_MODE_ANS_TYPE.value:
                ans_type, = struct.unpack("<B", data[4:])
                return ans_type
            elif conf_type == LidarConfType.RP_LIDAR_SCAN_MODE_TYPICAL.value:
                typ_scan_mode, = struct.unpack("<H", data[4:])
                return typ_scan_mode
            elif conf_type == LidarConfType.RP_LIDAR_SCAN_MODE_NAME.value:
                name = data[4:-1].decode()
                return name

        buffer = self.serial.read(lenght)
        assert len(buffer) == lenght

        if data_type == ResponseCode.INFO.value:
            return process_info(buffer)
        elif data_type == ResponseCode.HEALTH.value:
            return process_health(buffer)
        elif data_type == ResponseCode.SAMPLE_RATE.value:
            return process_sample_rate(buffer)
        elif data_type == ResponseCode.CONF.value:
            return process_conf(buffer)
        else:
            raise Exception(f"Unknow data type {data_type}")

    def send_stop(self):
        lidar.send(Request.STOP)
        time.sleep(0.002)

    def send_reset(self):
        lidar.send(Request.RESET)
        time.sleep(0.002)

    def get_health(self):
        lidar.send(Request.GET_HEALTH)
        response_length, mode, data_type = lidar.receive_response_descriptor()
        status, error_code = self.receive_single_response(response_length, data_type)
        status_str = HEALTH_STATUS.get(status, "Unknown status!")
        print(f"status: {status_str}, error: {error_code}")

    def get_info(self):
        lidar.send(Request.GET_INFO)
        response_length, mode, data_type = lidar.receive_response_descriptor()
        ret = self.receive_single_response(response_length, data_type)
        model, firmware_major, firmware_minor, hardware, serial_nb = ret
        print(f"model:{model}, fw:{firmware_major}.{firmware_minor}, hw:{hardware} SN:{serial_nb}")

    def get_lidar_conf(self, query: LidarConfType, param=None):
        tx_payload = struct.pack("<I", query.value)
        if param is not None:
            tx_payload += struct.pack("<H", param)
        lidar.send(Request.GET_LIDAR_CONF, tx_payload)
        length, mode, data_type = lidar.receive_response_descriptor()
        conf = lidar.receive_single_response(length, data_type)
        return conf



if __name__ == "__main__":
    try:
        lidar = A1M8("/dev/ttyUSB0")
        #lidar.get_health()
        #lidar.get_info()
        #time.sleep(2)
        #lidar.get_health()

        nb_modes = lidar.get_lidar_conf(LidarConfType.RP_LIDAR_SCAN_MODE_COUNT)
        for i in range(nb_modes):
            name = lidar.get_lidar_conf(LidarConfType.RP_LIDAR_SCAN_MODE_NAME, i)
            us_per_sample = lidar.get_lidar_conf(LidarConfType.RP_LIDAR_SCAN_MODE_US_PER_SAMPLE, i)
            max_dist = lidar.get_lidar_conf(LidarConfType.RP_LIDAR_SCAN_MODE_MAX_DISTANCE, i)
            ans_type = lidar.get_lidar_conf(LidarConfType.RP_LIDAR_SCAN_MODE_ANS_TYPE, i)
            print(f"{name}: {us_per_sample}us, {max_dist}m, {hex(ans_type)}")

        time.sleep(0.1)
        #lidar.send(Request.SCAN)
        #while True:
        #    time.sleep(1)
    finally:
        lidar.stop()
