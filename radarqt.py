#!/usr/bin/python3
from PyQt5 import QtCore, QtWidgets, QtGui
import time
import math
import sys
import a1m8
import xv11
import ld06
import sys


class LidarHandler(QtCore.QThread):
    sample_available = QtCore.pyqtSignal((float, float, float, int))
    speed = QtCore.pyqtSignal(float)

    def __init__(self, parent=None):
        QtCore.QThread.__init__(self, parent)
        #self.lidar = a1m8.A1M8("/dev/ttyUSB0")
        #self.lidar = xv11.XV11("/dev/ttyUSB0")
        self.lidar = ld06.LD06(sys.argv[1])

    def run(self):
        #msgs = self.lidar.send_reset()
        #print(msgs)
        for angle, quality, distance, s in self.lidar.start_scan():
            self.sample_available.emit(math.radians(angle), distance, quality, s)
            self.speed.emit(self.lidar.speed)


class RadarView(QtWidgets.QWidget):
    COLOR_SCALE = ["#00876c", "#3d9c73", "#63b179",
                   "#88c580", "#aed987", "#d6ec91",
                   "#ffff9d", "#fee17e", "#fcc267",
                   "#f7a258", "#ef8250", "#e4604e",
                   "#d43d51"]

    def __init__(self, *args, **kwargs):
        QtWidgets.QWidget.__init__(self, *args, **kwargs)
        self.data = []
        self.back = []
        self.last_tour_time = time.time()
        self.frequency = 0
        self.last_angle = 0
        self.mm_to_pixel = 0.1
        self.setSizePolicy(
            QtWidgets.QSizePolicy.MinimumExpanding,
            QtWidgets.QSizePolicy.MinimumExpanding
        )

    def set_speed(self, speed):
        self.frequency = speed/60

    def color_from_quality(self, quality):
        #color_index = quality - 15 + len(self.COLOR_SCALE) / 2
        color_index = len(self.COLOR_SCALE) - int(quality * len(self.COLOR_SCALE) / 255) - 1
        c = QtGui.QColor(self.COLOR_SCALE[color_index])
        return c

    def wheelEvent(self, e: QtGui.QWheelEvent) -> None:
        d = e.angleDelta().y()
        self.mm_to_pixel *= (1 + d / 1000)
        self.update()

    def add_data(self, angle, distance, quality, s):
        if s != 0:
            t = time.time()
            dt = t - self.last_tour_time
            self.last_tour_time = t
            #self.frequency = 0.6*self.frequency + 0.4*1/dt
            self.data = self.back
            self.back = []
            self.update()
        self.back.append((angle, distance, quality))

    def mousePressEvent(self, a0: QtGui.QMouseEvent) -> None:
        self.data = []

    def paintEvent(self, e: QtGui.QPaintEvent) -> None:
        painter = QtGui.QPainter(self)

        # paint background
        brush = QtGui.QBrush()
        brush.setColor(QtGui.QColor('black'))
        brush.setStyle(QtCore.Qt.SolidPattern)
        rect = QtCore.QRect(0, 0, painter.device().width(), painter.device().height())
        painter.fillRect(rect, brush)

        # paint frequency
        painter.setPen(QtCore.Qt.white)
        txt = "{:.2f} Hz".format(self.frequency)
        painter.drawText(10, 20, txt)

        # paint scale
        for i, color in enumerate(self.COLOR_SCALE):
            scale_rect = QtCore.QRect(rect.right()-50, 10 + i*20, 40, 20)
            painter.setBrush(QtGui.QColor(color))
            painter.setPen(QtCore.Qt.NoPen)
            painter.drawRect(scale_rect)

        # translate and rotate painter
        painter.translate(painter.device().width()/2, painter.device().height()/2)
        painter.rotate(-90)

        # draw 0° line
        painter.setPen(QtCore.Qt.gray)
        painter.drawLine(QtCore.QPoint(0, 0), QtCore.QPoint(1000, 0))

        # paint circles
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.setPen(QtCore.Qt.gray)
        painter.drawEllipse(QtCore.QPoint(0, 0), self.mm_to_pixel * 500, self.mm_to_pixel * 500)
        painter.drawEllipse(QtCore.QPoint(0, 0), self.mm_to_pixel * 1500, self.mm_to_pixel * 1500)
        painter.drawEllipse(QtCore.QPoint(0, 0), self.mm_to_pixel * 2500, self.mm_to_pixel * 2500)
        painter.drawEllipse(QtCore.QPoint(0, 0), self.mm_to_pixel * 3500, self.mm_to_pixel * 3500)

        painter.setPen(QtCore.Qt.white)
        painter.drawEllipse(QtCore.QPoint(0, 0), self.mm_to_pixel * 1000, self.mm_to_pixel * 1000)
        painter.drawEllipse(QtCore.QPoint(0, 0), self.mm_to_pixel * 2000, self.mm_to_pixel * 2000)
        painter.drawEllipse(QtCore.QPoint(0, 0), self.mm_to_pixel * 3000, self.mm_to_pixel * 3000)


        # draw center
        painter.setBrush(QtCore.Qt.green)
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawEllipse(QtCore.QPoint(0, 0), 5, 5)

        # paint points
        painter.setBrush(QtCore.Qt.yellow)
        painter.setPen(QtCore.Qt.NoPen)
        for angle, distance, quality in self.data:
            if quality != 0 and distance != 0:
                pos = QtCore.QPointF(self.mm_to_pixel * distance * math.cos(angle), self.mm_to_pixel * distance * math.sin(angle))
                size = 5
                c = self.color_from_quality(quality)
                painter.setBrush(c)
                painter.drawEllipse(pos, size, size)


    def sizeHint(self) -> QtCore.QSize:
        return QtCore.QSize(400, 400)


class ApplicationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self._main = QtWidgets.QWidget()
        self.setCentralWidget(self._main)
        layout = QtWidgets.QVBoxLayout(self._main)

        self.radarView = RadarView(self._main)
        layout.addWidget(self.radarView)

        self.lidar = LidarHandler()
        self.lidar.sample_available.connect(self.handle_data)
        self.lidar.speed.connect(self.handle_speed)
        self.lidar.start()

    def handle_data(self, theta, distance, quality, s):
        self.radarView.add_data(theta, distance, quality, s)

    def handle_speed(self, speed):
        self.radarView.set_speed(speed)


if __name__ == "__main__":
    qapp = QtWidgets.QApplication(sys.argv)
    app = ApplicationWindow()
    app.show()
    app.activateWindow()
    app.raise_()
    qapp.exec_()
