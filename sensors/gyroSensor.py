from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QSlider, QLabel


class GyroSensor(QtWidgets.QMainWindow):
    def __init__(self):
        super(GyroSensor, self).__init__()
        uic.loadUi('ui/gyro.ui', self)

        self.roll = 90
        self.pitch = 0
        self.yaw = 0
        self.sliderRoll = self.findChild(QSlider, 'sRoll')
        self.sliderRoll.valueChanged.connect(self.rot_roll)
        self.sliderPitch = self.findChild(QSlider, 'sPitch')
        self.sliderPitch.valueChanged.connect(self.rot_pitch)
        self.sliderYaw = self.findChild(QSlider, 'sYaw')
        self.sliderYaw.valueChanged.connect(self.rot_yaw)

        self.show()
        self.name = ""

    def set_window_location(self, x, y):
        self.move(x,y)

    def set_name(self, name):
        self.name = name
        self.setWindowTitle(name)

    def rot_pitch(self):
        self.pitch = self.sliderPitch.value()
        self.findChild(QLabel, 'lPitch').setText(str(self.pitch))

    def rot_roll(self):
        self.roll  = self.sliderRoll.value()
        self.findChild(QLabel, 'lRoll').setText(str(self.roll))

    def rot_yaw(self):
        self.yaw = self.sliderYaw.value()
        self.findChild(QLabel, 'lYaw').setText(str(self.yaw))

    def get_orientation(self):
        return (self.roll, self.pitch, self.yaw)

    def set_orientation(self, roll, pitch, yaw):
        self.rot_roll(roll)
        self.rot_pitch(pitch)
        self.rot_yaw(yaw)