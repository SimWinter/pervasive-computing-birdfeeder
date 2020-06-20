import threading
import time

from PyQt5 import uic, QtWidgets
from PyQt5.QtWidgets import QPushButton, QLineEdit, QCheckBox


class AccelSensor(QtWidgets.QMainWindow):
    def __init__(self):
        super(AccelSensor, self).__init__()
        uic.loadUi('ui/accel.ui', self)
        self.show()

        self.xAccel = 0.0
        self.yAccel = 0.0
        self.zAccel = 0.0

        self.cX = self.findChild(QCheckBox, 'cX')
        self.cY = self.findChild(QCheckBox, 'cY')
        self.cZ = self.findChild(QCheckBox, 'cZ')

        self.lSpeed = self.findChild(QLineEdit, 'lSpeed')
        self.lDuration = self.findChild(QLineEdit, 'lDuration')

        self.findChild(QPushButton, 'bMove').clicked.connect(self.perform_movement)

        self.name = ""

    def set_window_location(self, x, y):
        self.move(x,y)

    def set_name(self, name):
        self.name = name
        self.setWindowTitle(name)

    def perform_movement(self):
        threading.Thread(target=self.movement_thread).start()

    def movement_thread(self):
        self.xAccel = 0 if not self.cX.isChecked() else self.calc_accel()
        self.yAccel = 0 if not self.cY.isChecked() else self.calc_accel()
        self.zAccel = 0 if not self.cZ.isChecked() else self.calc_accel()
        time.sleep(float(self.lDuration.text())/1000)
        self.xAccel = 0
        self.yAccel = 0
        self.zAccel = 0

    def calc_accel(self):
        return (float(self.lSpeed.text()) / (float(self.lDuration.text()) / 1000))

    def get_acceleration(self):
        return (self.xAccel, self.yAccel, self.zAccel)

    def set_acceleration(self, x, y, z, speed, duration):
        self.cX.setChecked(x)
        self.cY.setChecked(y)
        self.cZ.setChecked(z)
        self.lSpeed.setText(str(speed))
        self.lDuration.setText(str(duration))
        self.perform_movement()