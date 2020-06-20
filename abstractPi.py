import threading
from PyQt5 import QtWidgets, uic


class AbstractRaspberryPi(QtWidgets.QMainWindow):
    def __init__(self, name, s_port, r_port):
        self.name = name
        self.sensorCnt = 0
        self.s_port = s_port
        self.r_port = r_port
        super(AbstractRaspberryPi, self).__init__()
        uic.loadUi('ui/pi.ui', self)

        self.setWindowTitle(name)
        self.sensorListWidget = self.findChild(QtWidgets.QListWidget, 'lSensors')

        self.show()

    def add_sensor(self, type, name):
        if name == "" or name is None:
            name = type+str(self.sensorCnt)
        self.activeSensors[name] = type()
        self.activeSensors[name].set_name(self.name + " " + name)
        self.sensorListWidget.addItem(name)
        self.sensorCnt += 1
