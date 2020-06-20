import sys
from PyQt5 import QtWidgets, uic
from pi_w import RaspberryPiW
from pi_c import RaspberryPiC
from pi_m import RaspberryPiM


class Main(QtWidgets.QMainWindow):
    def __init__(self):
        self.sensorCnt = 0
        super(Main, self).__init__()
        uic.loadUi('ui/main.ui', self)

        self.piW = RaspberryPiW()
        self.piC = RaspberryPiC()
        self.piM = RaspberryPiM()

        self.setWindowTitle("Connected Raspberry Pis")
        self.piListWidget = self.findChild(QtWidgets.QListWidget, 'lPis')
        self.piListWidget.addItem("piW")
        self.piListWidget.addItem("piC")
        self.piListWidget.addItem("piM")
        self.show()

    def set_window_location(self, x, y):
        self.move(x,y)

app = QtWidgets.QApplication(sys.argv)
window = Main()
app.exec_()