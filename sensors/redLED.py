from PyQt5 import uic, QtWidgets, QtGui
from PyQt5.QtCore import QPoint
from PyQt5.QtWidgets import QLabel


class RedLED(QtWidgets.QMainWindow):

    def __init__(self):
        super(RedLED, self).__init__()
        self.isOn = False
        uic.loadUi('ui/led.ui', self)
        self.show()
        self.lStatus = self.findChild(QLabel, 'lStatus')
        self.name = ""

    def set_window_location(self, x, y):
        self.move(x,y)

    def set_name(self, name):
        self.name = name
        self.setWindowTitle(name)

    def paintEvent(self, event):
        if self.isOn:
            qp = QtGui.QPainter()
            qp.begin(self)
            qp.setPen(QtGui.QColor(255, 0, 0))
            qp.drawEllipse(QPoint(100,80), 30, 30)
            qp.end()

    def on(self):
        self.isOn = True
        self.lStatus.setText("Status: on")
        self.repaint()

    def off(self):
        self.isOn = False
        self.lStatus.setText("Status: off")
        self.repaint()


