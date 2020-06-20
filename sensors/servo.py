from PyQt5 import uic, QtWidgets
from PyQt5.QtCore import QPoint
from PyQt5.QtGui import QPainter, QPolygon

from PyQt5.QtWidgets import QLabel


class Servo(QtWidgets.QMainWindow):

    def __init__(self):
        super(Servo, self).__init__()
        uic.loadUi('ui/servo.ui', self)
        self.direction = "off"
        self.lStatus = self.findChild(QLabel, 'lStatus')
        self.show()
        self.name = ""

        self.pointsUp = QPolygon([
            QPoint(50, 10),
            QPoint(80, 100),
            QPoint(20, 100),

        ])

        self.pointsDown = QPolygon([
            QPoint(50, 100),
            QPoint(80, 10),
            QPoint(20, 10),

        ])

    def set_window_location(self, x, y):
        self.move(x,y)

    def set_name(self, name):
        self.name = name
        self.setWindowTitle(name)

    def paintEvent(self, event):
        painter = QPainter(self)
        if self.direction is not "off":
            painter.drawPolygon(self.pointsDown) if self.direction == "down" else painter.drawPolygon(self.pointsUp)

    def on(self, direction):
        self.direction = direction
        self.lStatus.setText("Status: on")
        self.repaint()

    def off(self):
        self.direction = "off"
        self.lStatus.setText("Status: off")
        self.repaint()


