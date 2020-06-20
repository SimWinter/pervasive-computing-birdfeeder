import threading
import time

from PyQt5 import uic, QtWidgets


class GestureSensor(QtWidgets.QMainWindow):

    def __init__(self):
        super(GestureSensor, self).__init__()
        uic.loadUi('ui/gesture.ui', self)
        self.bRight = self.findChild(QtWidgets.QPushButton, 'bRight')
        self.bUp = self.findChild(QtWidgets.QPushButton, 'bUp')
        self.bDown = self.findChild(QtWidgets.QPushButton, 'bDown')
        self.bLeft = self.findChild(QtWidgets.QPushButton, 'bLeft')
        self.bRight.clicked.connect(self.b_pressedR)
        self.bUp.clicked.connect(self.b_pressedU)
        self.bDown.clicked.connect(self.b_pressedD)
        self.bLeft.clicked.connect(self.b_pressedL)
        self.show()
        self.name = ""
        self.direction = ""

    def set_name(self, name):
        self.name = name
        self.setWindowTitle(name)

    def b_pressedR(self):
        self.perform_gesture("Right")

    def b_pressedU(self):
        self.perform_gesture("Up")

    def b_pressedD(self):
        self.perform_gesture("Down")

    def b_pressedL(self):
        self.perform_gesture("Left")

    def set_window_location(self, x, y):
        self.move(x,y)

    def gesture_thread(self):
        time.sleep(1)
        self.direction = ""

    def get_gesture(self):
        return self.direction

    def perform_gesture(self, direction):
        self.direction = direction
        threading.Thread(target=self.gesture_thread).start()