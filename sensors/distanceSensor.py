import threading
import time

from PyQt5 import uic, QtWidgets
from PyQt5.QtWidgets import QPushButton, QLineEdit, QLabel


class DistanceSensor(QtWidgets.QMainWindow):
    def __init__(self):
        super(DistanceSensor, self).__init__()
        uic.loadUi('ui/distance.ui', self)
        self.show()

        self.distance = 0.0

        self.lStepSize = self.findChild(QLineEdit, 'lSteps')
        self.lFrom = self.findChild(QLineEdit, 'lFrom')
        self.lTo = self.findChild(QLineEdit, 'lTo')
        self.lCurDist = self.findChild(QLabel, "lCurDist")
        self.findChild(QPushButton, 'bMove').clicked.connect(self.perform_movement)

        self.name = ""

    def set_name(self, name):
        self.name = name
        self.setWindowTitle(name)

    def perform_movement(self):
        if float(self.lStepSize.text()) == 0:
            self.distance = float(self.lFrom.text())
            self.lCurDist.setText("Current Distance: "+str(self.distance))
        threading.Thread(target=self.movement_thread).start()

    def set_window_location(self, x, y):
        self.move(x,y)

    def movement_thread(self):
        f = float(self.lFrom.text())
        t = float(self.lTo.text())
        s = float(self.lStepSize.text())
        if f > t:
            self.distance = f
            while self.distance >= float(self.lTo.text()):
                self.distance -= s
                self.lCurDist.setText("Current Distance: " + str(self.distance))
                time.sleep(0.5)
        else:
            while self.distance <= t:
                self.distance += s
                self.lCurDist.setText("Current Distance: " + str(self.distance))
                time.sleep(0.5)

    def get_distance(self):
        return self.distance

    def set_distance(self, start, to, step_size):
        self.lFrom.setText(str(start))
        self.lTo.setText(str(to))
        self.lStepSize.setText(str(step_size))
        self.perform_movement()