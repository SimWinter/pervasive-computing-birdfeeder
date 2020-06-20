import threading
import time

from PyQt5 import uic, QtWidgets


class Button(QtWidgets.QMainWindow):

    def __init__(self):
        super(Button, self).__init__()
        uic.loadUi('ui/button.ui', self)
        self.button = self.findChild(QtWidgets.QPushButton, 'bButton')
        self.button.clicked.connect(self.b_pressed)
        self.show()
        self.name = ""
        self.isPressed = False

    def set_name(self, name):
        self.name = name
        self.setWindowTitle(name)

    def b_pressed(self):
        threading.Thread(target=self.button_thread).start()

    def set_window_location(self, x, y):
        self.move(x,y)

    def button_thread(self):
        self.isPressed = True
        time.sleep(0.01)
        self.isPressed = False

    def is_pressed(self):
        return self.isPressed

    def setPressed(self, duration):
        self.isPressed = True
        self.lDuration.setText(str(duration))