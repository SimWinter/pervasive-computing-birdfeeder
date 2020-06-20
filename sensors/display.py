from PyQt5 import uic, QtWidgets, QtGui
from PyQt5.QtWidgets import QLabel


class Display(QtWidgets.QMainWindow):

    def __init__(self):
        super(Display, self).__init__()
        self.isOn = False
        uic.loadUi('ui/display.ui', self)
        self.show()
        self.lText = self.findChild(QLabel, 'lText')
        self.name = ""

    def set_window_location(self, x, y):
        self.move(x,y)

    def set_name(self, name):
        self.name = name
        self.setWindowTitle(name)

    def on(self, text):
        self.isOn = True
        self.lText.setText(str(text))

    def off(self):
        self.isOn = False
        self.lText.setText("")


