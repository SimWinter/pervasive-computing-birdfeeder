from PyQt5 import uic, QtWidgets


class TempSensor(QtWidgets.QMainWindow):

    def __init__(self):
        super(TempSensor, self).__init__()
        uic.loadUi('ui/temperature.ui', self)
        self.temp = 22
        self.button = self.findChild(QtWidgets.QPushButton, 'bSetTemp')
        self.tTemp = self.findChild(QtWidgets.QLineEdit, 'tTemp')
        self.button.clicked.connect(self.b_pressed)
        self.show()
        self.name = ""

    def set_window_location(self, x, y):
        self.move(x,y)

    def set_name(self, name):
        self.name = name
        self.setWindowTitle(name)

    def b_pressed(self):
        self.set_temp(float(self.tTemp.text()))

    def get_temp(self):
        return self.temp

    def set_temp(self, temp):
        self.temp = temp
        self.tTemp.setText(str(temp))