from PyQt5 import uic, QtWidgets


class HumiditySensor(QtWidgets.QMainWindow):

    def __init__(self):
        super(HumiditySensor, self).__init__()
        uic.loadUi('ui/humidity.ui', self)
        self.humidity = 22
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
        self.set_humidity(float(self.tTemp.text()))

    def get_humidity(self):
        return self.humidity

    def set_humidity(self, humidity):
        self.humidity = humidity
        self.tTemp.setText(str(humidity))