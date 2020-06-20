from PyQt5 import uic, QtWidgets


class BrightnessSensor(QtWidgets.QMainWindow):

    def __init__(self):
        super(BrightnessSensor, self).__init__()
        uic.loadUi('ui/brightness.ui', self)
        self.brightness = 22
        self.button = self.findChild(QtWidgets.QPushButton, 'bSetBrightness')
        self.tBrightness = self.findChild(QtWidgets.QLineEdit, 'lBrightness')
        self.button.clicked.connect(self.b_pressed)
        self.show()
        self.name = ""

    def set_window_location(self, x, y):
        self.move(x,y)

    def set_name(self, name):
        self.name = name
        self.setWindowTitle(name)

    def b_pressed(self):
        self.set_brightness(float(self.tBrightness.text()))

    def get_brightness(self):
        return self.brightness

    def set_brightness(self, brightness):
        self.brightness = brightness
