from PyQt5 import uic, QtWidgets


class WeightSensor(QtWidgets.QMainWindow):

    def __init__(self):
        super(WeightSensor, self).__init__()
        uic.loadUi('ui/weight.ui', self)
        self.weight = 0
        self.button = self.findChild(QtWidgets.QPushButton, 'bSetWeight')
        self.lWeight = self.findChild(QtWidgets.QLineEdit, 'lWeight')
        self.button.clicked.connect(self.b_pressed)
        self.show()
        self.name = ""

    def set_name(self, name):
        self.name = name
        self.setWindowTitle(name)

    def set_window_location(self, x, y):
        self.move(x,y)

    def b_pressed(self):
        self.set_weight(float(self.lWeight.text()))

    def get_weight(self):
        return self.weight

    def set_weight(self, weight):
        self.weight = weight
        self.lWeight.setText(str(weight))