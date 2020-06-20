from PyQt5 import uic, QtWidgets, QtGui
from PyQt5.QtWidgets import QPushButton, QLabel


class InfraredSensor(QtWidgets.QMainWindow):

    def __init__(self):
        super(InfraredSensor, self).__init__()
        uic.loadUi('ui/ir.ui', self)
        self.button = self.findChild(QPushButton, 'bInterrupt')
        self.button.clicked.connect(self.b_pressed)
        self.lStatus = self.findChild(QLabel, 'lStatus')
        self.show()
        self.interrupted = False
        self.name = ""

    def set_window_location(self, x, y):
        self.move(x,y)

    def b_pressed(self):
        self.interrupt() if not self.interrupted else self.close()

    def set_name(self, name):
        self.name = name
        self.setWindowTitle(name)

    def paintEvent(self, event):
        qp = QtGui.QPainter()
        qp.begin(self)
        if not self.interrupted:
            qp.drawLine(50, 80, 140, 80)
        else:
            qp.drawLine(50, 80, 90, 80)
            qp.drawLine(110, 80, 140, 80)
        qp.end()

    def get_state(self):
        return self.interrupted

    def interrupt(self):
        self.interrupted = True
        self.lStatus.setText("Status: Interrupted")
        self.repaint()

    def close(self):
        self.interrupted = False
        self.lStatus.setText("Status: Uninterrupted")
        self.repaint()