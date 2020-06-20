from PyQt5 import uic, QtWidgets


class RFIDSensor(QtWidgets.QMainWindow):

    def __init__(self):
        super(RFIDSensor, self).__init__()
        uic.loadUi('ui/rfid.ui', self)
        self.knownTags = []
        self.status = "nothing present"
        self.bAddTag = self.findChild(QtWidgets.QPushButton, 'bAddTag')
        self.bTestTag = self.findChild(QtWidgets.QPushButton, 'bTestTag')
        self.lAddTag = self.findChild(QtWidgets.QLineEdit, 'lAddTag')
        self.lTestTag = self.findChild(QtWidgets.QLineEdit, 'lTestTag')
        self.lKnownTags = self.findChild(QtWidgets.QListWidget, 'lKnownTags')
        self.lStatus = self.findChild(QtWidgets.QLabel, 'lStatus')
        self.bAddTag.clicked.connect(self.add_pressed)
        self.bTestTag.clicked.connect(self.test_pressed)
        self.show()
        self.name = ""

    def set_window_location(self, x, y):
        self.move(x,y)

    def set_name(self, name):
        self.name = name
        self.setWindowTitle(name)

    def add_pressed(self):
        self.add_tag(self.lAddTag.text())

    def test_pressed(self):
        self.test_tag(self.lTestTag.text())

    def add_tag(self, tag):
        self.knownTags.append(str(tag))
        self.lKnownTags.addItem(str(tag))

    def test_tag(self, tag):
        if str(tag) == "":
            self.status = "nothing present"
        elif str(tag) in self.knownTags:
            self.status = "accepted"
        else:
            self.status = "rejected"
        self.lStatus.setText(self.status)

    def get_status(self):
        return self.status