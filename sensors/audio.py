from PyQt5 import uic, QtWidgets
from playsound import playsound



class Audio(QtWidgets.QMainWindow):

    def __init__(self):
        super(Audio, self).__init__()
        uic.loadUi('ui/audio.ui', self)

        self.show()
        self.name = ""

        self.audioFile = "beep.wav"

    def set_window_location(self, x, y):
        self.move(x,y)

    def set_name(self, name):
        self.name = name

    def play_audio(self):
        return playsound(self.audioFile)
