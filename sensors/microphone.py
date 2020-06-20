import threading
from array import array

import pyaudio
from PyQt5 import uic, QtWidgets, QtGui
from PyQt5.QtCore import QPoint


class MicrophoneSensor(QtWidgets.QMainWindow):

    def __init__(self):
        super(MicrophoneSensor, self).__init__()
        uic.loadUi('ui/microphone.ui', self)

        self.show()
        self.name = ""

        self.form_1 = pyaudio.paInt16  # 16-bit resolution
        self.chans = 1  # 1 channel
        self.samp_rate = 44100  # 44.1kHz sampling rate
        self.chunk = 4096  # 2^12 samples for buffer
        self.dev_index = 1  # device index found by p.get_device_info_by_index(ii)
        self.audio = pyaudio.PyAudio()  # create pyaudio instantiation

        self.stream = self.audio.open(format=self.form_1, rate=self.samp_rate, channels=1, input=True,
                            output=True, frames_per_buffer=self.chunk)
        self.frames = []
        self.data = []
        self.audio_present = False
        threading.Thread(target=self.audio_thread).start()

    def set_window_location(self, x, y):
        self.move(x,y)

    def set_name(self, name):
        self.name = name

    def audio_thread(self):
        while True:
            d = (array('h', self.stream.read(self.chunk)))
            self.data.append(d)
            self.audio_present = not self.is_silent(d)
            self.repaint()

    def paintEvent(self, event):
        if self.audio_present:
            qp = QtGui.QPainter()
            qp.begin(self)
            qp.setPen(QtGui.QColor(0, 255, 0))
            qp.drawEllipse(QPoint(100, 80), 30, 30)
            qp.end()

    def is_silent(self, data):
        return max(data) < 500

    def get_audio_stream(self):
        return self.audioStream

    def get_audio_data(self):
        return self.data
