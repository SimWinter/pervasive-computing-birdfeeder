import threading
import time

from abstractPi import AbstractRaspberryPi
from communication.abstractReceiver import AbstractReceiver
from communication.abstractSender import AbstractSender
from sensors.accelSensor import AccelSensor
from sensors.audio import Audio
from sensors.brightnessSensor import BrightnessSensor
from sensors.button import Button
from sensors.cameraSensor import CameraSensor
from sensors.display import Display
from sensors.distanceSensor import DistanceSensor
from sensors.gesture import GestureSensor
from sensors.greenLED import GreenLED
from sensors.gyroSensor import GyroSensor
from sensors.humiditySensor import HumiditySensor
from sensors.infraredSensor import InfraredSensor
from sensors.microphone import MicrophoneSensor
from sensors.redLED import RedLED
from sensors.rfidSensor import RFIDSensor
from sensors.servo import Servo
from sensors.tempSensor import TempSensor
from sensors.weightSensor import WeightSensor


class RaspberryPiC(AbstractRaspberryPi):
    def __init__(self):
        self.activeSensors = {}
        self.initialSensors = {"camera": CameraSensor}
        self.sensorCnt = 0
        AbstractRaspberryPi.__init__(self, "piC", 5560, 5562)

        for s in self.initialSensors:
            self.add_sensor(self.initialSensors[s], s)

        self.show()

        # TODO: path to video / cam index
        self.activeSensors["camera"].videoPath = ""
        self.activeSensors["camera"].camIndex = 0

        # communication
        self.sender = AbstractSender(self.s_port)
        self.receiver = AbstractReceiver(self.r_port)
        self.lock = threading.Lock()
        threading.Thread(target=self.receiver_thread).start()

        # start sampling
        threading.Thread(target=self.sampling_thread).start()

    def set_window_location(self, x, y):
        self.move(x,y)

    def receiver_thread(self):
        while True:
            incoming_request = self.receiver.socket.recv()
            if "camera" in str(incoming_request):
            # TODO: send image data
                self.receiver.socket.send(self.activeSensors["camera"].get_image())
                #Test:
                #self.receiver.socket.send(b'\x00\x01\x00\x02\x00\x03')

    def send_message(self, message):
        self.lock.acquire()
        self.sender.socket.send_string(message)
        response = self.sender.socket.recv()
        self.lock.release()

    def sampling_thread(self):
        time.sleep(5)
        while True:
            None
            # Do some sampling
