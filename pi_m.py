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


class RaspberryPiM(AbstractRaspberryPi):
    def __init__(self):
        self.activeSensors = {}
        self.initialSensors = {"gesture": GestureSensor, "display": Display}
        self.sensorCnt = 0
        AbstractRaspberryPi.__init__(self, "pim", 5555, 5560)

        for s in self.initialSensors:
            self.add_sensor(self.initialSensors[s], s)

        self.show()

        # communication
        self.sender = AbstractSender(self.s_port)
        self.senderW = AbstractSender(5561);
        self.senderC = AbstractSender(5562);
        self.receiver = AbstractReceiver(self.r_port)
        self.lock = threading.Lock()
        threading.Thread(target=self.receiver_thread).start()

        # sensor data
        self.weight = 0;
        self.image = 0;

        # start sampling
        threading.Thread(target=self.sampling_thread).start()

    def set_window_location(self, x, y):
        self.move(x,y)

    def receiver_thread(self):
        while True:
            incoming_request = self.receiver.socket.recv()

    def send_messageC(self, message):
        self.lock.acquire()
        self.senderC.socket.send_string(message)
        response = self.senderC.socket.recv()
        self.image = bytes(response); # read image data
        self.lock.release()

    def send_messageW(self, message):
        self.lock.acquire()
        self.senderW.socket.send_string(message)
        response = self.senderW.socket.recv()
        if "weight" in str(response):
            self.activeSensors["display"].on(str(response))
            self.weight = str(response)[18:-1]
        self.lock.release()

    def sampling_thread(self):
        time.sleep(5)
        while True:
            # poll piW
            #self.send_messageW("pW request weight")

            # poll piC
            #self.send_messageC("pC request camera")

            # analyze data

            # send instructions to piW

            # test stuff
            if self.activeSensors["gesture"].get_gesture() == "Up":
                self.send_messageW("pW request weight")
                self.send_messageC("pC request camera")
            elif self.activeSensors["gesture"].get_gesture() == "Left":
                self.send_messageW("pW servo up")
            elif self.activeSensors["gesture"].get_gesture() == "Right":
                self.send_messageW("pW servo down")
            elif self.activeSensors["gesture"].get_gesture() == "Down":
                self.send_messageW("pW servo off")