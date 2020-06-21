import threading
import time
from dataclasses import Field

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

#This raspberryPi is responsible for measuring the weight of the birds which come to the feeder. Its purpose is
#to check whether the birds are small. If they are, the PI will make the servo motor open the door. If not, a
#request will be sent to PI2 to get the number of birds. If the number is one and the weight is too large, the doors
#will close. However, if the weight is beyond a given threshold, but the number of birds is greater than four,
#the door will remain opened.
class RaspberryPi1(AbstractRaspberryPi):
    birds = 0   #field for the number of birds received from the other PI (PI2)

    def __init__(self):
        self.activeSensors = {}
        self.initialSensors = {"weight": WeightSensor, "motor": Servo}
        self.sensorCnt = 0
        self.birds = 0
        AbstractRaspberryPi.__init__(self, "pi3", 5555, 5556)

        for s in self.initialSensors:
            self.add_sensor(self.initialSensors[s], s)

        self.set_window_location(50, 50)
        self.show()

        # communication
        self.sender = AbstractSender(self.s_port)
        self.receiver = AbstractReceiver(self.r_port)
        self.lock = threading.Lock()
        threading.Thread(target=self.receiver_thread).start()

        # start sampling
        threading.Thread(target=self.sampling_thread).start()

    def set_window_location(self, x, y):
        self.move(x, y)

    def receiver_thread(self):
        while True:
            incoming_request = self.receiver.socket.recv()

#The main logic for the communication between two PIs is located in this method. Here we first send the message
#asking for data, and we also receive the response in this method. The amount of birds will be extracted from
#the response (string).
    def send_message(self, message):
        self.lock.acquire()
        self.sender.socket.send_string(message)

        response = str(self.sender.socket.recv())
        if "p2" in response:
            time_stamp = time.asctime()
            for word in response.split(' ', 5):
                if word.isdigit():
                    self.birds = (int(word))
                    print("P3: Response from P2 (" + str(self.birds) + " birds) received at" + str(time_stamp))
        self.lock.release()

#Here, we use the data we have received from the weight sensor and possibly the amount of birds from the other PI.
#If the weight measured is smaller or equal 10 the door will be opened by the servo. Otherwise, there are two
#possibilities: the amount of birds is smaller than 4 which means that there are rather large birds. In this case
#the door will be closed. However, if the weight is greater 10, and the amount of birds are greater equal 4, the
#door will remain open.
    def sampling_thread(self):
        #  time.sleep(5)
        while True:
            measured_weight = self.activeSensors["weight"].get_weight()
            if measured_weight >= 10:
                self.send_message("p3 request: no. of birds ")
                if self.birds < 4:
                    self.activeSensors["motor"].on("down")
                    self.activeSensors["motor"].off()
                    print("Closed! Weight: " + str(measured_weight) + ", No. of birds: " + str(self.birds))
                    time.sleep(2)
                else:
                    self.activeSensors["motor"].on("up")
                    self.activeSensors["motor"].off()
                    print("Open! Weight: " + str(measured_weight) + ",  No. of birds: " + str(self.birds))
                    time.sleep(2)
            else:
                self.activeSensors["motor"].on("up")
                self.activeSensors["motor"].off()
                print("Open! Weight: " + str(measured_weight) + ",  No. of birds: " + str(self.birds))
                time.sleep(2)
