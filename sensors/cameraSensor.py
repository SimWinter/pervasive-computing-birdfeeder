import threading

from PyQt5 import uic, QtWidgets
import cv2
#from PyQt5.QtWidgets import QLabel, QPushButton, QRadioButton
import PyQt5


class CameraSensor(QtWidgets.QMainWindow):

    def __init__(self):
        
        super(CameraSensor, self).__init__()
        uic.loadUi('ui/camera.ui', self)
        self.rCamera = self.findChild(PyQt5.QtWidgets.QRadioButton, 'rCamera')
        self.rCamera.setChecked(True)
        self.rCamera.toggled.connect(self.camera_mode)
        self.rVideo = self.findChild(PyQt5.QtWidgets.QRadioButton, 'rVideo')
        self.bStart = self.findChild(PyQt5.QtWidgets.QPushButton, 'bStart')
        self.rVideo.toggled.connect(self.video_mode)
        self.mode = "Video"
        self.show()
        self.img = None
        self.liveView = True
        self.iCamImage = self.findChild(PyQt5.QtWidgets.QLabel, 'iCam')
        self.viewButton = self.findChild(PyQt5.QtWidgets.QPushButton, 'bLiveView')
        self.viewButton.clicked.connect(self.toggle_view)
        self.name = ""
        self.cam = None
        self.bStart.clicked.connect(self.start)
        self.running = False

        # set your video path here
        self.videoPath = "videoplayback.mp4"

        # if you don't get a camera image change this to another index
        self.camIndex = 0

    def set_window_location(self, x, y):
        self.move(x,y)

    def start(self):
        if not self.running:
            self.cam = cv2.VideoCapture(self.videoPath) if self.mode == "Video" else cv2.VideoCapture(self.camIndex)
            self.rCamera.setEnabled(False)
            self.rVideo.setEnabled(False)
            self.running=True
            threading.Thread(target=self.video_thread).start()
            self.bStart.setText("Stop")
        else:
            self.running = False
            self.rCamera.setEnabled(True)
            self.rVideo.setEnabled(True)
            self.bStart.setText("Start")

    def camera_mode(self):
        self.mode = "Camera"
        self.rVideo.setChecked(False)

    def video_mode(self):
        self.mode = "Video"
        self.rCamera.setChecked(False)

    def set_name(self, name):
        self.name = name
        self.setWindowTitle(name)

    def toggle_view(self):
        self.liveView = not self.liveView

    def get_image(self):
        return self.img

    def video_thread(self):
        while self.running:
            ret, frame = self.cam.read()
            self.img = frame
            if self.liveView and ret:
                cv2.imshow("Live Image "+self.name, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.cam.release()
        cv2.destroyAllWindows()
        self.cam = None
