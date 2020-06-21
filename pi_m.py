import threading
import time
import cv2
import os
import base64

import numpy as np
from abstractPi import AbstractRaspberryPi
from communication.abstractReceiver import AbstractReceiver
from communication.abstractSender import AbstractSender

""" This class will use openCVs deep neural network running YOLO (You Only Look Once)
    object dedection trained on the coco dataset. """


class RaspberryPiM(AbstractRaspberryPi):

    def __init__(self):
        self.activeSensors = {}
        self.initialSensors = {}
        self.sensorCnt = 0
        self.currentBirds = 0
        self.min_confidence = 0.2
        self.DEBUGFLAG = False
        
        # geht the paths to the weights, cfg and name files
        self.dirname = os.path.dirname(__file__)
        self.weightPath = os.path.join(self.dirname, "yolo-coco/yolov3-tiny.weights")
        self.configPath = os.path.join(self.dirname, "yolo-coco/yolov3-tiny.cfg")
        self.labelsPath = os.path.join(self.dirname, "yolo-coco/coco.names")

        #load the COCO class labels our model was trained on
        self.LABELS = open(self.labelsPath).read().strip().split("\n")

        #load the YOLO object detector int openCV's deep neural network
        self.net = cv2.dnn.readNetFromDarknet(self.configPath, self.weightPath)

        # determine only the output layer names that we need from YOLO
        self.ln = self.net.getLayerNames()
        self.ln = [self.ln[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

        AbstractRaspberryPi.__init__(self, "pi2", 5555, 5560)

        for s in self.initialSensors:
            self.add_sensor(self.initialSensors[s], s)
        self.show()

        # communication
        self.sender = AbstractSender(self.s_port)
        self.senderW = AbstractSender(5561)
        self.senderC = AbstractSender(5562)
        self.receiver = AbstractReceiver(self.r_port)
        self.lock = threading.Lock()
        threading.Thread(target=self.receiver_thread).start()
        threading.Thread(target=self.firstSend_thread).start()

    def set_window_location(self, x, y):
        self.move(x,y)

    def receiver_thread(self):
        while True:
            incoming_request = self.receiver.socket.recv()
            print(str(incoming_request))

    def send_message_pic_c (self, message):
        while True:
            self.lock.acquire()
            self.senderC.socket.send_string(message)
            response = self.senderC.socket.recv()                  
            #self.countBirds(image(response))        
        
            self.lock.release()
            
    def firstSend_thread(self):
        time.sleep(5)
        if self.DEBUGFLAG:
            print("Start sending message to C")        
        self.send_message_pic_c("camera")
             
    def countBirds(self, image):

        # load input image and get spatial dimensions 
        (H,W) = image.shape[:2]
        # construct a blob and perform a forward pass through the network
        blob = cv2.dnn.blobFromImage(image,1 / 255.0, (416, 416), swapRB=True, crop=False)        
        self.net.setInput(blob)
        start = time.time()
        layerOutputs = self.net.forward(self.ln)
        end = time.time()
        if self.DEBUGFLAG:
            print(f"YOLO took {end - start} seconds")

        # lists of bounding boxes, confidences and class IDs 
        boxes = []
        confidences = []
        classIDs = []

        # loop over eacht layer output
        for output in layerOutputs:
            # loop over detections       
            for detection in output:  
                # get class ID and confidence              
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID] 
                # filter low out detections with low confidences             
                if confidence > self.min_confidence: 
                    # scale bounding boxes to image size (if needed)                                      
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")                   
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))
                    # update lists with new bounding boxes, confidences and class IDs			        
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)               

        # filter out weak, overlapping bounding boxes
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, self.min_confidence, 0.3)

        # if we have at least one valid detetion
        print(f"Number of Dedections: {len(idxs)}")
        birdCount = 0
        if len(idxs) > 0:
            # loop over indexes
            for i in idxs.flatten():
                # get class labels and confidences as string
                text = f"{self.LABELS[classIDs[i]]}: {confidences[i]}"
                # draw rectangles over detected objects
                if self.DEBUGFLAG:
                    (x, y) = (boxes[i][0], boxes[i][1])
                    (w, h) = (boxes[i][2], boxes[i][3])
                    cv2.rectangle(image, (x, y), (x + w, y + h), 15, 2)
                    cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 15, 2)
                    print(text)                                  
                # extract number of birds
                if "bird" in f"{self.LABELS[classIDs[i]]}":
                    birdCount += 1
                
            if self.DEBUGFLAG:
                print(f"Bird Count: {birdCount}")
        # if last birdcount differs from actual birdcount
        if birdCount != self.currentBirds:
            self.currentBirds = birdCount
            #TODO: Send correct Message for pi_w
            self.senderW.socket.send_string(f"Birds: {self.currentBirds}")
            response = self.senderW.socket.recv()
        birdCount = 0
        if self.DEBUGFLAG:
            cv2.namedWindow("output", cv2.WINDOW_NORMAL)
            cv2.imshow("output", image)
            cv2.waitKey(0)
