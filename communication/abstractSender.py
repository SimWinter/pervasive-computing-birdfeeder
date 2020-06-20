import zmq

class AbstractSender():
    def __init__(self, port):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect("tcp://127.0.0.1:"+str(port))

