import zmq


class AbstractReceiver():
    def __init__(self, port):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind("tcp://127.0.0.1:"+str(port))
