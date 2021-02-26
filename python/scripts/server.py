from xmlrpc.server import SimpleXMLRPCServer
from socketserver import ThreadingMixIn
from xmlrpc.client import ServerProxy
import serial


class SmartBox():
    

    def __init__(self):
        arduino_port = "/dev/cu.usbserial-C15173ED83" #serial port of Arduino
        baud = 115200 #arduino uno runs at 9600 baud
        print("Starting connection to box...")
        ser = serial.Serial(arduino_port, baud)
        print("Connected to box!")

    def monitor_inputs(self):
        #display the data to the terminal
        getData=str(ser.readline())
        data=getData[0:][:-2]
        print(data)

class InterfaceServer(ThreadingMixIn, SimpleXMLRPCServer):
    pass


class EventServer:
    def __init__(self):
        self.rpc_server = InterfaceServer(("0.0.0.0", 8000), allow_none=True)

        self.subscriber = dict()

    def start_rpc_server(self):
        print("EventServer: Starting RPC server...")
        self.rpc_server.register_introspection_functions()
        self.rpc_server.register_function(self.subscribe_to_event, "subscribe_to_event")
        self.rpc_server.register_function(self.unsubscribe_from_event, "unsubscribe_from_event")
        self.rpc_server.serve_forever()

    def subscribe_to_event(self, event, sender_ip, sender_port):
        print("EventServer: Subscribing to event " + event)
        if event not in self.subscriber:
            self.subscriber[event] = []
        if (sender_ip, sender_port) not in self.subscriber[event]:
            self.subscriber[event].append((sender_ip, sender_port))

    def unsubscribe_from_event(self, event, sender_ip, sender_port):
        print("EventServer: Unsubscribing from event " + event)
        if event in self.subscriber:
            if (sender_ip, sender_port) in self.subscriber[event]:
                self.subscriber[event].remove((sender_ip, sender_port))

    def trigger_event(self, event, content):
        print("EventServer: Triggering event " + event)
        for sender in self.subscriber[event]:
            s = ServerProxy("http://" + sender[0] + ":" + sender[1], allow_none=True)
            s.set_event(event, content)

    def watchdog(self):
        # Implement here any hardware-related stuff -> use trigger_event to notify any subscribers
        if data:
            print("something happened")
        pass


if __name__ == "__main__":
    box = SmartBox()
    server = EventServer()
    server.start_rpc_server()
