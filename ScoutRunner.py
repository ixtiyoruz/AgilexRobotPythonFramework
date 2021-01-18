import threading
from scout import ScoutBase
import time
import socket

class ScoutRunner:
    def __init__(self, keep_running=False):
        self.scout_base =  ScoutBase()
        self.init_udp_server()
        self.th1 = threading.Thread(target=self.process, daemon=True)
        self.th2 = threading.Thread(target=self.process_udp, daemon=True)
        self.th1.start()
        self.th2.start()
        if(keep_running):
            self.th1.join()
            self.th2.join()
    def process(self):
        try:
            while(True):
                self.scout_base.receive_msg()
        except KeyboardInterrupt:
           pass  # exit normally
    def init_udp_server(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        # Enable port reusage so we will be able to run multiple clients and servers on single (host, port).
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        # Enable broadcasting mode
        #https://github.com/ninedraft/python-udp/blob/master/server.py
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # Set a timeout so the socket does not block
        # indefinitely when trying to receive data.
        self.server.settimeout(0.2)


    def process_udp(self):
        try:
            while(True):
                message = self.scout_base.convert_all_to_json()
                
                self.server.sendto(str.encode(message), ("localhost", 37020))
                # print("message sent :", message)
        except KeyboardInterrupt:
            pass # exit normally

    def wait_until_control_received(self,):
         while(True):
            if(not self.scout_base.is_can_control_on): 
                time.sleep(1)
                continue
            break
    