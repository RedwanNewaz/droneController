import time 
import numpy as np
from PyQt5.QtCore import QTimer, QObject, pyqtSignal, pyqtSlot
from pyvicon_datastream import tools


import logging
logging.basicConfig(
    format='%(asctime)s %(levelname)-8s %(message)s',
    level=logging.INFO,
    datefmt='[Vicon %H:%M:%S]')


class ViconClient(QObject):
    pose_signal = pyqtSignal(dict)
    
    def __init__(self, obj_name:str, ip_address:str, parent: QObject = None) -> None:
        self.ip_address = ip_address
        self.obj_name = obj_name
        super().__init__(parent) 
        self.tracker = tools.ObjectTracker(self.ip_address)
        self.connection_ressolve()
   
    def pose_callback(self):
        if not self.tracker.is_connected:
            logging.info('vicon is not connected ...')
            self.connection_ressolve()
            return

        latency, frameno, position = self.tracker.get_position(self.obj_name)
        if position != []:
            xyz_position = position[0][2:5] # get x,y,z only
            orientation = position[0][7] # get rotation around z axis
            xyz_position[0], xyz_position[1] = xyz_position[1], xyz_position[0]
            data = {"position": np.array(xyz_position) / 1000.0, "orientation" : orientation}
            logging.debug(f'{self.obj_name} position = {data}')
            self.pose_signal.emit(data)
    
    def isConnected(self):
        return self.tracker.is_connected
            

    def connection_ressolve(self):
        while not self.tracker.is_connected:
            logging.warning('waiting for vicon to connect ...[!]')
            self.tracker.connect(self.ip_address)
            time.sleep(1)
        logging.info('vicon connection established ...')


if __name__ == "__main__":
    import sys 
    from PyQt5.QtWidgets import QApplication
    app = QApplication(sys.argv)
    vicon = ViconClient(obj_name="djimini", ip_address="192.168.10.2")
    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            break 

    sys.exit(app.exec())
  

