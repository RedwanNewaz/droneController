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
    
    def __init__(obj_name:str, self, parent: QObject = None) -> None:
        self.ip_address = "0.0.0.0"
        self.tracker = tools.ObjectTracker(self.ip_address)
        self.obj_name = obj_name
        super().__init__(parent) 
        if self.tracker.is_connected:
            logging.info('vicon connected ...')
        else:
            logging.error('vicon cannot be connected')
        self.connection_ressolve()
        



    def pose_callback(self):
       
        if not self.tracker.is_connected:
            self.connection_ressolve()
            return

        latency, frameno, position = self.tracker.get_position(self.obj_name)
        if position != []:
            xyz_position = position[0][2:5] # get x,y,z only
            orientation = position[0][7] # get rotation around z axis
            data = {"position": np.array(xyz_position) / 1000.0, "orientation" : orientation}
            self.pose_signal.emit(data)
            print(data)

    def connection_ressolve(self):
        while not self.tracker.is_connected:
            logging.warning('waiting for vicon to connect ...[!]')
            self.tracker.connect(self.IP_ADDR)
            time.sleep(1)
        logging.info('vicon connection ressolved ...')
        self.vicon_timer = QTimer()
        self.vicon_timer.timeout.connect(self.pose_callback)
        self.vicon_timer.start(10) # 1ms -> 100 Hz  

if __name__ == "__main__":
    import sys 
    from PyQt5.QtWidgets import QApplication
    app = QApplication(sys.argv)
    vicon = ViconClient("djimini")
    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            break 

    sys.exit(app.exec())
  

