from PyQt5.QtCore import QTimer, QObject, pyqtSignal, QThread
from dronekit import connect, VehicleMode
from .StateMachine import State, Action
from .vicon import ViconClient
import time 
import logging

logging.basicConfig(
    format='%(asctime)s %(levelname)-8s %(message)s',
    level=logging.INFO,
    datefmt='%H:%M:%S')

class RobotInterface(QThread):
    def __init__(self, mainWindow, robot_name):
        self.vehicle = None
        self.state = State.LAND
        self.mainWindow = mainWindow
        self.robot_name = robot_name
        super().__init__()
    
    def run(self):

        self.state = State.INITIALIZED
        #TODO take args from mainWindow
        self._vicon = ViconClient(obj_name=self.robot_name, ip_address="192.168.10.2", parent=self)
        self._vicon.pose_signal.connect(self.updateLocationGUI)
        self.mainWindow.progressBar.setValue(25)
        while not self._vicon.isConnected():
            time.sleep(1)

        self.mainWindow.progressBar.setValue(50)

        self.connection_string = "0.0.0.0:18990"
        try:
            # Connect to the Vehicle
            self.vehicle = connect(self.connection_string, wait_ready=True)
            self.mainWindow.progressBar.setValue(100)
            logging.info('[RobotInterface]: Connected to vehicle on: %s' % self.connection_string)
            
            self.state = State.HOVER
        except:
            self.state = State.LAND

        # Display Flight Mode
        self.updateFlightModeGUI(self.vehicle.mode)
        self.addObserverAndInit(
            'mode'
            , lambda vehicle, name, mode: self.updateFlightModeGUI(mode))

        self.vehicle.mode = VehicleMode("GUIDED")

        while True:
            try:
                if self._vicon.isConnected():
                    self._vicon.pose_callback()
                else:
                    self._vicon.connection_ressolve()
            except:
                logging.error('[RobotInterface]: vicon connection failed vehicle on: %s' % self.connection_string)
                # break 
            finally:
                time.sleep(0.01)
    def updateLocationGUI(self, pose):
        x, y, z = pose['position']
        def is_inside_rectangle():
            rect = [-2.8, 2.8, -2.8, 2.8]
            xmin, xmax, ymin, ymax = rect
            return xmin <= x <= xmax and ymin <= y <= ymax
        
        if not is_inside_rectangle():
            self.state = State.STALL
            logging.info(f'[RobotInterface]: stall location: {x}, {y}, {z}')
        
        logging.debug(f'[RobotInterface]: location: {x}, {y}, {z}')
        self.mainWindow.updateCoord(x, y, z)

    def addObserverAndInit(self, name, cb):
        """We go ahead and call our observer once at startup to get an initial value"""
        self.vehicle.add_attribute_listener(name, cb)

    def updateFlightModeGUI(self, value):
        logging.info(f'[SimulatorInterface]: flight mode change to {value}')
        index, mode = str(value).split(':')
        self.mainWindow.lblFlightModeValue.setText(mode)