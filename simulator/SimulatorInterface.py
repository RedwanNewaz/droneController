from .StateMachine import State, Action
from collections import defaultdict
from threading import Thread
from dronekit import connect, VehicleMode
import dronekit_sitl
from PyQt5.QtCore import QTimer, QObject, pyqtSignal, QThread
import time 

import logging
logging.basicConfig(
    format='%(asctime)s %(levelname)-8s %(message)s',
    level=logging.INFO,
    datefmt='%H:%M:%S')
class SimulatorInterface(QThread):
    def __init__(self, mainWindow):
        self.vehicle = None
        self.state = State.LAND
        self.setTransitionGraph()
        self.launchAlt = 1 # m
        self.mainWindow = mainWindow
        super().__init__()
    
    def setTransitionGraph(self):
        self.transition = defaultdict(dict)
        self.transition[State.INITIALIZED][Action.IS_ARMABLE] = State.ARMED
        self.transition[State.ARMED][Action.IS_ARMED] = State.TAKEOFF
        self.transition[State.TAKEOFF][Action.HAS_ARRIVED] = State.HOVER
        self.transition[State.LAND][Action.DISARMED] = State.INITIALIZED

    def run(self):
        self.sitl = dronekit_sitl.start_default()
        self.connection_string = self.sitl.connection_string()

        # Connect to the Vehicle
        logging.info('[SimulatorInterface]: Connecting to vehicle on: %s' % self.connection_string)
        self.vehicle = connect(self.connection_string, wait_ready=True)
        self.state = State.INITIALIZED
        self.mainWindow.progressBar.setValue(25)

        while True:
            self.timerCallback()
            if self.state != State.HOVER:
                time.sleep(1)
            else: break

        

        # Display Flight Mode
        self.updateFlightModeGUI(self.vehicle.mode)
        self.addObserverAndInit(
            'mode'
            , lambda vehicle, name, mode: self.updateFlightModeGUI(mode))

        # Display Location Info
        self.updateLocationGUI(self.vehicle.location)
        self.addObserverAndInit(
            'location'
            , lambda vehicle, name, location: self.updateLocationGUI(location))

    def getAction(self, state: State):
        """get an action based on the state of the vehicle"""
        if state == State.INITIALIZED and self.vehicle.is_armable:
            return Action.IS_ARMABLE
        elif state == State.ARMED and self.vehicle.armed and self.vehicle.mode.name == 'GUIDED':
            return Action.IS_ARMED
        elif state == State.TAKEOFF and self.vehicle.location.global_relative_frame.alt >= self.launchAlt * 0.95:
            return Action.HAS_ARRIVED
        elif state == State.LAND and not self.vehicle.armed:
            return Action.DISARMED

    def timerCallback(self):
        """
        complete the launch process, update the state machine
        """
        action = self.getAction(self.state)
        if action is None:
            return

        self.state = self.transition[self.state][action]
        logging.info("[SimulatorInterface]: State = {} | Action = {}".format(self.state.name, self.vehicle.system_status.state))

        if self.state == State.ARMED:
            self.vehicle.mode = VehicleMode("GUIDED")
            self.vehicle.armed = True
            self.mainWindow.progressBar.setValue(50)


        elif self.state == State.TAKEOFF:
            self.vehicle.simple_takeoff(self.launchAlt)
            self.mainWindow.progressBar.setValue(75)

        elif self.state == State.HOVER:
            logging.info("[SimulatorInterface]: vehicle reached to hovering position")
            self.mainWindow.progressBar.setValue(100)

        elif self.state == State.INITIALIZED:
            logging.info("[SimulatorInterface]: vehicle landed")
         

    def addObserverAndInit(self, name, cb):
        """We go ahead and call our observer once at startup to get an initial value"""
        self.vehicle.add_attribute_listener(name, cb)

    def updateLocationGUI(self, location):
        # self.lblLongValue.setText(str(location.global_frame.lon))
        # self.lblLatValue.setText(str(location.global_frame.lat))
        # location.local_frame.
        x = location.local_frame.east
        y = location.local_frame.north
        z = location.local_frame.down

        if x is None or y is None or z is None:
            x,  y, z = 0, 0, 0
        z = max(-z, 0.0)
        logging.debug(f'[SimulatorInterface]: location: {x}, {y}, {z}')
        self.mainWindow.updateCoord(x, y, z)

    def updateFlightModeGUI(self, value):
        logging.info(f'[SimulatorInterface]: flight mode change to {value}')
        index, mode = str(value).split(':')
        self.mainWindow.lblFlightModeValue.setText(mode)
