from .StateMachine import State, Action
from collections import defaultdict
from threading import Thread
from dronekit import connect, VehicleMode
import dronekit_sitl

import logging
logging.basicConfig(
    format='%(asctime)s %(levelname)-8s %(message)s',
    level=logging.INFO,
    datefmt='%H:%M:%S')
class SimulatorInterface:
    def __init__(self):
        self.vehicle = None
        self.state = State.LAND
        self.transition = defaultdict(dict)
        self.transition[State.INITIALIZED][Action.IS_ARMABLE] = State.ARMED
        self.transition[State.ARMED][Action.IS_ARMED] = State.TAKEOFF
        self.transition[State.TAKEOFF][Action.HAS_ARRIVED] = State.HOVER
        self.transition[State.LAND][Action.DISARMED] = State.INITIALIZED
        self.thread = Thread(target=self.connect)
        self.thread.start()
        self.launchAlt = 1 # m

    def connect(self):
        self.sitl = dronekit_sitl.start_default()
        self.connection_string = self.sitl.connection_string()

        # Connect to the Vehicle
        logging.info('Connecting to vehicle on: %s' % self.connection_string)
        self.vehicle = connect(self.connection_string, wait_ready=True)
        self.state = State.INITIALIZED
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
        logging.info("[State]: {} | Action = {}".format(self.state.name, self.vehicle.system_status.state))

        if self.state == State.ARMED:
            self.vehicle.mode = VehicleMode("GUIDED")
            self.vehicle.armed = True


        elif self.state == State.TAKEOFF:
            self.vehicle.simple_takeoff(self.launchAlt)

        elif self.state == State.HOVER:
            logging.info("vehicle reached to hovering position")


        elif self.state == State.INITIALIZED:
            logging.info("vehicle landed")
            self.timer.stop()

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
        logging.debug(f'location: {x}, {y}, {z}')
