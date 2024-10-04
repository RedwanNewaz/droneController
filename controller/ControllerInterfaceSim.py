from pymavlink import mavutil
from dronekit import connect, VehicleMode
from threading import Thread
import time
from PyQt5.QtCore import QTimer, QObject
from PyQt5.QtCore import QTimer, QObject, pyqtSignal, pyqtSlot
from enum import Enum
import logging
logging.basicConfig(
    format='%(asctime)s %(levelname)-8s %(message)s',
    level=logging.INFO,
    datefmt='%H:%M:%S')


class State(Enum):
    INITIALIZED = 0
    ARMED = 1
    TAKEOFF = 2
    HOVER = 3
    LAND = 4
    STALL = 5

class ControllerInterface(QObject):
    def __init__(self, mainWindow, platform):
        self.mainWindow = mainWindow
        self.vehicle = platform.vehicle
        self.default_cmd_vel = 0.4
        self.default_dt = 0.05
        super().__init__()

        self.mainWindow.btnWest.clicked.connect(self.west_click)
        self.mainWindow.btnEast.clicked.connect(self.east_click)
        self.mainWindow.btnNorth.clicked.connect(self.north_click)
        self.mainWindow.btnSouth.clicked.connect(self.south_click)
        self.mainWindow.btnRTL.clicked.connect(self.rtl_click)
        self.mainWindow.btnUp.clicked.connect(self.up_click)
        self.mainWindow.btnDown.clicked.connect(self.down_click)
        logging.info("[ControllerInterface]: controller configured")

    
    

############### MAV LINK communication ##########################################################################
    def publish_cmd_vel(self, velocity_x, velocity_y, velocity_z):
        velocity_x, velocity_y, velocity_z = -velocity_x, -velocity_y, velocity_z
        print(velocity_x, velocity_y, velocity_z)
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        self.vehicle.send_mavlink(msg)

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        # send command to vehicle on x Hz cycle
        elapsed_time = 0.0
        while elapsed_time < duration:
            self.publish_cmd_vel(velocity_x, velocity_y, velocity_z)
            time.sleep(self.default_dt)
            elapsed_time += self.default_dt
############### Joystick communication ##########################################################################
    def vehicle_validation(self, function):
        if self.vehicle.mode == "GUIDED":
            logging.info(f'button clicked {function.__name__}')
            function()
    @pyqtSlot()
    def west_click(self):
        if self.state != State.HOVER:
            logging.warning("[Invalid request]: moving west is not possible")
            return
        @self.vehicle_validation
        def west_wrapped():
            self.send_ned_velocity(0, self.default_cmd_vel, 0, 1)
            # self.send_ned_velocity(0, 0, 0, 1)
    @pyqtSlot()
    def east_click(self):
        if self.state != State.HOVER:
            logging.warning("[Invalid request]: moving east is not possible")
            return
        @self.vehicle_validation
        def east_wrapped():
            self.send_ned_velocity(0, -self.default_cmd_vel, 0, 1)
            # self.send_ned_velocity(0, 0, 0, 1)
    
    @pyqtSlot()
    def north_click(self):
        if self.state != State.HOVER:
            logging.warning("[Invalid request]: moving north is not possible")
            return
        @self.vehicle_validation
        def north_wrapped():
            self.send_ned_velocity(-self.default_cmd_vel, 0, 0, 1)
            # self.send_ned_velocity(0, 0, 0, 1)
    @pyqtSlot()
    def south_click(self):
        if self.state != State.HOVER:
            logging.warning("[Invalid request]: moving south is not possible")
            return
        @self.vehicle_validation
        def south_wrapped():
            self.send_ned_velocity(self.default_cmd_vel, 0, 0, 1)
            # self.send_ned_velocity(0, 0, 0, 1)

    @pyqtSlot()
    def rtl_click(self):
        if self.state == State.LAND or self.state == State.INITIALIZED:
            logging.warning("[Invalid request]: landing is not possible")
            return
        @self.vehicle_validation
        def rtl_wrapped():
            self.vehicle.mode = VehicleMode("LAND")
            self.state = State.LAND

    @pyqtSlot()
    def up_click(self):
        if self.state != State.HOVER:
            logging.warning("[Invalid request]: moving up is not possible")
            return
        @self.vehicle_validation
        def up_wrapped():
            alt = self.vehicle.location.global_relative_frame.alt
            # if alt < 3:
            self.send_ned_velocity(0, 0, 0.5 * self.default_cmd_vel, 1)
                # self.send_ned_velocity(0, 0, 0, 1)

    @pyqtSlot()
    def down_click(self):
        if self.state != State.HOVER:
            logging.warning("[Invalid request]: moving down is not possible")
            return
        @self.vehicle_validation
        def down_wrapped():
            alt = self.vehicle.location.global_relative_frame.alt
            # if alt > 0.5:
            self.send_ned_velocity(0, 0, -0.5 * self.default_cmd_vel, 1)
                # self.send_ned_velocity(0, 0, 0, 1)
        

    # def publish_cmd_vel(self, velocity_x, velocity_y, velocity_z):
    #     msg = self.platform.vehicle.message_factory.set_position_target_local_ned_encode(
    #         0,  # time_boot_ms (not used)
    #         0, 0,  # target system, target component
    #         mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
    #         0b0000111111000111,  # type_mask (only speeds enabled)
    #         0, 0, 0,  # x, y, z positions (not used)
    #         velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
    #         0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
    #         0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    #     self.platform.vehicle.send_mavlink(msg)

    # def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):
    #     # send command to vehicle on x Hz cycle
    #     elapsed_time = 0.0
    #     while elapsed_time < duration:
    #         self.publish_cmd_vel(velocity_x, velocity_y, velocity_z)
    #         time.sleep(self.default_dt)
    #         elapsed_time += self.default_dt


    # ############### Joystick communication ##########################################################################
    # def vehicle_validation(self, function):
    #     if self.platform.vehicle.mode == "GUIDED":
    #         logging.debug('button clicked ', function.__name__)
    #         function()

    # def west_click(self):
    #     logging.info("[ControllerInterface]: moving west")
    #     if self.platform.state.name != "HOVER":
    #         logging.warning("[Invalid request]: moving west is not possible")
    #         return
    #     @self.vehicle_validation
    #     def west_wrapped():
    #         self.send_ned_velocity(0, -self.default_cmd_vel, 0, 1)
    #         # self.send_ned_velocity(0, 0, 0, 1)

    # def east_click(self):
    #     logging.info("[ControllerInterface]: moving east")
    #     if self.platform.state.name != "HOVER":
    #         logging.warning("[Invalid request]: moving east is not possible")
    #         return
    #     @self.vehicle_validation
    #     def east_wrapped():
    #         self.send_ned_velocity(0, self.default_cmd_vel, 0, 1)
    #         # self.send_ned_velocity(0, 0, 0, 1)

    # def north_click(self):
    #     logging.info("[ControllerInterface]: moving north")
    #     if self.platform.state.name != "HOVER":
    #         logging.warning("[Invalid request]: moving north is not possible")
    #         return
    #     @self.vehicle_validation
    #     def north_wrapped():
    #         self.send_ned_velocity(self.default_cmd_vel, 0, 0, 1)
    #         # self.send_ned_velocity(0, 0, 0, 1)

    # def south_click(self):
    #     logging.info("[ControllerInterface]: moving south")
    #     if self.platform.state.name != "HOVER":
    #         logging.warning("[Invalid request]: moving south is not possible")
    #         return
    #     @self.vehicle_validation
    #     def south_wrapped():
    #         self.send_ned_velocity(-self.default_cmd_vel, 0, 0, 1)
    #         # self.send_ned_velocity(0, 0, 0, 1)

    # def rtl_click(self):
    #     logging.info("[ControllerInterface]: moving rtl")
    #     if self.platform.state.name == "LAND" or self.platform.state == self.platform.state.INITIALIZED:
    #         logging.warning("[Invalid request]: landing is not possible")
    #         return
    #     @self.vehicle_validation
    #     def rtl_wrapped():
    #         self.platform.vehicle.mode = VehicleMode("LAND")
    #         self.platform.state = self.platform.state.LAND


    # def up_click(self):
    #     if self.platform.state.name != "HOVER":
    #         logging.warning("[Invalid request]: moving up is not possible")
    #         return
    #     @self.vehicle_validation
    #     def up_wrapped():
    #         alt = self.platform.vehicle.location.global_relative_frame.alt
    #         if alt < 3:
    #             self.send_ned_velocity(0, 0, -0.5 * self.default_cmd_vel, 1)
    #             # self.send_ned_velocity(0, 0, 0, 1)

    # def down_click(self):
    #     if self.platform.state.name != "HOVER":
    #         logging.warning("[Invalid request]: moving down is not possible")
    #         return
    #     @self.vehicle_validation
    #     def down_wrapped():
    #         alt = self.platform.vehicle.location.global_relative_frame.alt
    #         if alt > 0.5:
    #             self.send_ned_velocity(0, 0, 0.5 * self.default_cmd_vel, 1)
    #             # self.send_ned_velocity(0, 0, 0, 1)


