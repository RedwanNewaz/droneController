from dronekit import connect, VehicleMode
from pymavlink import mavutil

from PyQt5.QtWidgets import QApplication, QTableWidgetItem, QMainWindow
import sys
from PyQt5 import uic
from PyQt5.QtCore import QTimer
from threading import Thread
import PyQt5
from EKF import  ekf_estimation

import configparser
from enum import Enum
from collections import defaultdict
import time
from Quadrotor import Quadrotor
import numpy as np
import os
import logging
logging.basicConfig(
    format='%(asctime)s %(levelname)-8s %(message)s',
    level=logging.INFO,
    datefmt='%H:%M:%S')
from planner import PlannerInterface

class Config:
    def __init__(self):
        config = configparser.ConfigParser()
        config.read('config.ini')
        self.traj_path = f"{str(config['Trajectory']['traj_path'])}"
        self.traj_dt = int(config['Trajectory']['traj_dt'])
        self.default_takeoff_alt = float(config['DEFAULT']['TAKEOFF_ALTITUDE'])
        self.default_cmd_vel = float(config['DEFAULT']['CMD_VEL'])
        self.default_dt = float(config['DEFAULT']['DT'])
        self.ip_addr = str(config['DRONEKIT']['IP_ADDR'])

        if not os.path.exists(self.traj_path):
            logging.error(f'{self.traj_path} does not exist!')
            return



class State(Enum):
    INITIALIZED = 0
    ARMED = 1
    TAKEOFF = 2
    HOVER = 3
    LAND = 4

class Action(Enum):
    IS_ARMABLE = 0
    IS_ARMED = 1
    HAS_ARRIVED = 2
    DISARMED = 3

def generate_spiral_eight(num_points, x_scale=3.5, y_scale=2.0, t0=0.0):
  """
  Generates coordinates for points on a circle.

  Args:
      num_points (int): Number of points on the circle.
      x_scale (float, optional): Scaling factor for x-coordinate. Defaults to 2.0.
      y_scale (float, optional): Scaling factor for y-coordinate. Defaults to 2.0.
      t0 (float, optional): Offset value for the angle. Defaults to 0.0.

  Returns:
      tuple: A tuple containing two NumPy arrays (x_coordinates, y_coordinates).
  """
  theta = np.linspace(t0, t0 + 2*np.pi, num_points)  # Create angles for all points
  x = x_scale * np.cos(theta) * np.sin(theta)
  y = y_scale * np.sin(theta)
  return x, y

def calculate_derivaties(waypoints):
  """
  Calculates velocities for a given set of waypoints.

  Args:
      waypoints (list): A list of waypoints, where each waypoint is a tuple (x, y).

  Returns:
      list: A list of velocities, where each velocity is a tuple (vx, vy).
  """

  num_points = len(waypoints)
  velocities = []
  for i in range(1, num_points):
    # Get current and previous waypoints
    current_point = waypoints[i]
    prev_point = waypoints[i-1]

    # Calculate relative distance (assuming Euclidean distance)
    dx = current_point[0] - prev_point[0]
    dy = current_point[1] - prev_point[1]
    distance = np.sqrt(dx**2 + dy**2)
    if distance != 0.0:
        # Velocity is assumed to be constant between waypoints
        # (adjust this logic if you have additional information about speed)
        velocity = (dx / distance, dy / distance)
    else:
        velocity = (0.0, 0.0)

    velocities.append(velocity)
  velocity = (0.0, 0.0)
  velocities.append(velocity)

  return np.array(velocities)

class MainWindow(QMainWindow):
    def __init__(self):

        super().__init__()
        uic.loadUi('window_v1.ui', self)
        self.config = Config()
        self.state = State.LAND
        self.coord = np.zeros(3)
        self.progressBar.setValue(0)
        self.thread = Thread(target=self.connect, args=(self.config.ip_addr,))
        self.thread.start()
        self.num_points = 760

        self.transition = defaultdict(dict)
        self.transition[State.INITIALIZED][Action.IS_ARMABLE] = State.ARMED
        self.transition[State.ARMED][Action.IS_ARMED] = State.TAKEOFF
        self.transition[State.TAKEOFF][Action.HAS_ARRIVED] = State.HOVER
        self.transition[State.LAND][Action.DISARMED] = State.INITIALIZED

        self.launchAlt = self.config.default_takeoff_alt
        self.btnLaunch.clicked.connect(self.launch_click)

        self.btnWest.clicked.connect(self.west_click)
        self.btnEast.clicked.connect(self.east_click)
        self.btnNorth.clicked.connect(self.north_click)
        self.btnSouth.clicked.connect(self.south_click)
        self.btnRTL.clicked.connect(self.rtl_click)

        self.btnUp.clicked.connect(self.up_click)
        self.btnDown.clicked.connect(self.down_click)
        self.btnSendTraj.clicked.connect(self.sendTrajectory)
        self.maxVelText.textChanged.connect(self.updateVel)

        # intialize buttons
        self.btnLaunch.setEnabled(False)
        self.btnSendTraj.setEnabled(False)

        # add visualizer
        self.quad = Quadrotor(size=0.5)



        # planner interface
        self.plannerInterface = PlannerInterface()
        self.drone_view.addWidget(self.quad.canvas)
        self.plan_view.addWidget(self.plannerInterface.canvas)

        ## add buttons
        self.planButton.clicked.connect(self.plannerInterface.plan)
        self.plannerInterface.startTxt = self.goalInputText
        self.plannerInterface.goalTxt = self.startInputText
        self.plannerInterface.startRadio = self.startPos
        self.updateButton.clicked.connect(self.plannerInterface.updateButton)



    def updateVel(self):
        def speed_to_points(x):
            # −39.27135903x2+107.71750607x+661.00327388
            return int(-39.27135903 * (x ** 2) + 107.71750607 * x + 661.00327388)

        text = self.maxVelText.text()
        try:
            maxVel = float(text)
            self.num_points = speed_to_points(maxVel)
            print(f'max vel changed to {maxVel} num points {self.num_points}')
        except:
            pass





    def traj_callback(self):
        self.traj_index += 1
        if self.traj_index < len(self.traj):
            target = self.traj[self.traj_index]
            target_vel = self.traj_vel[self.traj_index]
            target_acc = self.traj_acc[self.traj_index]
            dt = self.config.traj_dt / 1000.0

            setpoint = np.vstack((target, target_vel, target_acc)).flatten()
            current = np.squeeze(self.xEst)
            error = setpoint - current
            K = np.array([
                [1, 0, dt, 0, dt * dt / 2.0, 0],
                [0, 1, 0, dt, 0, dt * dt / 2.0]
            ])
            vx, vy = K @ error

            self.publish_cmd_vel(vy, vx, 0.0)
            z = np.array([[self.coord[0]], [self.coord[1]]])
            u = np.array([[vx], [vy]])
            self.xEst, self.PEst = ekf_estimation(self.xEst, self.PEst, z, u, dt)

            self.VMAX = max(self.VMAX, np.sqrt(self.xEst[2, 0] ** 2 + self.xEst[3, 0] ** 2))
            logging.debug(f"x = {self.xEst[0, 0]:.3f} y = {self.xEst[1, 0]:.3f} vx = {self.xEst[2, 0]:.3f} vy = {self.xEst[3, 0]:.3f} ")
            logging.info(f"VMAX = {self.VMAX}")

        elif self.traj_index - 10 < len(self.traj):
            self.publish_cmd_vel(0, 0, 0)
        else:
            logging.debug("trajectory timer stopped")
            self.traj_timer.stop()

    def sendTrajectory(self):



        if not os.path.exists(self.config.traj_path):
            logging.error(f'{self.config.traj_path} does not exist!')
            return
        logging.info('sending trajectory')

        if self.state == State.HOVER:
            self.VMAX = 0.0
            self.xEst = np.zeros((6, 1), dtype=np.float32)
            self.PEst = np.eye(6, dtype=np.float32)
            self.xEst[0, 0] = self.coord[0]
            self.xEst[1, 0] = self.coord[1]
            # self.traj = np.loadtxt(self.config.traj_path, delimiter=",")

            xx, yy = generate_spiral_eight(num_points=self.num_points)
            self.traj = np.column_stack((xx, yy))
            self.traj_vel = calculate_derivaties(self.traj)
            self.traj_acc = calculate_derivaties(self.traj_vel)
            print(self.traj.shape, self.traj_vel.shape, self.traj_acc.shape)

            self.traj_timer = QTimer()
            self.traj_index = 0
            self.traj_timer.timeout.connect(self.traj_callback)
            self.traj_timer.start(self.config.traj_dt)

    def connect(self, ip_addr):
        self.connection_string = ip_addr
        self.sitl = None

        # Start SITL if no connection string specified
        if  len(self.connection_string) < 3:
            import dronekit_sitl
            self.sitl = dronekit_sitl.start_default()
            self.connection_string = self.sitl.connection_string()

        # Connect to the Vehicle
        logging.info('Connecting to vehicle on: %s' % self.connection_string)

        self.vehicle = connect(self.connection_string, wait_ready=True)
        self.progressBar.setValue(25)

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

        # change state
        self.state = State.INITIALIZED
        self.btnLaunch.setEnabled(True)

    def updateFlightModeGUI(self, value):
        logging.info(f'flight mode change to {value}')
        index, mode = str(value).split(':')
        self.lblFlightModeValue.setText(mode)

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
        self.coord[0] = x
        self.coord[1] = y
        self.coord[2] = z

        heading = np.deg2rad(45)
        self.lblLongValue.setText(f"{x:.4f}")
        self.lblLatValue.setText(f"{y:.4f}")
        self.lblAltValue.setText(f"{z:.4f}")

        self.quad.update_pose(x,y,z,0,0,heading)

    def addObserverAndInit(self, name, cb):
        """We go ahead and call our observer once at startup to get an initial value"""
        self.vehicle.add_attribute_listener(name, cb)

    def getAction(self, state:State):
        """get an action based on the state of the vehicle"""
        if state == State.INITIALIZED and self.vehicle.is_armable:
            return Action.IS_ARMABLE
        elif state == State.ARMED and self.vehicle.armed and self.vehicle.mode.name == 'GUIDED':
            return  Action.IS_ARMED
        elif state == State.TAKEOFF  and self.vehicle.location.global_relative_frame.alt >= self.launchAlt * 0.95:
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
            self.progressBar.setValue(50)

        elif self.state == State.TAKEOFF:
            self.vehicle.simple_takeoff(self.launchAlt)
            self.progressBar.setValue(75)

        elif self.state == State.HOVER:
            logging.info("vehicle reached to hovering position")
            self.progressBar.setValue(100)
            self.btnSendTraj.setEnabled(True)


        elif self.state == State.INITIALIZED:
            logging.info("vehicle landed")
            self.timer.stop()

            ############### MAV LINK communication ##########################################################################

    def publish_cmd_vel(self, velocity_x, velocity_y, velocity_z):
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
            time.sleep(self.config.default_dt)
            elapsed_time += self.config.default_dt


    ############### Joystick communication ##########################################################################
    def vehicle_validation(self, function):
        if self.vehicle.mode == "GUIDED":
            logging.debug('button clicked ', function.__name__)
            function()

    def west_click(self):
        if self.state != State.HOVER:
            logging.warning("[Invalid request]: moving west is not possible")
            return
        @self.vehicle_validation
        def west_wrapped():
            self.send_ned_velocity(0, -self.config.default_cmd_vel, 0, 1)
            # self.send_ned_velocity(0, 0, 0, 1)

    def east_click(self):
        if self.state != State.HOVER:
            logging.warning("[Invalid request]: moving east is not possible")
            return
        @self.vehicle_validation
        def east_wrapped():
            self.send_ned_velocity(0, self.config.default_cmd_vel, 0, 1)
            # self.send_ned_velocity(0, 0, 0, 1)

    def north_click(self):
        if self.state != State.HOVER:
            logging.warning("[Invalid request]: moving north is not possible")
            return
        @self.vehicle_validation
        def north_wrapped():
            self.send_ned_velocity(self.config.default_cmd_vel, 0, 0, 1)
            # self.send_ned_velocity(0, 0, 0, 1)

    def south_click(self):
        if self.state != State.HOVER:
            logging.warning("[Invalid request]: moving south is not possible")
            return
        @self.vehicle_validation
        def south_wrapped():
            self.send_ned_velocity(-self.config.default_cmd_vel, 0, 0, 1)
            # self.send_ned_velocity(0, 0, 0, 1)

    def rtl_click(self):
        if self.state == State.LAND or self.state == State.INITIALIZED:
            logging.warning("[Invalid request]: landing is not possible")
            return
        @self.vehicle_validation
        def rtl_wrapped():
            self.vehicle.mode = VehicleMode("LAND")
            self.state = State.LAND


    def up_click(self):
        if self.state != State.HOVER:
            logging.warning("[Invalid request]: moving up is not possible")
            return
        @self.vehicle_validation
        def up_wrapped():
            alt = self.vehicle.location.global_relative_frame.alt
            if alt < 3:
                self.send_ned_velocity(0, 0, -0.5 * self.config.default_cmd_vel, 1)
                # self.send_ned_velocity(0, 0, 0, 1)

    def down_click(self):
        if self.state != State.HOVER:
            logging.warning("[Invalid request]: moving down is not possible")
            return
        @self.vehicle_validation
        def down_wrapped():
            alt = self.vehicle.location.global_relative_frame.alt
            if alt > 0.5:
                self.send_ned_velocity(0, 0, 0.5 * self.config.default_cmd_vel, 1)
                # self.send_ned_velocity(0, 0, 0, 1)


    def launch_click(self):
        """
        Arms vehicle and fly to self.alt
        """
        logging.debug('launch button pressed')
        if self.state == State.INITIALIZED:
            logging.info('initializeing taking off ...')
            self.timer = QTimer()
            self.timer.timeout.connect(self.timerCallback)
            self.timer.start(1000)
        else:
            logging.warning("launch has not been initialized !")




if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())