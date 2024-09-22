from dronekit import connect, VehicleMode
from pymavlink import mavutil

from PyQt5.QtWidgets import QApplication, QTableWidgetItem, QMainWindow
import sys
from PyQt5 import uic
from PyQt5.QtCore import QTimer
from threading import Thread
import PyQt5
from EKF import  ekf_estimation
from viewer.Quadrotor import Quadrotor

import configparser
from enum import Enum
from collections import defaultdict
import time
# from Quadrotor import Quadrotor
import numpy as np
import os
import logging
import yaml
from pathlib import Path
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

        self.coord = np.zeros(3)
        self.progressBar.setValue(0)

        self.num_points = 760



        #
        # self.btnLaunch.clicked.connect(self.launch_click)
        #
        # self.btnWest.clicked.connect(self.west_click)
        # self.btnEast.clicked.connect(self.east_click)
        # self.btnNorth.clicked.connect(self.north_click)
        # self.btnSouth.clicked.connect(self.south_click)
        # self.btnRTL.clicked.connect(self.rtl_click)
        #
        # self.btnUp.clicked.connect(self.up_click)
        # self.btnDown.clicked.connect(self.down_click)
        # self.btnSendTraj.clicked.connect(self.sendTrajectory)
        # self.maxVelText.textChanged.connect(self.updateVel)

        # intialize buttons
        self.btnLaunch.setEnabled(False)
        self.btnSendTraj.setEnabled(False)

        #################################







        ##################################33
        # add visualizer
        # self.quad = Quadrotor(size=0.5)
        # self.drone_view.addWidget(self.quad.canvas)





        # planner interface
        self.plannerInterface = PlannerInterface()

        self.plan_view.addWidget(self.plannerInterface.canvas)

        # env config
        self.loadEnvCombo.currentTextChanged.connect(self.on_combobox_changed)
        for env in Path('envs').glob('*.yaml'):
            self.loadEnvCombo.addItem(env.name)




        ## add buttons
        self.planButton.clicked.connect(self.plannerInterface.plan)
        self.plannerInterface.startTxt = self.goalInputText
        self.plannerInterface.goalTxt = self.startInputText
        self.plannerInterface.startRadio = self.startPos
        self.updateButton.clicked.connect(self.plannerInterface.updateButton)

        # self.plannerInterface.envComboBox = self.loadEnvCombo


    def on_combobox_changed(self, value):
        print("Current text:", value)
        env = 'envs/%s' % value
        with open(env) as file:
            config = yaml.load(file, Loader=yaml.SafeLoader)
        self.plannerInterface.config_env(config)

        # Create the PyVista QtInteractor
        boundary = config['boundary']
        shape = [10, 10, 5]
        self.quad = Quadrotor(self.centralwidget, boundary, shape)
        self.drone_view.addWidget(self.quad.plotter.interactor)





    #
    # def updateFlightModeGUI(self, value):
    #     logging.info(f'flight mode change to {value}')
    #     index, mode = str(value).split(':')
    #     self.lblFlightModeValue.setText(mode)
    #
    # def updateLocationGUI(self, location):
    #     # self.lblLongValue.setText(str(location.global_frame.lon))
    #     # self.lblLatValue.setText(str(location.global_frame.lat))
    #     # location.local_frame.
    #     x = location.local_frame.east
    #     y = location.local_frame.north
    #     z = location.local_frame.down
    #
    #     if x is None or y is None or z is None:
    #         x,  y, z = 0, 0, 0
    #     z = max(-z, 0.0)
    #     logging.debug(f'location: {x}, {y}, {z}')
    #     self.coord[0] = x
    #     self.coord[1] = y
    #     self.coord[2] = z
    #
    #     heading = np.deg2rad(45)
    #     self.lblLongValue.setText(f"{x:.4f}")
    #     self.lblLatValue.setText(f"{y:.4f}")
    #     self.lblAltValue.setText(f"{z:.4f}")
    #
    #     self.quad.update_pose(x,y,z,0,0,heading)











if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())