#!/home/airlab/anaconda3/envs/droneController3D/bin/python
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

from simulator import SimulatorInterface
from controller import  ControllerInterface
from robot import RobotInterface, ekf_estimation
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

class MainWindow(QMainWindow):
    def __init__(self):

        super().__init__()
        uic.loadUi('window_v1.ui', self)

        self.coord = np.zeros(3)
        self.progressBar.setValue(0)

        self.num_points = 760
        self.traj_dt = 200

        self.__isSim = False

        if self.__isSim:
            self._robot = SimulatorInterface(self)
        else:
            self._robot = RobotInterface(self)
        self._robot.start()

        self._controller = ControllerInterface(self, self._robot)
        self.quad = None

        # planner interface
        self.plannerInterface = PlannerInterface(self)
        # env config
        self.loadEnvCombo.currentTextChanged.connect(self.on_combobox_changed)
        for env in Path('envs').glob('*.yaml'):
            self.loadEnvCombo.addItem(env.name)

        

     

    def updateCoord(self, x, y, z):
        # self.coord[0] = max(0, x)
        # self.coord[1] = max(0, y)
        # self.coord[2] = max(0, z)
        self.coord[0] = x
        self.coord[1] = y
        self.coord[2] = z
        self.lblLongValue.setText(f"{self.coord[0]:.4f}")
        self.lblLatValue.setText(f"{self.coord[1]:.4f}")
        self.lblAltValue.setText(f"{self.coord[2]:.4f}")

        __position = f"{x},{y}"
        self.plannerInterface.updateRobot(__position)
        if self.quad:
            position = self.coord / self.quad.scale
            self.quad.actor.position = position.tolist()


    def on_combobox_changed(self, value):
        print("Current text:", value)
        env = 'envs/%s' % value
        with open(env) as file:
            config = yaml.load(file, Loader=yaml.SafeLoader)
        self.plannerInterface.config_env(config)

        # Create the PyVista QtInteractor
        boundary = config['boundary']
        self._dt = dt = config['dt']
        shape = np.array([boundary[1] - boundary[0], boundary[3] - boundary[2], 0]).astype('int')
        self.quad = Quadrotor(self.centralwidget, dt, shape)
        # add send trajectory button
        # self.btnSendTraj.clicked.connect(self.quad.sendTrajectory)
        self.btnSendTraj.clicked.connect(self.sendTrajectory)
        self.updateButton.clicked.connect(self.planUpdate)

        self.drone_view.addWidget(self.quad.plotter.interactor)
        for obstacle in config['obstacle_list']:
            self.quad.add_cube(obstacle)
        self.quad.update()

    # def onTimeout(self):
    #     if self._trajIndex < len(self._traj):
    #         position = self._traj[self._trajIndex]
    #         vel = position - self.coord
    #         self._controller.publish_cmd_vel(vel[1], vel[0], -vel[2])
    #         # self._controller.publish_cmd_vel(-vel[1], -vel[0],0)
    #     elif self._trajIndex - 10 < len(self._traj):
    #         self._controller.publish_cmd_vel(0, 0, 0)
    #     else:
    #         self._timer.stop()
    #     self._trajIndex += 1
    
    # def sendTrajectory(self):
    #     """excute a trajectory and update the plotter."""
    #     self._trajIndex = 0
    #     self._traj = np.loadtxt('trajs/traj.csv', delimiter=',') 
    #     self._timer = QTimer()
    #     self._timer.timeout.connect(self.onTimeout)
    #     self._timer.start(int(self._dt * 1000))

    
    def planUpdate(self):
        self.plannerInterface.updateButton()
        self.quad.sendTrajectory()


    

    def onTimeout(self):

        if self._robot.state.name != "HOVER":
            self._controller.publish_cmd_vel(0, 0, 0)
            logging.info("geofence enforce emergency landing ..")
            self.traj_timer.stop()
            return

        self.traj_index += 1
        if self.traj_index < len(self.traj):
            # target = self.traj[self.traj_index][1:]
            # vel = target - self.coord[:3]
            # vx, vy, vz = vel

            target = self.traj[self.traj_index]
            target_vel = self.traj_vel[self.traj_index]
            target_acc = self.traj_acc[self.traj_index]
            dt = self._dt * self.traj_dt / 1000.0

            setpoint = np.vstack((target, target_vel, target_acc)).flatten()
            current = np.squeeze(self.xEst)
            error = setpoint - current
            K = np.array([
                [1, 0, dt, 0, dt * dt / 2.0, 0],
                [0, 1, 0, dt, 0, dt * dt / 2.0]
            ])
            vx, vy = K @ error

            logging.debug(f"[Traj] vx = {vx:.3f}, vy = {vy:.3f} ")

        
            if self.__isSim:
                self._controller.publish_cmd_vel(vy, vx, 0.0)
            else:
                self._controller.publish_cmd_vel(-vx, -vy, 0.0)

            # update ekf 
            z = np.array([[self.coord[0]], [self.coord[1]]])
            u = np.array([[vx], [vy]])
            #FIXME ekf_estimation 
            self.xEst, self.PEst = ekf_estimation(self.xEst, self.PEst, z, u, dt)
            self.VMAX = max(self.VMAX, np.sqrt(self.xEst[2, 0] ** 2 + self.xEst[3, 0] ** 2))
            logging.debug(f"VMAX = {self.VMAX}")
            # self.maxVelSignal.emit(self.VMAX)


        elif self.traj_index - 10 < len(self.traj):
            self._controller.publish_cmd_vel(0, 0, 0)
        else:
            logging.info("trajectory timer stopped")
            self.traj_timer.stop()

    
    def sendTrajectory(self):
        """excute a trajectory and update the plotter."""
  

        self.xEst = np.zeros((6, 1), dtype=np.float32)
        self.PEst = np.eye(6, dtype=np.float32)
        self.xEst[0, 0] = self.coord[0]
        self.xEst[1, 0] = self.coord[1]
        self.VMAX = 0.0


     
        self.traj = np.loadtxt('trajs/traj.csv', delimiter=',')[:, :2] 
        # xx, yy = generate_spiral_eight(num_points=500)
        # self.traj = np.column_stack((xx, yy))
        # compute trajectory 
        self.traj_vel = calculate_derivaties(self.traj)
        self.traj_acc = calculate_derivaties(self.traj_vel)
        logging.info(f'sending trajectory points = {self.traj.shape}, vel = {self.traj_vel.shape}, acc = {self.traj_acc.shape}')

        self.quad.sendTrajectory()

        self.traj_timer = QTimer()
        self.traj_timer.stop()
        self.traj_index = 0
        self.traj_timer.timeout.connect(self.onTimeout)
        self.traj_timer.start(self.traj_dt)



if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())