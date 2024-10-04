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

class MainWindow(QMainWindow):
    def __init__(self):

        super().__init__()
        uic.loadUi('window_v1.ui', self)

        self.coord = np.zeros(3)
        self.progressBar.setValue(0)

        self.num_points = 760

        self._sim = SimulatorInterface(self)
        self._sim.start()

        self._controller = ControllerInterface(self, self._sim)
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

        self.drone_view.addWidget(self.quad.plotter.interactor)
        for obstacle in config['obstacle_list']:
            self.quad.add_cube(obstacle)
        self.quad.update()

    def onTimeout(self):
        if self._trajIndex < len(self._traj):
            position = self._traj[self._trajIndex]
            vel = position - self.coord
            self._controller.publish_cmd_vel(vel[1], vel[0], -vel[2])
        elif self._trajIndex - 10 < len(self._traj):
            self._controller.publish_cmd_vel(0, 0, 0)
        else:
            self._timer.stop()
        self._trajIndex += 1
    
    def sendTrajectory(self):
        """excute a trajectory and update the plotter."""
        self._trajIndex = 0
        self._traj = np.loadtxt('trajs/traj.csv', delimiter=',') 
        self._timer = QTimer()
        self._timer.timeout.connect(self.onTimeout)
        self._timer.start(int(self._dt * 1000))



if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())