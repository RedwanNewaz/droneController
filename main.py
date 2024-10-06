#!/home/airlab/anaconda3/envs/droneController3D/bin/python
from PyQt5.QtWidgets import QApplication, QTableWidgetItem, QMainWindow
import sys
from PyQt5 import uic
from PyQt5.QtCore import QTimer
from viewer.Quadrotor import Quadrotor
import argparse

from simulator import SimulatorInterface
from controller import  ControllerInterface
from robot import RobotInterface
from trajectory import TrajectoryFollower
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
    def __init__(self, args):

        super().__init__()
        uic.loadUi('window_v1.ui', self)

        self.coord = np.zeros(3)
        self.progressBar.setValue(0)

        self.num_points = 760
        self.traj_dt = 200
        self._robot = None
        self._quad = None
        self._trajFollower = None

        if args.sim:
            self.radioSim.setChecked(True)
            self.radioPhysical.setEnabled(False)
        else:
            self.radioSim.setEnabled(False)

        self.updateExpType()

        self._controller = ControllerInterface(self, self._robot)
        self._quad = None

        # planner interface
        self.plannerInterface = PlannerInterface(self)
        # env config
        self.loadEnvCombo.currentTextChanged.connect(self.on_combobox_changed)
        for env in Path('envs').glob('*.yaml'):
            self.loadEnvCombo.addItem(env.name)

    def updateCoord(self, x, y, z):
        self.coord[0] = x
        self.coord[1] = y
        self.coord[2] = z
        self.lblLongValue.setText(f"{self.coord[0]:.4f}")
        self.lblLatValue.setText(f"{self.coord[1]:.4f}")
        self.lblAltValue.setText(f"{self.coord[2]:.4f}")

        __position = f"{x},{y}"
        self.plannerInterface.updateRobot(__position)
        if self._quad:
            position = self.coord / self._quad.scale
            self._quad.actor.position = position.tolist()
        if self._robot.state.name == "HOVER":
            self._controller.calib(self.coord.tolist())
        elif self._trajFollower is not None:
            self._trajFollower.cancel()



    def on_combobox_changed(self, value):
        print("Current text:", value)
        env = 'envs/%s' % value
        with open(env) as file:
            config = yaml.load(file, Loader=yaml.SafeLoader)
        self.plannerInterface.config_env(config)

        # Create the PyVista QtInteractor
        boundary = config['boundary']
        self._dt = dt = config['dt']
        origin = config['origin']
        shape = np.array([boundary[1] - boundary[0], boundary[3] - boundary[2], 0]).astype('int')

        if self._quad is None:
            self._quad = Quadrotor(self.centralwidget, dt, shape)
            self._quad.init(shape, origin)
        else:
            self._quad.init(shape, origin)
        # add send trajectory button
        # self.btnSendTraj.clicked.connect(self.quad.sendTrajectory)
        self.btnSendTraj.clicked.connect(self.sendTrajectory)
        self.updateButton.clicked.connect(self.planUpdate)

        self.drone_view.addWidget(self._quad.plotter.interactor)
        for obstacle in config['obstacle_list']:
            self._quad.add_cube(obstacle)
        self._quad.update()

        # configure trajectory follower
        self._trajFollower = TrajectoryFollower(self._dt, self.traj_dt, self.coord, self._controller, self.__isSim)


    def planUpdate(self):
        self.plannerInterface.updateButton()
        self._quad.sendTrajectory()


    def updateExpType(self):
        self.__isSim = self.radioSim.isChecked()
        self._robot = SimulatorInterface(self) if self.__isSim else RobotInterface(self)
        self._robot.start()

    def sendTrajectory(self):
        """excute a trajectory and update the plotter."""
        self._quad.sendTrajectory()
        self._trajFollower.execute()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', action='store_true', help='simulation')
    args = parser.parse_args()

    app = QApplication(sys.argv)
    window = MainWindow(args)
    window.show()
    sys.exit(app.exec())