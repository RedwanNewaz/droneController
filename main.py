#!/home/airlab/anaconda3/envs/droneController3D/bin/python
from PyQt5.QtWidgets import QApplication, QTableWidgetItem, QMainWindow
import sys
from PyQt5 import uic
from PyQt5.QtCore import QTimer
from viewer.Quadrotor import Quadrotor
import argparse

from simulator import SimulatorInterface
from controller import  ControllerInterface
from robot import RobotInterface, ekf_estimation, EKF4D
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


class MainWindow(QMainWindow):
    def __init__(self, args):

        super().__init__()
        uic.loadUi('window_v1.ui', self)

        self.coord = np.zeros(3)
        self.progressBar.setValue(0)

        self.num_points = 760
        self.traj_dt = 200
        self._robot = None

        if args.sim:
            self.radioSim.setChecked(True)
            self.radioPhysical.setEnabled(False)
        else:
            self.radioSim.setEnabled(False)

        self.updateExpType()

       

        # updateExpType
        # self.radioSim.toggled.connect(self.updateExpType)
        # self.radioPhysical.toggled.connect(self.updateExpType)

        

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
        if self._robot.state.name == "HOVER":
            self._controller.calib(self.coord.tolist())


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


    def planUpdate(self):
        self.plannerInterface.updateButton()
        self.quad.sendTrajectory()


    def updateExpType(self):
        print("exp type changed")
        self.__isSim = self.radioSim.isChecked()

        if self.__isSim:
            self._robot = SimulatorInterface(self)
        else:
            self._robot = RobotInterface(self)
       
        self._robot.start()
    

    def onTimeout(self):

        if self._robot.state.name != "HOVER":
            self._controller.publish_cmd_vel(0, 0, 0)
            logging.info("geofence enforce emergency landing ..")
            self.traj_timer.stop()
            return

        self.traj_index += 1
        if self.traj_index < len(self.traj):

            target = self.traj[self.traj_index]
            target_vel = self.traj_vel[self.traj_index]
            dt = self._dt * self.traj_dt / 1000.0


            altitude = 1.0
            headingAngle = 0.0
            setpoint = np.array([target[0], target[1], altitude, headingAngle, target_vel[0], target_vel[1], 0.0, 0.0])
            # logging.info(f"setpoint = {setpoint}")
            current = np.squeeze(self.filter.xEst)
            # logging.info(f"current = {current}")
            error = setpoint - current

            K = np.array([
                [1.0, 0.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0, 0.0, dt, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, dt, 0.0],
                [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, dt]
            ])
            u = np.squeeze(K @ error)
            z = np.array([self.coord[0], self.coord[1], self.coord[2], 0.0]).reshape((4, 1))
            logging.info(f'u = {u}')
            if self.__isSim:
                self._controller.publish_cmd_vel(u[1], u[0], 0.0)
            else:
                self._controller.publish_cmd_vel(-u[0], -u[1], 0.0)
            self.filter.update(u, z, dt)


        elif self.traj_index - 10 < len(self.traj):
            self._controller.publish_cmd_vel(0, 0, 0)
        else:
            logging.info("trajectory timer stopped")
            self.traj_timer.stop()

    
    def sendTrajectory(self):
        """excute a trajectory and update the plotter."""
  

        self.VMAX = 0.0

        xEst = np.zeros((8, 1), dtype=np.float32)
        for i in range(3):
            xEst[i, 0] = self.coord[i]
        self.filter = EKF4D(xEst)



     
        self.traj = np.loadtxt('trajs/traj.csv', delimiter=',')[:, :2] 
        # compute trajectory
        self.traj_vel = calculate_derivaties(self.traj)
        logging.info(f'sending trajectory points = {self.traj.shape}, vel = {self.traj_vel.shape}')

        self.quad.sendTrajectory()

        self.traj_timer = QTimer()
        self.traj_timer.stop()
        self.traj_index = 0
        self.traj_timer.timeout.connect(self.onTimeout)
        self.traj_timer.start(self.traj_dt)



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', action='store_true', help='simulation')
    args = parser.parse_args()

    app = QApplication(sys.argv)
    window = MainWindow(args)
    window.show()
    sys.exit(app.exec())