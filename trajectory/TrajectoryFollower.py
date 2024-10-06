import numpy as np
from PyQt5.QtCore import QTimer, QObject, pyqtSignal
from robot import EKF4D
import logging
logging.basicConfig(
    format='%(asctime)s %(levelname)-8s %(message)s',
    level=logging.INFO,
    datefmt='%H:%M:%S')
class TrajectoryFollower(QObject):
    def __init__(self, dt, traj_dt, coord, controller, expType):
        self._dt = dt
        self.traj_dt = traj_dt

        self.coord = coord # numpy shared array for coordinate
        self._controller = controller
        self.__isSim = expType
        self.__active = False
        super().__init__()

    def stopAircraft(self):
        self.traj_index += 1
        self._controller.publish_cmd_vel(0, 0, 0)
        MAX_ZERO = 10
        logging.info(f"aircraft is stopping in ..{MAX_ZERO - self.traj_index}")
        if self.traj_index >= MAX_ZERO:
            self.traj_index = 0
            self.traj_timer.stop()

    def cancel(self):
        if not self.__active:
            return

        self._controller.publish_cmd_vel(0, 0, 0)
        logging.info("trajectory timer stopped")
        self.traj_timer.stop()
        self.__active = False

        self.traj_index = 0
        self.traj_timer = QTimer()
        self.traj_timer.timeout.connect(self.stopAircraft)
        self.traj_timer.start(self.traj_dt)

    def onTimeout(self):
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

        else:
            self.cancel()

    def execute(self):
        self.traj = np.loadtxt('trajs/traj.csv', delimiter=',')[:, :2]
        self.traj_vel = self.calculate_derivaties(self.traj)

        # configure EKF
        self.VMAX = 0.0
        xEst = np.zeros((8, 1), dtype=np.float32)
        for i in range(3):
            xEst[i, 0] = self.coord[i]
        self.filter = EKF4D(xEst)

        # start the timer
        self.__active = True

        self.traj_index = 0
        self.traj_timer = QTimer()
        self.traj_timer.timeout.connect(self.onTimeout)
        self.traj_timer.start(self.traj_dt)

    @staticmethod
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
            prev_point = waypoints[i - 1]

            # Calculate relative distance (assuming Euclidean distance)
            dx = current_point[0] - prev_point[0]
            dy = current_point[1] - prev_point[1]
            distance = np.sqrt(dx ** 2 + dy ** 2)
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
