import numpy as np
from PyQt5.QtCore import QTimer, QObject, pyqtSignal
from .TrajectoryGenerator import generate_trajectory

class TrajectoryInterface(QObject):
    setPoint = pyqtSignal(str)
    def __init__(self, path, dt, parent=None):
        self._path = np.column_stack((path, np.ones(len(path))))
        self._path = self.interpolate(self._path)
        self._dt = int(1000 * dt) #msec
        self._timer = QTimer()
        self._trajIndex = 0
        super().__init__(parent)
    
    @staticmethod
    def generate_interpolated_points(p1, p2):
        p1 = np.array(p1)
        p2 = np.array(p2)
        dist = np.linalg.norm(p2 - p1)
        # V = 0.0001 # m/s
        # V = 0.00008 # m/s
        V = 0.25
        num_points = int(dist / V)
        # Generate evenly spaced values from 0 to 1
        t = np.linspace(0, 1, num_points)
        
        # Interpolate between p1 and p2
        interpolated_points = p1[np.newaxis, :] * (1 - t)[:, np.newaxis] + p2[np.newaxis, :] * t[:, np.newaxis]
        
        return interpolated_points
    
    def interpolate(self, path):
        traj = []
        for i, p in enumerate(path):
            if i == 0:
                continue

            points = self.generate_interpolated_points(path[i-1], p)
            traj.extend(points)
        return traj

    def start(self):
        self._traj = generate_trajectory(self._path, self._dt / 1000.0)
        # self._timer.timeout.connect(self.onTimeout)
        # self._timer.start(self._dt)
        print("trajectory execution started")
        np.savetxt('trajs/traj.csv', self._traj, delimiter=',')
    def stop(self):
        self._timer.stop()

    # def onTimeout(self):
    #     #TODO relace path with trajectory
    #     if self._trajIndex <= len(self._traj) - 1:
    #         coord = ",".join(map(str, self._traj[self._trajIndex]))
    #         # print('sending ', coord)
    #         self.setPoint.emit(coord)
    #     else:
    #         self.stop()
    #         self._trajIndex = 0
    #     self._trajIndex += 1
