import numpy as np
from PyQt5.QtCore import QTimer, QObject, pyqtSignal
from .TrajectoryGenerator import generate_trajectory

class TrajectoryInterface(QObject):
    setPoint = pyqtSignal(str)
    def __init__(self, path, dt, parent=None):
        self._path = np.column_stack((path, np.ones(len(path))))
        self._dt = int(1000 * dt) #msec
        self._timer = QTimer()
        self._trajIndex = 0
        super().__init__(parent)

    def start(self):
        self._traj = generate_trajectory(self._path, self._dt / 1000.0)
        self._timer.timeout.connect(self.onTimeout)
        self._timer.start(self._dt)
        print("trajectory execution started")
    def stop(self):
        self._timer.stop()

    def onTimeout(self):
        #TODO relace path with trajectory
        if self._trajIndex <= len(self._traj) - 1:
            coord = ",".join(map(str, self._traj[self._trajIndex]))
            # print('sending ', coord)
            self.setPoint.emit(coord)
        else:
            self.stop()
            self._trajIndex = 0
        self._trajIndex += 1
