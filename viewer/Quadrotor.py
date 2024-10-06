import pyvista as pv
from pyvistaqt import QtInteractor
from PyQt5 import QtWidgets
import os
import numpy as np
current_dir = os.path.dirname(__file__)

import pyvista as pv
from pyvistaqt import QtInteractor
import os
import numpy as np
from PyQt5.QtCore import QTimer, QObject, pyqtSignal, pyqtSlot
from .angle import angle_mod

VIEWER_SCALE = 10.0 # meter to cm
camera_position = (-0.440, -0.880, 1.938)
camera_focal_point = (0.05, 0.00, 0.06)
camera_up = (0.0, 0.65, 1.0)
class Quadrotor(QObject):
    def __init__(self, centralwidget, dt, grid_shape):
        super(Quadrotor, self).__init__()
        self.plotter = QtInteractor(centralwidget)
        self.plotter.camera.position = camera_position
        self.plotter.camera.focal_point = camera_focal_point
        self.plotter.camera.up = camera_up
        self._timer = QTimer()
        self._dt = dt
        self._trajIndex = 0
        self.viewer_points = None
        self.scale = VIEWER_SCALE

    def init(self, grid_shape, origin):
        # Clears all actors
        self.plotter.clear()
        # Create the grid world
        dx, dy, dz = 0.1, 0.1, 0.1  # 10 cm spacing

        self.grid = pv.ImageData()
        self.grid.dimensions = np.array(grid_shape) + 1  # Add 1 to create cells

        self.grid.origin = (0, 0, 0)
        if origin == 'center':
            self.grid.origin = (-grid_shape[0] / 20, -grid_shape[1] / 20, 0)
     
        self.grid.spacing = (dx, dy, dz)

        # Add the grid to the plotter
        self.plotter.add_mesh(self.grid, style='wireframe', color='gray', opacity=0.5)

        # Load and scale the robot mesh
        self.setActor((0.0, 0.0, 0.1))
        self.plotter.add_axes()

        # Set up the camera
        # self.plotter.camera_position = 'iso'
        # self.plotter.reset_camera()
        # print( 'camera ppos',   self.plotter.camera.position,
        # self.plotter.camera.focal_point,   self.plotter.camera.up)

    def setActor(self, position):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        robot_mesh = pv.read(os.path.join(current_dir, 'Quadcopter.stl'))
        robot_mesh = robot_mesh.scale([0.0003192, 0.0003192, 0.0003192], inplace=False)
        rotate_degree = 90
        self.robot_mesh = robot_mesh.rotate_x(rotate_degree).translate(
            position
        )
        # Add the robot mesh to the plotter
        self.actor = self.plotter.add_mesh(self.robot_mesh, color='red')

    def add_cube(self, obstacle, color='blue'):
        """Add a cube to the grid world at the specified position."""
        cube_size = obstacle[-1] / VIEWER_SCALE # cm
        position = np.array([obstacle[0], obstacle[1], 1]) / VIEWER_SCALE # cm
        cube = pv.Cube(center=position, x_length=cube_size, y_length=cube_size, z_length=cube_size)
        self.plotter.add_mesh(cube, color=color)

    def update(self):
        """Update the plotter."""
        self.plotter.update()


    def onTimeout(self):
        if self._trajIndex < len(self._traj):
            position = self._traj[self._trajIndex]
            currentPos = np.array(self.actor.position)
            self.actor.position = position.tolist()

            # dy, dx, dz = position - currentPos
            # theta = np.deg2rad(self.actor.orientation[-1])
            # alpha = angle_mod(np.arctan2(dy, dx) - theta)
            # self.actor.orientation = [0, 0, np.rad2deg(alpha)]


        else:
            self._timer.stop()
            self._trajIndex = 0
        self._trajIndex += 1
        self.update()

    @pyqtSlot()
    def sendTrajectory(self):
        """excute a trajectory and update the plotter."""
        self._trajIndex = 0
        self._traj = np.loadtxt('trajs/traj.csv', delimiter=',') / VIEWER_SCALE
        # Create a PolyData object from the trajectory points
        trajectory = pv.PolyData(self._traj)
        if self.viewer_points is not None:
            self.plotter.remove_actor(self.viewer_points)


        # Add the trajectory points to the plotter
        self.viewer_points = self.plotter.add_points(trajectory, render_points_as_spheres=True, point_size=10, color='grey')
        print(self._traj.shape)
        # self._timer.timeout.connect(self.onTimeout)
        # self._timer.start(int(self._dt * 1000))


