import pyvista as pv
from pyvistaqt import QtInteractor
from PyQt5 import QtWidgets
import os
import numpy as np
current_dir = os.path.dirname(__file__)
class Quadrotor:
    def __init__(self, centralwidget, env_extent, shape):
        self.plotter = QtInteractor(centralwidget)
        # Add PyVista content to the plotter

        robot_mesh = pv.read(os.path.join(current_dir, 'Quadcopter.stl'))

        self.plotter.add_mesh(robot_mesh)

        # x_range = env_extent[1] - env_extent[0]
        # y_range = env_extent[3] - env_extent[2]
        # z_range = 3
        # resolution = [x_range / shape[0], y_range / shape[1], z_range]
        #
        # origin = np.array([env_extent[0], env_extent[2], 0])
        # image_data = pv.ImageData(dimensions=shape, origin=origin, spacing=resolution)
        # map = np.zeros(shape)
        # image_data.point_data["values"] = map.flatten()
        # terrain = image_data.warp_by_scalar()
        # self.plotter.add_mesh(terrain, cmap="jet", lighting=True, show_scalar_bar=False)

        # Set up the camera
        self.plotter.show_grid()
        self.plotter.reset_camera()

