import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle
from .RRTStar import RRTStar
from .CollisionChecker import CollisionChecker
from .PathSmoothing import path_smoothing
import yaml
class PlannerInterface:
    def __init__(self):
        self.fig = Figure()
        self.ax = self.fig.add_subplot()
        self.canvas = FigureCanvasQTAgg(self.fig)
        self.cid = self.canvas.mpl_connect('button_press_event', self.onclick)
        self.goalTxt = None
        self.startTxt = None
        self.startRadio = None
        self.pathPlot = None
        self.envComboBox = None

        colors = ['red', 'green']
        self.scatter = self.ax.scatter([-2, -2], [-2, -2], c=colors)


    def config_env(self, config):
        # ====Search Path with RRT====

        self.obstacle_list = config['obstacle_list']
        self.boundary = config['boundary']
        self.ax.set_xlim(self.boundary[:2])
        self.ax.set_ylim(self.boundary[2:])
        self.draw_obstacles(self.obstacle_list)

        # draw target area
        rect = Rectangle((self.boundary[0], self.boundary[2]),
                         self.boundary[1] - self.boundary[0],
                         self.boundary[3] - self.boundary[2],
                         facecolor='none', edgecolor='blue', linewidth=2)
        self.ax.add_patch(rect)
    def update_point(self, index, new_x, new_y):
        offsets = self.scatter.get_offsets()
        offsets[index] = [new_x, new_y]
        self.scatter.set_offsets(offsets)
        self.canvas.draw_idle()
    def updateButton(self):
        if self.pathPlot:
            print(self.pathPlot.get_xdata())
            print(self.pathPlot.get_ydata())


    def plan(self):
        # print('plan with', self.startTxt.toPlainText(), self.goalTxt.toPlainText())
        offsets = self.scatter.get_offsets()
        start, goal = offsets
        print(self.scatter.get_offsets())
        self.demo(start, goal)

    def draw_obstacles(self, obstacle_list):
        for obstacle in obstacle_list:
            center = np.array(obstacle[:2])
            radius = obstacle[2]
            origin = center - (radius / 2)
            rect = Rectangle(origin.tolist(), radius, radius, facecolor='grey', edgecolor='black', linewidth=2)

            self.ax.add_patch(rect)

    def onclick(self, event):
        if event.xdata is not None and event.ydata is not None:
            print(f"Clicked coordinates: x={event.xdata:.2f}, y={event.ydata:.2f}")
            msg = "{:.2f},{:.2f}".format(event.xdata, event.ydata)
            if self.startRadio.isChecked():
                self.startTxt.setPlainText(msg)
                self.update_point(0, event.xdata, event.ydata)
            else:
                self.goalTxt.setPlainText(msg)
                self.update_point(1, event.xdata, event.ydata)


    def demo(self, start, goal):

        # config collision checker
        collisionManager = CollisionChecker(self.obstacle_list, self.boundary)
        # Set Initial parameters
        rrt_star = RRTStar(
            start=start,
            goal=goal,
            rand_area=[-2, 15],
            obstacle_list=self.obstacle_list,
            expand_dis=1,
            check_collision=collisionManager.check_collision,
            robot_radius=0.4)
        path = rrt_star.planning(animation=False)

        if path is None:
            print("Cannot find path")
        else:
            while len(self.ax.lines):
                for line in self.ax.lines:
                    line.remove()
            self.ax.plot([x for (x, y) in path], [y for (x, y) in path], 'g--')
            path = path_smoothing(path, rrt_star.max_iter, self.obstacle_list)
            self.pathPlot, = self.ax.plot([x for (x, y) in path], [y for (x, y) in path], 'r--')

            # self.ax.grid(True)
            print("found path!!: ", path)
            self.canvas.draw_idle()
