import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle
from planner.RRT.RRTStar import RRTStar
from planner.RRT.CollisionChecker import CollisionChecker
from planner.RRT.PathSmoothing import path_smoothing
from PyQt5.QtCore import QObject, pyqtSlot
from trajectory import TrajectoryInterface
from planner.VisibilityRoadmap import getRoadmap


class PlannerInterface(QObject):
    def __init__(self, mainWindow):
        self.fig = Figure()
        self.ax = self.fig.add_subplot()
        self.canvas = FigureCanvasQTAgg(self.fig)
        self.cid = self.canvas.mpl_connect('button_press_event', self.onclick)
        self.__start  = None
        
        self.mainWindow = mainWindow
        self.mainWindow.plan_view.addWidget(self.canvas)

        self.goalTxt = self.mainWindow.goalInputText
        self.startTxt = self.mainWindow.startInputText
        self.startRadio = self.mainWindow.startPos
        self.goalRadio = self.mainWindow.goalPos
        self.mainWindow.planButton.clicked.connect(self.plan)
        # self.mainWindow.updateButton.clicked.connect(self.updateButton)

        self.pathPlot = None
        self.__roadmap = None
    

        super().__init__()

        # FIXME get start poisition from simulation
        self.startRadio.setEnabled(False)
        self.goalRadio.setChecked(True)


    def config_env(self, config):
        # ====Search Path with RRT====
        self.ax.cla()
        colors = ['red', 'green']
        self.scatter = self.ax.scatter([-2, -2], [-2, -2], c=colors)

        self.obstacle_list = config['obstacle_list']
        boundary = np.array(config['boundary'])
        origin = config['origin']
        self._shape = boundary
        if origin == 'center':
            self.boundary = boundary - (boundary / 2)
            self.boundary[0] = -self.boundary[1]
            self.boundary[2] = -self.boundary[3]
            self.origin = np.array([self.boundary[0], self.boundary[2]])
        else:
            self.boundary = boundary
            self.origin = np.array([0, 0])
        self.dt = config['dt']
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
    
    
    @pyqtSlot(str)
    def updateRobot(self, position):
        # print('received ', position)
        coord = list(map(float, position.split(",")))
        self.update_point(0, coord[0], coord[1])
        self.__start = np.array([coord[0], coord[1]])

    def updateButton(self):
        path = np.column_stack((self.pathPlot.get_xdata(), self.pathPlot.get_ydata()))[::-1]
        self.trajExec = TrajectoryInterface(path, dt=self.dt)
        self.trajExec.start()
        # if self.pathPlot:
        #     path = np.column_stack((self.pathPlot.get_xdata(), self.pathPlot.get_ydata()))[::-1]

        #     self.trajExec = TrajectoryInterface(path, dt=self.dt)
        #     # self.trajExec.setPoint.connect(self.updateRobot)
        #     self.trajExec.start()



    def plan(self):
        # print('plan with', self.startTxt.toPlainText(), self.goalTxt.toPlainText())
        offsets = self.scatter.get_offsets()
        # start, goal = offsets
        print(self.scatter.get_offsets())
        self.demo_visibility_roadmap(self.__start, self.__goal)

    def draw_obstacles(self, obstacle_list):
        for obstacle in obstacle_list:
            center = np.array(obstacle[:2])
            radius = obstacle[2]
            origin = center - (radius / 2)
            rect = Rectangle(origin.tolist(), radius, radius, facecolor='grey', edgecolor='black', linewidth=2)

            self.ax.add_patch(rect)

    def onclick(self, event):
        # update start position
        msg = "{:.3f},{:.3f}".format(self.mainWindow.coord[0], self.mainWindow.coord[1])
        self.startTxt.setPlainText(msg)
        self.update_point(0, self.mainWindow.coord[0], self.mainWindow.coord[1])

        if event.xdata is not None and event.ydata is not None:
            print(f"Clicked coordinates: x={event.xdata:.2f}, y={event.ydata:.2f}")
            msg = "{:.2f},{:.2f}".format(event.xdata, event.ydata)
            if self.startRadio.isChecked():
                self.startTxt.setPlainText(msg)
                self.update_point(0, event.xdata, event.ydata)
            else:
                self.goalTxt.setPlainText(msg)
                self.update_point(1, event.xdata, event.ydata)
                self.__goal = np.array([event.xdata, event.ydata])
    
    def demo_visibility_roadmap(self, start, goal):

        if start is None:
            return

        if self.__roadmap is None:
            self.__roadmap = getRoadmap(self.obstacle_list, self.boundary,0.5)

        rx, ry, found = self.__roadmap.plan(start, goal)

        if not found:
            return

        while len(self.ax.lines):
            for line in self.ax.lines:
                line.remove()
        self.pathPlot, = self.ax.plot([x for x in reversed(rx)], [y for y in reversed(ry)], 'r--')

        # self.ax.grid(True)
        # print("found path!!: ")
        self.canvas.draw_idle()
    def demo_rrt(self, start, goal):

        # config collision checker
        collisionManager = CollisionChecker(self.obstacle_list, self.boundary)
        # Set Initial parameters
        rrt_star = RRTStar(
            start=start,
            goal=goal,
            rand_area=[-7, 7],
            obstacle_list=self.obstacle_list,
            expand_dis=1,
            check_collision=collisionManager.check_collision,
            robot_radius=0.25)
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
            print("found path!!: ")
            self.canvas.draw_idle()



