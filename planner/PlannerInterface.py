from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle
from .RRTStar import RRTStar
import yaml
class PlannerInterface:
    def __init__(self):
        self.fig = Figure()
        self.ax = self.fig.add_subplot()
        self.canvas = FigureCanvasQTAgg(self.fig)
        rect = Rectangle((-1.0, -1.0), 11, 11, facecolor='none', edgecolor='blue', linewidth=2)
        self.ax.add_patch(rect)
        self.ax.set_xlim(-1.0, 10.0)
        self.ax.set_ylim(-1.0, 10.0)
        self.demo('envs/env1.yaml', start=[0, 0], goal=[6, 10])

    def demo(self, env, start, goal):
        # ====Search Path with RRT====
        with open(env) as file:
            config = yaml.load(file, Loader=yaml.SafeLoader)
        obstacle_list = config['obstacle_list']
        print(obstacle_list)

        # Set Initial parameters
        rrt_star = RRTStar(
            start=start,
            goal=goal,
            rand_area=[-2, 15],
            obstacle_list=obstacle_list,
            expand_dis=1,
            robot_radius=0.4)
        path = rrt_star.planning(animation=False)

        if path is None:
            print("Cannot find path")
        else:
            self.ax.plot([x for (x, y) in path], [y for (x, y) in path], 'r--')
            self.ax.grid(True)
            print("found path!!: ", path)
