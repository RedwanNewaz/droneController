import fcl
import numpy as np

class CollisionChecker:
    def __init__(self, obstacle_list, boundary, robot_radius):
        self.manager = self.getCollisionManager(obstacle_list)
        self.boundary = boundary
        self.robot_radius = robot_radius

    def is_inside_boundary(self, p):
        x, y = p
        xmin, xmax, ymin, ymax = self.boundary
        return xmin <= x <= xmax and ymin <= y <= ymax

    def getCollisionManager(self, obstacle_list):
        # Define the cube positions and size
        collisionObjects = []
        for obstacle in obstacle_list:
            cube_position = [obstacle[0], obstacle[1], 0]
            cube_size = obstacle[-1]
            box = fcl.Box(cube_size, cube_size, cube_size)
            obj = fcl.CollisionObject(box, fcl.Transform(np.array(cube_position)))
            collisionObjects.append(obj)
        manager = fcl.DynamicAABBTreeCollisionManager()
        manager.registerObjects(collisionObjects)
        return manager

    def getCollisionTraj(self, path, robot_size):
        collisionObjects = []
        for x, y in path:
            cube_position = [x, y, 0]
            sphere = fcl.Sphere(robot_size)
            obj = fcl.CollisionObject(sphere, fcl.Transform(np.array(cube_position)))
            collisionObjects.append(obj)
        manager = fcl.DynamicAABBTreeCollisionManager()
        manager.registerObjects(collisionObjects)
        return manager

    @staticmethod
    def generate_interpolated_points(p1, p2, V):
        p1 = np.array(p1)
        p2 = np.array(p2)
        dist = np.linalg.norm(p2 - p1)
        num_points = int(dist / V)
        # Generate evenly spaced values from 0 to 1
        t = np.linspace(0, 1, num_points)

        # Interpolate between p1 and p2
        interpolated_points = p1[np.newaxis, :] * (1 - t)[:, np.newaxis] + p2[np.newaxis, :] * t[:, np.newaxis]

        return interpolated_points

    def interpolate(self, path, V):
        traj = []
        for i, p in enumerate(path):
            if i == 0:
                continue

            points = self.generate_interpolated_points(path[i - 1], p, V)
            traj.extend(points)
        return np.array(traj)

    def is_edge_valid(self, target_node, node, obstacle):

        goal = np.array((target_node.x, target_node.y))
        start = np.array((node.x, node.y))
        path = self.interpolate(np.vstack((goal, start)), self.robot_radius / 2)
        print(path.shape)
        trajManager = self.getCollisionTraj(path, self.robot_radius / 2)

        cdata = fcl.CollisionData()

        self.manager.collide(trajManager, cdata, fcl.defaultCollisionCallback)
        if cdata.result.is_collision:
            return False # collision
        inside = map(self.is_inside_boundary, path)
        if not all(inside):
            return False # outside boundary

        return True  # safe


