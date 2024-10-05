import fcl
import numpy as np

class CollisionChecker:
    def __init__(self, obstacle_list, bounday):
        self.manager = self.getCollisionManager(obstacle_list)
        self.boundary = bounday

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


    def check_collision(self, node, obstacleList, robot_radius):

        if node is None:
            return False

        path = list(zip(node.path_x, node.path_y))
        trajManager = self.getCollisionTraj(path, robot_radius)

        cdata = fcl.CollisionData()

        self.manager.collide(trajManager, cdata, fcl.defaultCollisionCallback)
        if cdata.result.is_collision:
            return False # collision
        inside = map(self.is_inside_boundary, path)
        if not all(inside):
            return False # outside boundary

        return True  # safe


