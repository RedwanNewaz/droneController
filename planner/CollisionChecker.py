import fcl
import numpy as np

class CollisionChecker:
    def __init__(self, obstacle_list, bounday):
        self.manager = self.getCollisionManager(obstacle_list)
        self.boundary = bounday

    def is_inside_boundary(self, x, y):
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

    def check_collision(self, node, obstacleList, robot_radius):

        if node is None:
            return False

        for x, y in zip(node.path_x, node.path_y):

            if not self.is_inside_boundary(x, y):
                return False # treat like collision

            robot_position = np.array([x, y, 0])
            robot_sphere = fcl.Sphere(robot_radius)
            robot = fcl.CollisionObject(robot_sphere, fcl.Transform(robot_position))

            req = fcl.CollisionRequest()
            cdata = fcl.CollisionData()

            self.manager.collide(robot, cdata, fcl.defaultCollisionCallback)

            if cdata.result.is_collision:
                print('collision detected: ', x, y)
                return False # collision

        return True  # safe


