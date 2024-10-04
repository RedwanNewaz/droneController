"""
Generates a quintic polynomial trajectory.
"""

import numpy as np
from math import cos, sin

def calculate_derivaties(waypoints):
  """
  Calculates velocities for a given set of waypoints.

  Args:
      waypoints (list): A list of waypoints, where each waypoint is a tuple (x, y).

  Returns:
      list: A list of velocities, where each velocity is a tuple (vx, vy).
  """

  num_points = len(waypoints)
  velocities = []
  for i in range(1, num_points):
    # Get current and previous waypoints
    current_point = waypoints[i]
    prev_point = waypoints[i-1]

    # Calculate relative distance (assuming Euclidean distance)
    dx = current_point[0] - prev_point[0]
    dy = current_point[1] - prev_point[1]
    dz = current_point[2] - prev_point[2]
    distance = np.sqrt(dx**2 + dy**2 + dz**2)
    if distance != 0.0:
        # Velocity is assumed to be constant between waypoints
        # (adjust this logic if you have additional information about speed)
        velocity = (dx / distance, dy / distance, dz / distance if distance > 0 else 0)
    else:
        velocity = (0.0, 0.0, 0.0)

    velocities.append(velocity)
  velocity = (0.0, 0.0, 0.0)
  velocities.append(velocity)

  return np.array(velocities)
class TrajectoryGenerator:
    def __init__(self, start_pos, des_pos, T, start_vel=[0,0,0], des_vel=[0,0,0], start_acc=[0,0,0], des_acc=[0,0,0]):
        self.start_x = start_pos[0]
        self.start_y = start_pos[1]
        self.start_z = start_pos[2]

        self.des_x = des_pos[0]
        self.des_y = des_pos[1]
        self.des_z = des_pos[2]

        self.start_x_vel = start_vel[0]
        self.start_y_vel = start_vel[1]
        self.start_z_vel = start_vel[2]

        self.des_x_vel = des_vel[0]
        self.des_y_vel = des_vel[1]
        self.des_z_vel = des_vel[2]

        self.start_x_acc = start_acc[0]
        self.start_y_acc = start_acc[1]
        self.start_z_acc = start_acc[2]

        self.des_x_acc = des_acc[0]
        self.des_y_acc = des_acc[1]
        self.des_z_acc = des_acc[2]

        self.T = T

    def solve(self):
        A = np.array(
            [[0, 0, 0, 0, 0, 1],
             [self.T**5, self.T**4, self.T**3, self.T**2, self.T, 1],
             [0, 0, 0, 0, 1, 0],
             [5*self.T**4, 4*self.T**3, 3*self.T**2, 2*self.T, 1, 0],
             [0, 0, 0, 2, 0, 0],
             [20*self.T**3, 12*self.T**2, 6*self.T, 2, 0, 0]
            ])

        b_x = np.array(
            [[self.start_x],
             [self.des_x],
             [self.start_x_vel],
             [self.des_x_vel],
             [self.start_x_acc],
             [self.des_x_acc]
            ])

        b_y = np.array(
            [[self.start_y],
             [self.des_y],
             [self.start_y_vel],
             [self.des_y_vel],
             [self.start_y_acc],
             [self.des_y_acc]
            ])

        b_z = np.array(
            [[self.start_z],
             [self.des_z],
             [self.start_z_vel],
             [self.des_z_vel],
             [self.start_z_acc],
             [self.des_z_acc]
            ])

        self.x_c = np.linalg.solve(A, b_x)
        self.y_c = np.linalg.solve(A, b_y)
        self.z_c = np.linalg.solve(A, b_z)


def calculate_position(c, t):
    """
    Calculates a position given a set of quintic coefficients and a time.

    Args
        c: List of coefficients generated by a quintic polynomial
            trajectory generator.
        t: Time at which to calculate the position

    Returns
        Position
    """
    return c[0] * t**5 + c[1] * t**4 + c[2] * t**3 + c[3] * t**2 + c[4] * t + c[5]


def calculate_velocity(c, t):
    """
    Calculates a velocity given a set of quintic coefficients and a time.

    Args
        c: List of coefficients generated by a quintic polynomial
            trajectory generator.
        t: Time at which to calculate the velocity

    Returns
        Velocity
    """
    return 5 * c[0] * t**4 + 4 * c[1] * t**3 + 3 * c[2] * t**2 + 2 * c[3] * t + c[4]


def calculate_acceleration(c, t):
    """
    Calculates an acceleration given a set of quintic coefficients and a time.

    Args
        c: List of coefficients generated by a quintic polynomial
            trajectory generator.
        t: Time at which to calculate the acceleration

    Returns
        Acceleration
    """
    return 20 * c[0] * t**3 + 12 * c[1] * t**2 + 6 * c[2] * t + 2 * c[3]


def rotation_matrix(roll_array, pitch_array, yaw):
    """
    Calculates the ZYX rotation matrix.

    Args
        Roll: Angular position about the x-axis in radians.
        Pitch: Angular position about the y-axis in radians.
        Yaw: Angular position about the z-axis in radians.

    Returns
        3x3 rotation matrix as NumPy array
    """
    roll = roll_array[0]
    pitch = pitch_array[0]
    return np.array(
        [[cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll), sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll)],
         [sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) *
          sin(roll), -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll)],
         [-sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(yaw)]
         ])

def generate_trajectory(path, dt):
    vel = calculate_derivaties(path)
    acc = calculate_derivaties(vel)
    N = len(path)
    result = []
    t = 0
    for i in range(1, N):
        T = 4
        traj = TrajectoryGenerator(path[i - 1], path[i], T, vel[i - 1], vel[i] * dt, acc[i - 1], 0.5 * acc[i] * (dt ** 2) )
        # traj = TrajectoryGenerator(path[i - 1], path[i], T, vel[i - 1], vel[i], acc[i - 1], acc[i])
        traj.solve()
        coeff = [traj.x_c, traj.y_c, traj.z_c]
        t = 0
        while t < T:
            rpos = list(map(lambda x: calculate_position(x, t)[0], coeff))
            result.append(rpos)
            dist = np.linalg.norm(path[i] - np.array(rpos))
            path[i - 1] = rpos
            vel[i - 1] = list(map(lambda x: calculate_velocity(x, t)[0], coeff))
            acc[i - 1] = list(map(lambda x: calculate_acceleration(x, t)[0], coeff))
            if dist < 0.345:
                break
            t += dt
    result.append(path[-1].tolist())

    return result