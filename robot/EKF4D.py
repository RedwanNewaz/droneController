import numpy as np

# Covariance for EKF
Q = np.diag([
    0.1,  # variance of location on x-axis
    0.1,  # variance of location on y-axis
    0.1,  # variance of location on z-axis
    0.0174533, # variance of rotation on z-axis
    0.08,  # variance of velocity on x-axis
    0.08,  # variance of velocity on y-axis
    0.08,  # variance of velocity on z-axis
    0.001  # variance of angular velocity on z-axis
]) ** 2  # predict state covariance
R = np.diag([0.01, 0.01, 0.01, 0.001]) ** 2  # Observation (x,y,z, yaw) covariance

class QuadrotorKinematics:
    def __init__(self, initial_position=[0.0, 0.0, 0.0], initial_orientation=0):
        self.position = np.array(initial_position)
        self.rotation_matrix = self.yaw_to_rotation_matrix(initial_orientation)

    @staticmethod
    def yaw_to_rotation_matrix(yaw):
        """
        Convert yaw angle to a 3x3 rotation matrix.
        Parameters:
        yaw (float): The yaw angle in radians.
        Returns:
        np.ndarray: A 3x3 rotation matrix.
        """

        cos_theta = np.cos(yaw)
        sin_theta = np.sin(yaw)
        rotation_matrix = np.array([
            [cos_theta, -sin_theta, 0],
            [sin_theta, cos_theta, 0],
            [0, 0, 1]
        ])
        return rotation_matrix

    def predict_motion(self, vx, vy, vz, wz, dt):

        # Update orientation (rotation matrix) based on angular velocity around z-axis
        omega_z_dt = wz * dt

        rotation_increment = self.yaw_to_rotation_matrix(omega_z_dt)
        self.rotation_matrix = np.dot(self.rotation_matrix, rotation_increment)

        # Update position based on linear velocities
        self.position = self.rotation_matrix @ self.position
        delta_p = np.array([vx * dt, vy * dt, vz * dt])
        self.position += delta_p

    def get_position(self):
        return np.squeeze(self.position)

    @staticmethod
    def rotation_matrix_to_euler_angles(R):
        """
        Converts a rotation matrix to Euler angles (ZYX convention).

        Parameters:
        R : ndarray
            3x3 rotation matrix

        Returns:
        tuple
            Euler angles (phi, theta, psi) in radians
        """
        assert R.shape == (3, 3), "R must be a 3x3 matrix"

        # Check for gimbal lock
        if R[2, 0] != 1 and R[2, 0] != -1:
            theta = -np.arcsin(R[2, 0])
            cos_theta = np.cos(theta)
            psi = np.arctan2(R[2, 1] / cos_theta, R[2, 2] / cos_theta)
            phi = np.arctan2(R[1, 0] / cos_theta, R[0, 0] / cos_theta)
        else:
            phi = 0
            if R[2, 0] == -1:
                theta = np.pi / 2
                psi = np.arctan2(R[0, 1], R[0, 2])
            else:
                theta = -np.pi / 2
                psi = np.arctan2(-R[0, 1], -R[0, 2])

        return phi, theta, psi

    def get_orientation(self):
        phi, theta, psi = self.rotation_matrix_to_euler_angles(self.rotation_matrix)
        return phi

class EKF4D:
    def __init__(self, xEst):
        self.xEst = xEst.copy()
        self.PEst = np.eye(8)

    @staticmethod
    def motion_model(x, u, DT):
        '''
        Compute the motion
        :param x: current state (x, y, z, theta, vx, vy, vz, w)
        :param u: (vx, vy, vz, w)
        :param DT: dt
        :return: predicted state (x, y, z, theta, vx, vy, vz, w)
        '''
        model = QuadrotorKinematics(x[:3, 0], x[3, 0])
        vel = x[4:, 0] + u
        model.predict_motion(vel[0], vel[1], vel[2], vel[3], DT)
        position = model.get_position()
        yaw = model.get_orientation()
        newstate = np.zeros((8, 1))
        newstate[:3, 0] = position
        newstate[3, 0] = yaw
        newstate[4:, 0] = vel
        return newstate

    @staticmethod
    def observation_model(x):
        H = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0]
        ], dtype=np.float32)
        z = H @ x

        return z
    @staticmethod
    def jacob_h():
        jH = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0]
        ], dtype=np.float32)
        return jH

    @staticmethod
    def jacob_f(x, u, DT):
        """
        Jacobian of Motion Model

        motion model
        """
        yaw = x[3, 0]
        R = QuadrotorKinematics.yaw_to_rotation_matrix(yaw)
        # compute velocity with respect to body frame
        v = R @ u[:3].reshape((3, 1)) # 3x1 matrix

        jF = np.array([
            [1.0, 0.0, 0.0, 0.0, v[0, 0] * DT, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, v[1, 0] * DT, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, v[2, 0] * DT, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, u[-1] * DT],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        ], dtype=np.float32)
        return jF

    def ekf_estimation(self, xEst, PEst, z, u, DT):
        #  Predict
        xPred = self.motion_model(xEst, u, DT)
        jF = self.jacob_f(xEst, u, DT)
        PPred = jF @ PEst @ jF.T + Q

        # print('predict', xPred.shape, jF.shape, PPred.shape)
        #  Update
        jH = self.jacob_h()
        zPred = self.observation_model(xPred)
        y = z - zPred
        S = jH @ PPred @ jH.T + R
        K = PPred @ jH.T @ np.linalg.inv(S)
        # print('Update', z.shape, y.shape, zPred.shape)

        xEst = xPred + K @ y
        PEst = (np.eye(len(xEst)) - K @ jH) @ PPred
        return xEst, PEst

    def update(self, u, z, dt):
        self.xEst, self.PEst = self.ekf_estimation(self.xEst, self.PEst, z, u, dt)
        # print(f"xEst: {self.xEst.shape}")