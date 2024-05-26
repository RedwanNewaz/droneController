import numpy as np

# Covariance for EKF simulation
Q = np.diag([
    0.1,  # variance of location on x-axis
    0.1,  # variance of location on y-axis
    np.deg2rad(1.0),  # variance of yaw angle
    0.08,  # variance of velocity on x-axis
    0.08  # variance of velocity on y-axis
]) ** 2  # predict state covariance
R = np.diag([0.01, 0.01]) ** 2  # Observation x,y position covariance

def motion_model(x, u, DT):
    F = np.array([[1.0, 0, 0, 0, 0],
                  [0, 1.0, 0, 0, 0],
                  [0, 0, 1.0, 0, 0],
                  [0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0]])

    B = np.array([[DT * x[3, 0], 0],
                  [DT * x[4, 0], 0],
                  [0.0, DT],
                  [1.0, 0.0],
                  [1.0, 0.0]
                  ])

    x = F @ x + B @ u

    return x


def observation_model(x):
    H = np.array([
        [1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0]
    ])
    z = H @ x

    return z


def jacob_f(x, u, DT):
    """
    Jacobian of Motion Model

    motion model
    x_{t+1} = x_t+v*s*dt*cos(yaw)
    y_{t+1} = y_t+v*s*dt*sin(yaw)
    yaw_{t+1} = yaw_t+omega*dt
    v_{t+1} = v{t}
    s_{t+1} = s{t}
    so
    dx/dyaw = -v*s*dt*sin(yaw)
    dx/dv = dt*s*cos(yaw)
    dx/ds = dt*v*cos(yaw)
    dy/dyaw = v*s*dt*cos(yaw)
    dy/dv = dt*s*sin(yaw)
    dy/ds = dt*v*sin(yaw)
    """
    yaw = x[2, 0]
    vx = u[0, 0]
    vy = u[1, 0]
    s = x[4, 0]
    jF = np.array([
        [1.0, 0.0, 0.0, DT * vx, 0.0],
        [0.0, 1.0, 0.0, 0.0, DT * vy],
        [0.0, 0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0]])
    return jF


def jacob_h():
    jH = np.array([[1, 0, 0, 0, 0],
                   [0, 1, 0, 0, 0]])
    return jH


def ekf_estimation(xEst, PEst, z, u, DT):
    #  Predict
    xPred = motion_model(xEst, u, DT)
    jF = jacob_f(xEst, u, DT)
    PPred = jF @ PEst @ jF.T + Q

    #  Update
    jH = jacob_h()
    zPred = observation_model(xPred)
    y = z - zPred
    S = jH @ PPred @ jH.T + R
    K = PPred @ jH.T @ np.linalg.inv(S)
    xEst = xPred + K @ y
    PEst = (np.eye(len(xEst)) - K @ jH) @ PPred
    return xEst, PEst