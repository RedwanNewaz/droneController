import pickle
from collections import defaultdict
from datetime import datetime
from enum import Enum

import numpy as np


class CalState(Enum):
    INIT = 0
    MOVE_NORTH = 1
    MOVE_SOUTH = 2
    MOVE_EAST = 3
    MOVE_WEST = 4
    MOVE_UP = 5
    MOVE_DOWN = 6
    FINISHED = 7
class ActionType(Enum):
    NORTH = 0
    SOUTH = 1
    EAST = 2
    WEST = 3
    UP = 4
    DOWN = 5

def getTransitionGraph():
    graph = defaultdict(dict)
    for s in range(8):
        for a in range(6):
            if a == (s - 1):
                graph[CalState(s)][ActionType(a)] = CalState(7)
            else:
                graph[CalState(s)][ActionType(a)] = CalState(a + 1)
    return graph
class Calibration:
    def __init__(self):
        self.action = None
        self.trace = defaultdict(list)
        self.state = CalState(0)
        self.graph = getTransitionGraph()

    def updateAction(self, action:int):
        self.action = ActionType(action)
        self.state  = self.graph[self.state ][self.action]
        if self.state == CalState.FINISHED:
            self.save()
            self.state = CalState.INIT
    def __call__(self, state):
        if self.action is not None:
            self.trace[self.action.name].append(state)

    def save(self):
        timestamp = datetime.now().timestamp()  # Get current timestamp
        filename = datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d_%H-%M-%S')
        with open(f'envs/{filename}.pkl', 'wb') as f:
            pickle.dump(self.trace, f)
        print("[+] calibration saved in envs folder as ", filename)

if __name__ == '__main__':
    filename = '../envs/sim-2024-10-05_11-57-48.pkl'
    with open(filename, 'rb') as f:
        data = pickle.load(f)

    def getSignedIndex(traj):
        gradient = traj - traj[0]
        abs_val = np.abs(gradient[-1])
        index = abs_val.argmax()
        sign = np.sign(gradient[-1][index])
        return [sign, index]

    ActionDict = {'EAST': [0, -1, 0], 'NORTH': [-1, 0, 0], 'UP': [0, 1, 0]}
    states = []
    for action in ['EAST', 'NORTH', 'UP']:
        traj = np.array(data[action])
        index = getSignedIndex(traj)
        states.append(index[0])

    states = np.array(states)
    actions = np.array(list(ActionDict.values()))
    states = np.diag(states)
    rotationMatrix = actions @ states
    print(f'[Calibration]:{filename}\n[RotationMatrix]:\n {rotationMatrix}')
