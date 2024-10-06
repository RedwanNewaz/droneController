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
        self._actionTrace = set()

    def updateAction(self, action:int):
        self.action = ActionType(action)
        self._actionTrace.add(action)
        nextstate = self.graph[self.state ][self.action]

        if nextstate == CalState.FINISHED:
            actions = [0, 2, 4] # NORTH, EAST, UP
            if any(map(lambda x: x not in self._actionTrace, actions)):
                return
            self.save()
            nextstate = CalState.INIT
        self.state = nextstate

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
    states = np.zeros((3, 3))
    for i, action in enumerate(['EAST', 'NORTH', 'UP']):
        traj = np.array(data[action])
        index = getSignedIndex(traj)
        states[i, index[1]] = index[0]

    states = np.array(states)
    actions = np.array(list(ActionDict.values()))
    rotationMatrix = actions @ states

    print(f'[Calibration]:{filename}\n'
          f'[states]: (x, y, z) \n {states}\n'
          f'[actions]: (vx, vy, vz) \n {actions}\n'
          f'[RotationMatrix]:\n {rotationMatrix}'
          )
