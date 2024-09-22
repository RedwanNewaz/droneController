from enum import Enum
class State(Enum):
    INITIALIZED = 0
    ARMED = 1
    TAKEOFF = 2
    HOVER = 3
    LAND = 4

class Action(Enum):
    IS_ARMABLE = 0
    IS_ARMED = 1
    HAS_ARRIVED = 2
    DISARMED = 3
