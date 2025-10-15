from enum import Enum, auto

class SystemState(Enum):
    IDLE = auto()
    RUNNING = auto()
    STOPPED = auto()
    ERROR = auto()