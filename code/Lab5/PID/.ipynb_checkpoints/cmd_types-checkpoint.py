from enum import Enum

class CMD(Enum):
    ECHO = 0
    SET_PID_VALUE = 1
    STOP = 2
    MOVE = 3
    LINEAR_PID = 4