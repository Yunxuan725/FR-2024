from enum import Enum

class CMD(Enum):
    ECHO = 0
    STOP = 1
    MOVE = 2
    SET_PID_VALUE = 3
    LINEAR_PID = 4