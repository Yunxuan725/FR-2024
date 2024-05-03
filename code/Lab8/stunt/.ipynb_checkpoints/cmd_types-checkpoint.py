from enum import Enum

class CMD(Enum):
    ECHO = 0
    MOVE = 1
    SET_YAW_PID = 2
    SET_LINEAR_PID = 3
    PID = 4