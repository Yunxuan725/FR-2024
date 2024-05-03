from enum import Enum

class CMD(Enum):
    ECHO = 0
    MOVE = 1
    SET_LINEAR_PID = 2
    GET_DISTANCE = 3
    SET_DISTANCE = 4
    LINEAR_PID = 5