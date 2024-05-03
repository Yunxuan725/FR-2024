from enum import Enum

class CMD(Enum):
    ECHO = 0
    MOVE = 1
    SET_YAW_PID = 2
    GET_ANGLE = 3
    SET_ANGLE = 4
    ORIENT_PID = 5