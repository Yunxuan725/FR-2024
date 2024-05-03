from enum import Enum

class CMD(Enum):
    ECHO = 0
    GET_IMU_DATA = 1
    GET_DISTANCE = 2
    STOP = 3
    MOVE = 4