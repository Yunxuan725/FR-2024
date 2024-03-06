from enum import Enum

class CMD(Enum):
    PING = 0
    SEND_TWO_INTS = 1
    ECHO = 2
    GET_TIME_MILLIS = 3
    GET_TIME = 4
    SEND_TIME_Array = 5
    GET_TEMP_READINGS = 6
    CALC_DATA_RATE = 7