from construct import Byte, Struct, Enum, Bytes, Const, Array, Int16ul,Float64b
Short = Int16ul
Double = Float64b

RobotData = "robot_data" / Struct(
    "xpos" / Double,
    "ypos" / Double,
    "wpos" / Double,
    "robot_status" / Enum(Byte, 
                            Forward=0,
                            Shift=1,
                            Rotate_CW=2,
                            Rotate_CCW=3,
                            Revo_CW=4,
                            Revo_CCW=5),
    "ball_status" / Enum(Byte, 
                            Not_Found=0,
                            Found=1),
    "is_moving" / Byte,
)


BallPosition = "ball_position" / Struct(
    "xpos" / Double,
    "ypos" / Double,
    "wpos" / Double
)

BaseStationData = "base_station_data" / Struct(
    "header" / Const(Bytes(4), b"UGMx"),
    "robot_id" / Enum(Byte,
                        ALFA=1,
                        ROBI=2,
                        ABI=3,
                        BASE=4),
    "robot_data" /  RobotData,
    "ball_pos" / BallPosition,
)

RobotName = {'ALFA':1, 'ROBI':2, 'ABI':3, 'BASE':4}
RobotStatus = {'Forward': 0, 'Shift':1, 'Rotate CW':2, 'Rotate CCW':3, 'Revo CW':4, 'Revo CCW':5}
BallStatus = {'Not Found': 0, 'Found':1}
