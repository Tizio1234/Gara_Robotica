from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase

class RobotLabirinto():
    def __init__(self,
                 axle_track:int,
                 wheel_diameter:int,
                 left_motor_port:Port,
                 right_motor_port:Port,
                 right_sensor_port:Port,
                 front_sensor_port:Port,
                 gyro_sensor_port:Port,
                 right_default_distance:int,
                 front_default_distance:int,
                 left_motor_inverted:bool=False,
                 right_motor_inverted:bool=False,) -> None:
        self.left_motor = Motor(left_motor_port, positive_direction=Direction.COUNTERCLOCKWISE if left_motor_inverted else Direction.CLOCKWISE)
        self.right_motor = Motor(right_motor_port, positive_direction=Direction.COUNTERCLOCKWISE if right_motor_inverted else Direction.CLOCKWISE)
        self.drive_base = DriveBase(self.left_motor, self.right_motor, wheel_diameter, axle_track)
        self.right_sensor = UltrasonicSensor(right_sensor_port)
        self.front_sensor = UltrasonicSensor(front_sensor_port)
        self.right_distance = right_default_distance
        self.front_distance = front_default_distance
        self.gyro_sensor = GyroSensor(gyro_sensor_port)
    
    def drive(self, speed, turn_rate):
        self.drive_base.drive(speed, turn_rate)
    
    def rotate(self, angle):
        if angle == 0: return
        self.drive_base.drive(0, -90 if angle < 0 else 90)
        starting_angle = self.gyro_sensor.angle()
        while abs(self.gyro_sensor.angle() - starting_angle) <= abs(angle):
            pass
        self.drive_base.stop()
