#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait

# Define motor and sensor objects
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
right_distance_sensor = UltrasonicSensor(Port.S2)
front_distance_sensor = UltrasonicSensor(Port.S1)

# Set the target distance from the wall in mm
target_distance = 50

# Set the PID constants
kp = 1.0  # Proportional gain
ki = 0.0  # Integral gain
kd = 0.0  # Derivative gain

# Initialize error and integral variables
error = 0
integral = 0

# Set the loop frequency in Hz
loop_frequency = 10
loop_time = 1 / loop_frequency

last_error = 0

# Loop forever
while True:
    # Get the distance readings from the sensors
    right_distance = right_distance_sensor.distance()
    front_distance = front_distance_sensor.distance()

    # Calculate the error
    error = target_distance - right_distance

    # Add the error to the integral
    integral += error * loop_time

    # Calculate the derivative
    derivative = (error - last_error) / loop_time

    # Calculate the PID output
    output = kp * error + ki * integral + kd * derivative

    # Set the motor speeds based on the PID output
    left_motor_speed = 50 - output
    right_motor_speed = 50 + output

    # Set the motor speeds
    left_motor.run(left_motor_speed)
    right_motor.run(right_motor_speed)

    # Remember the last error
    last_error = error

    # Wait for the loop time to elapse
    wait(loop_time)
