"""
centauri, i have yet to write better code...
pid, used to be implemented but i removed it
since it did more harm than good. maybe i'll
add it back. but this code is good enough for
now. - piyaphat liamwilai :)
"""

import novapi
import time
import math
from mbuild.encoder_motor import encoder_motor_class
from mbuild import power_expand_board
from mbuild import gamepad
from mbuild.led_matrix import led_matrix_class
from mbuild.smart_camera import smart_camera_class
from mbuild.ranging_sensor import ranging_sensor_class
from mbuild.smartservo import smartservo_class
from mbuild import power_manage_module

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.setpoint = setpoint  # Desired value (target)
        self.integral = 0  # Sum of errors over time
        self.previous_error = 0  # Previous error (used for derivative)

    def update(self, current_value):
        # Calculate the error (setpoint - current value)
        error = self.setpoint - current_value
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error
        I = self.Ki * self.integral

        # Derivative term
        derivative = error - self.previous_error
        D = self.Kd * derivative

        # Calculate the output
        output = P + I + D

        # Save the current error for the next update
        self.previous_error = error

        return output

    def set_setpoint(self, setpoint):
        """ Update the target setpoint for the PID controller """
        self.setpoint = setpoint
        self.integral = 0  # Reset the integral to avoid wind-up
        self.previous_error = 0  # Reset previous error to avoid a large derivative spike

class utilities:
    def convert_to_true_speed(num):
        return num * 0.2 # rule of three :shrug:

    def convert_to_motor_speed(num):
        return num / 0.2 # rule of three :shrug:

class dc_motor:
    def __init__(self, motor_port):
        self.motor_port = motor_port
        
    def set_power(self, power):
        power_expand_board.set_power(self.motor_port, power)
        
    def enable(self, inverse=False):
        power_expand_board.set_power(self.motor_port, -100 if inverse else 100)
        
    def disable(self):
        power_expand_board.set_power(self.motor_port, 0)
        
led = led_matrix_class("PORT2", "INDEX1")
servo = smartservo_class("M3", "INDEX1")

class mecanum_drive:
    
    def __init__(self, left_front, left_back, right_front, right_back, deadzone=20):
        self.left_front = encoder_motor_class(left_front, "INDEX1")
        self.left_back = encoder_motor_class(left_back, "INDEX1")
        self.right_front = encoder_motor_class(right_front, "INDEX1")
        self.right_back = encoder_motor_class(right_back, "INDEX1")
        self.deadzone = deadzone
        self.vx_pid = PID(Kp=1, Ki=0, Kd=0, setpoint=0)
        self.vy_pid = PID(Kp=1, Ki=0, Kd=0.1, setpoint=0)
        
    def motor_drive(self, left_front_speed, left_back_speed, right_front_speed, right_back_speed):
        self.left_front.set_speed(left_front_speed * 0.95)
        self.left_back.set_speed(left_back_speed)
        self.right_front.set_speed(-right_front_speed * 0.9)
        self.right_back.set_speed(-right_back_speed * 0.9)
    
    def drive(self, velocity_x, velocity_y, angular_velocity):
        velocity_x *= 2.5
        velocity_y *= 2.5
        angular_velocity *= 2.5
        # Use deadzone to prevent motor from moving when joystick is not touched
        if math.fabs(velocity_x) < math.fabs(self.deadzone):
            velocity_x = 0
        if math.fabs(velocity_y) < math.fabs(self.deadzone):
            velocity_y = 0
        if math.fabs(angular_velocity) < math.fabs(self.deadzone):
            angular_velocity = 0
        # Calculation for the wheel speed
        # Visit https://github.com/neumann-lab/holonomic-mecanum/blob/main/th.md for the formula
        vFL = velocity_x + velocity_y - angular_velocity
        vFR = -(velocity_x) + velocity_y - angular_velocity
        vBL = -(velocity_x) + velocity_y + angular_velocity
        vBR = velocity_x + velocity_y + angular_velocity
        self.motor_drive(vFL, vFR, vBL, vBR)

    def move_forward(self, power):
        self.motor_drive(power, 0, 0)

    def move_backward(self, power):
        self.motor_drive(-power, 0, 0)

    def turn_left(self, power):
        self.motor_drive(0, 0, -power)

    def turn_right(self, power):
        self.motor_drive(0, 0, power)

    def slide_left(self, power):
        self.motor_drive(0, power, 0)

    def slide_right(self, power):
        self.motor_drive(0, -power, 0)

    def stop(self):
        self.motor_drive(0, 0, 0)

bottom_feed = dc_motor("DC8")
lift = encoder_motor_class("M5", "INDEX1")
up_feed = encoder_motor_class("M6", "INDEX1")
gripper = dc_motor("DC5")
conveyer = dc_motor("DC7")
left_ranging = ranging_sensor_class("PORT1", "INDEX1")
back_ranging = ranging_sensor_class("PORT2", "INDEX1")

class manual:
    
    def __init__(self, mode):
        self.mode = mode
        self.brushless = False
    
    def run(self):
        if gamepad.is_key_pressed("N1"):
            bottom_feed.set_power(-100)
            up_feed.set_power(-100)
        elif gamepad.is_key_pressed("N4"):
            bottom_feed.set_power(100)
            up_feed.set_power(100)

        if gamepad.is_key_pressed("N2"):
            conveyer.set_power(-100)
        elif gamepad.is_key_pressed("N3"):
            conveyer.set_power(100)

        if gamepad.is_key_pressed("L1"):
            bottom_feed.set_power(0)
            up_feed.set_power(0)
            conveyer.set_power(0)
            power_expand_board.set_power("BL2", 0)

        if gamepad.is_key_pressed("R1"):
            power_expand_board.set_power("BL2", 80)

        if gamepad.is_key_pressed("R2"):
            power_expand_board.set_power("BL2", 100)

        if gamepad.is_key_pressed("Up"):
            servo.move(3, 10)
        elif gamepad.is_key_pressed("Down"):
            servo.move(-3, 10)

        if gamepad.is_key_pressed("Left"):
            gripper.set_power(100)
        elif gamepad.is_key_pressed("Right"):
            gripper.set_power(-100)
        

class automatic:
    def __init__(self):
        pass

class robot:
    def __init__(self, movement: mecanum_drive, manual_stage: manual, automatic_stage: automatic):
        self.movement = movement
        self.manual = manual_stage
        self.automatic = automatic_stage
    
    def joystick_control(self):
        if math.fabs(gamepad.get_joystick("Lx")) > 20 or math.fabs(gamepad.get_joystick("Ly")) > 20 or math.fabs(gamepad.get_joystick("Rx")) > 20:
            self.movement.drive(-gamepad.get_joystick("Lx"), gamepad.get_joystick("Ly"), gamepad.get_joystick("Rx"))
        else:
            self.movement.motor_drive(0, 0, 0, 0)
        if math.fabs(gamepad.get_joystick("Ry")) > 50:
            lift.set_speed(gamepad.get_joystick("Ry"))
        else:
            lift.set_speed(0)

    def handle_manual_stage(self):
        self.joystick_control()
        self.manual.run()
    
    def handle_automatic_stage(self):
        self.movement.move_forward(30)
        time.sleep(1)
        self.movement.stop()
        time.sleep(1)
        self.movement.turn_right(30)
        time.sleep(0.9)
        self.movement.move_backward(50)
        time.sleep(1)
        self.movement.move_forward(50)
        bottom_feed.set_power(-100)
        up_feed.set_power(-100)
        time.sleep(3)
        self.movement.stop()
        bottom_feed.set_power(0)
        up_feed.set_power(0)

robot = robot(mecanum_drive("M1", "M3", "M2", "M4"), manual("shoot"), automatic())

while True:
    if power_manage_module.is_auto_mode():
        robot.handle_automatic_stage()
        while power_manage_module.is_auto_mode():
            pass
    else:
        robot.handle_manual_stage()
