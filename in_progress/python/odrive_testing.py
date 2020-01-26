import odrive
from odrive.enums import *
from odrive.utils import *
import time

my_drive = odrive.find_any()

print("starting calibration...")
my_drive.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
while my_drive.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
    
print('Configuring for sensorless...')
my_drive.axis0.controller.config.vel_gain = 0.01
my_drive.axis0.controller.config.vel_integrator_gain = 0.05
my_drive.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
my_drive.axis0.controller.vel_setpoint = 400
my_drive.axis0.motor.config.direction = 1
my_drive.axis0.sensorless_estimator.config.pm_flux_linkage = 5.51328895422 / (7 * 280)
my_drive.axis0.requested_state = AXIS_STATE_SENSORLESS_CONTROL

start_liveplotter(lambda:[ my_drive.axis0.sensorless_estimator.vel_estimate, my_drive.axis0.controller.vel_setpoint])

print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")
my_drive.axis0.motor.config.direction = 1
my_drive.axis0.controller.vel_setpoint = 400
print("Velocity setpoint is " + str(my_drive.axis0.controller.vel_setpoint))

time.sleep(2)
my_drive.axis0.motor.config.direction = -1
my_drive.axis0.controller.vel_setpoint = 800
print("Velocity setpoint is " + str(my_drive.axis0.controller.vel_setpoint))

time.sleep(2)
my_drive.axis0.motor.config.direction = 1
my_drive.axis0.controller.vel_setpoint = 2200
print("Velocity setpoint is " + str(my_drive.axis0.controller.vel_setpoint))

while True:
    pass
