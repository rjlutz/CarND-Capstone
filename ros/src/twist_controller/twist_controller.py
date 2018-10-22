from time import time
from yaw_controller import YawController
from velocity_controller import VelocityController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, wheel_radius,
                 decel_limit, accel_limit, wheel_base, steer_ratio, max_lat_accel,
                 max_steer_angle):

        min_speed = 1.0 * ONE_MPH

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.wheel_radius = wheel_radius
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle

        self.speed_controller = VelocityController(vehicle_mass,
            wheel_radius, accel_limit, decel_limit, brake_deadband,
            fuel_capacity, max_lat_accel)
        self.yaw_control = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel,
            max_steer_angle)
        self.prev_time = None
        self.max_speed = 0

    '''
       Calc throttle based on target_v, target_omega and current_v
    '''
    def control(self, target_v, target_omega, current_v, dbw_enabled):

        if self.prev_time is None or not dbw_enabled:
            self.prev_time = time()
            return 0.0, 0.0, 0.0

        if target_v > self.max_speed:
            self.max_speed = target_v

        dt = time() - self.prev_time

        throttle, brake = self.speed_controller.control(target_v,
            current_v, 0.5)

        error = min(target_v, self.max_speed) - current_v # max speed in MPH

        steer = self.yaw_control.get_steering(target_v, target_omega, current_v)
        self.prev_time = time()

        return throttle, brake, steer
