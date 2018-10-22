GAS_DENSITY = 2.858

class VelocityController(object):

    def __init__(self, vehicle_mass, wheel_radius, accel_limit, decel_limit, brake_deadband,
                 fuel_capacity,max_acceleration):
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        self.brake_deadband = brake_deadband
        self.fuel_capacity = fuel_capacity
        self.max_acc_torque = (self.vehicle_mass + fuel_capacity * GAS_DENSITY) * \
                              max_acceleration * self.wheel_radius
        self.max_brake_torque = (self.vehicle_mass + fuel_capacity * GAS_DENSITY) * \
                                abs(self.decel_limit) * self.wheel_radius

    def control(self, target_velocity, current_velocity, realization_time):

        throttle, brake = 0., 0.

        error = target_velocity - current_velocity
        acceleration = error / realization_time
        acceleration = min(self.accel_limit, acceleration) if acceleration > 0 \
            else max(self.decel_limit, acceleration)

        if abs(acceleration) < self.brake_deadband: # no controls in deadband
            return throttle, brake

        torque = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * acceleration * self.wheel_radius

        throttle = min(1.0, torque / self.max_acc_torque) if torque > 0 else 0.0
        brake = 0.0 if torque > 0 else min(abs(torque), self.max_brake_torque)

        return throttle, brake
