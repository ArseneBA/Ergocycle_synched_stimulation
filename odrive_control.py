import odrive
from odrive.enums import *


class Odrive:
    def __init__(self):
        print("Look for an odrive ...")
        self.odrv0 = odrive.find_any()
        print("Odrive found")

    def _config_encoder(self, mode, cpr,  bandwidth, calib_scan_distance=None, calib_range=None):
        self.odrv0.axis1.encoder.config.mode = mode  # Mode of the encoder
        self.odrv0.axis1.encoder.config.cpr = cpr  # Count Per Revolution
        self.odrv0.axis1.encoder.config.bandwidth = bandwidth
        if mode == EncoderMode.HALL:
            self.odrv0.axis1.encoder.config.calib_scan_distance = calib_scan_distance
        elif mode == EncoderMode.INCREMENTAL:
            self.odrv0.axis1.encoder.config.calib_range = calib_range

        self.odrv0.config.gpio9_mode = GpioMode.DIGITAL
        self.odrv0.config.gpio10_mode = GpioMode.DIGITAL
        self.odrv0.config.gpio11_mode = GpioMode.DIGITAL

    def _config_motor(self, pole_pairs):
        self.odrv0.axis1.motor.config.motor_type = MotorType.HIGH_CURRENT
        self.odrv0.axis1.motor.config.pole_pairs = pole_pairs

        self.odrv0.axis1.motor.config.calibration_current = 10
        self.odrv0.axis1.motor.config.resistance_calib_max_voltage = 20
        self.odrv0.axis1.motor.config.requested_current_range = 25  # Requires config save and reboot
        self.odrv0.axis1.motor.config.current_control_bandwidth = 100
        self.odrv0.axis1.motor.config.torque_constant = 0.21  # Not sure of this value

    def _config_brake_resistor(self):
        self.odrv0.config.enable_brake_resistor = True
        self.odrv0.config.brake_resistance = 3.3

    def _config_controller(self, vel_limit):
        self.odrv0.axis1.controller.config.pos_gain = 1  # For position control
        self.odrv0.axis1.controller.config.vel_gain = 0.02 * self.odrv0.axis1.motor.config.torque_constant * \
                                                      self.odrv0.axis1.encoder.config.cpr
        self.odrv0.axis1.controller.config.vel_integrator_gain = 0.1 * self.odrv0.axis1.motor.config.torque_constant * \
                                                                 self.odrv0.axis1.encoder.config.cpr
        self.odrv0.axis1.controller.config.vel_limit = vel_limit

    def confirm_configuration_calibration(self):
        self.odrv0.axis1.encoder.config.pre_calibrated = True
        self.odrv0.axis1.motor.config.pre_calibrated = True

    def _set_turn_s(self, turn_s):
        self.odrv0.axis1.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        # TODO: Once the mechanical system is robust remove the abs and the "-"
        self.odrv0.axis1.controller.input_vel = - abs(turn_s)
        self.odrv0.axis1.requested_state = AxisState.CLOSED_LOOP_CONTROL

    def print_values(self):
        print("shadow_count", self.odrv0.axis1.encoder.shadow_count)
        # print("pos_estimate", self.odrv0.axis1.encoder.pos_estimate)
        # print("pos_estimate_counts", self.odrv0.axis1.encoder.pos_estimate_counts)
        # print("pos_circular", self.odrv0.axis1.encoder.pos_circular)
        # print("shadow_count", self.odrv0.axis1.encoder.shadow_count)
        # print("count_in_cpr", self.odrv0.axis1.encoder.count_in_cpr)
        print("pos_cpr_counts", self.odrv0.axis1.encoder.pos_cpr_counts)
        # print("delta_pos_cpr_counts", self.odrv0.axis1.encoder.delta_pos_cpr_counts)
        print("vel_estimate", self.odrv0.axis1.encoder.vel_estimate, "\n")


class OdriveEncoderHall(Odrive):
    def __init__(self):
        Odrive.__init__(self)
        self._mode = EncoderMode.HALL
        self._cpr = 6 * 8
        self._calib_scan_distance = 150
        self._bandwidth = 100
        self._pole_pairs = 8
        self._vel_limit = 10
        self._shadow_count_init = self.odrv0.axis1.encoder.shadow_count
        self._angle_motor = 0
        self._angle_crank = 0

        self._old_shadow = 0

    def configuration(self):
        print("Configuration for HALL encoder")
        self._config_encoder(self._mode, self._cpr, self._bandwidth, calib_scan_distance=self._calib_scan_distance)
        self._config_motor(self._pole_pairs)
        self._config_brake_resistor()
        self._config_controller(self._vel_limit)
        print("Configuration done")

    def set_speed(self, speed: float):
        """
        Sets the motor to a given speed in turn/s
        """
        self._set_turn_s(speed)

    def get_angle_motor(self) -> float:
        """
        Gives the angles corresponding of the crank.
        We consider the initial position to be 0°.

        Returns
        -------
        _angle_motor : float
            Motor angle value (0-360)
        """
        # TODO:To reset this value we could use the Z value from the lm13.
        shadow_count = self.odrv0.axis1.encoder.shadow_count

        print("dif shadow", self._old_shadow - shadow_count)
        self._old_shadow = shadow_count

        self._angle_motor = (((self._shadow_count_init - shadow_count) / 48) * 360) % 360
        return self._angle_motor

    def get_angle_crank(self) -> float:
        """
        Gives the angle of the crank.
        For now, a calibration is needed since the initial angle is considered to be 0.

        Returns
        -------
        _angle_crank : float
            Crank angle value (0-360)
        """
        shadow_count = self.odrv0.axis1.encoder.shadow_count
        self._angle_crank = ((((self._shadow_count_init - shadow_count) / 48) * 360) / 41.8) % 360
        return self._angle_crank


class OdriveEncoderIncremental(Odrive):

    REDUCTION_POLE_PAIRS = 209
    REDUCTION_CPR = 5

    def __init__(self):
        Odrive.__init__(self)
        self._mode = EncoderMode.INCREMENTAL
        self._cpr = 24000 * self.REDUCTION_CPR  # Sensix documentation
        self.calib_range = 0.05  # Relax the sensibility for the encoder
        self._bandwidth = 3000  # We have a lot of point, so we need a big bandwidth
        self._pole_pairs = 8 * self.REDUCTION_POLE_PAIRS  # Reduction of 41.8:1, here we consider it to be 42:1
        self._vel_limit = 10 / 209

    def configuration(self):
        print("Configuration for Incremental encoder")
        self._config_encoder(self._mode, self._cpr, self._bandwidth, calib_range=self.calib_range)
        self._config_motor(self._pole_pairs)
        self._config_brake_resistor()
        self._config_controller(self._vel_limit)
        print("Configuration done")

    def set_speed(self, speed: float):
        """
        Sets the motor to a given speed in turn/s
        """
        self._set_turn_s(speed / self.REDUCTION_POLE_PAIRS)
