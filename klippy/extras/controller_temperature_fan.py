# Support fans that are enabled when temperature exceeds a set threshold
#
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import fan

KELVIN_TO_CELSIUS = -273.15
MAX_FAN_TIME = 5.0
AMBIENT_TEMP = 25.
PID_PARAM_BASE = 255.
PIN_MIN_TIME = 0.100

class ControllerTemperatureFan:
    def __init__(self, config):
        self.name = config.get_name().split()[1]
        self.printer = config.get_printer()
        self.fan = fan.Fan(config, default_shutdown_speed=1.)
        self.min_temp = config.getfloat('min_temp', minval=KELVIN_TO_CELSIUS)
        self.max_temp = config.getfloat('max_temp', above=self.min_temp)
        self.reverse = config.getboolean('reverse', False)
        pheaters = self.printer.load_object(config, 'heaters')
        self.sensor = pheaters.setup_sensor(config)
        self.sensor.setup_minmax(self.min_temp, self.max_temp)
        self.sensor.setup_callback(self.temperature_callback)
        pheaters.register_sensor(config, self)
        self.heaters = []
        self.heater_names = config.getlist("heater", None)
        self.stepper_names = config.getlist("stepper", None)
        self.stepper_enable = self.printer.load_object(config, 'stepper_enable')
        self.speed_delay = self.sensor.get_report_time_delta()

        self.fan_speed = config.getfloat('fan_speed', default=1.,
                                         minval=0., maxval=1.)
        self.idle_speed = config.getfloat('idle_speed', default=self.fan_speed,
                                          minval=0., maxval=1.)
        self.idle_timeout = config.getint("idle_timeout", default=30, minval=-1)

        self.max_speed_conf = config.getfloat(
            'max_speed', 1., above=0., maxval=1.)
        self.max_speed = self.max_speed_conf
        self.min_speed_conf = config.getfloat(
            'min_speed', 0.3, minval=0., maxval=1.)
        self.min_speed = self.min_speed_conf
        self.last_temp = 0.
        self.last_temp_time = 0.
        self.target_temp_conf = config.getfloat(
            'target_temp', 40. if self.max_temp > 40. else self.max_temp,
            minval=self.min_temp, maxval=self.max_temp)
        self.target_temp = self.target_temp_conf
        algos = {'watermark': ControlBangBang, 'pid': ControlPID}
        algo = config.getchoice('control', algos)
        self.control = algo(self, config)
        self.next_speed_time = 0.
        self.last_speed_value = 0.
        self.last_on = self.idle_timeout

        self.controller_speed = 0.
        gcode = self.printer.lookup_object('gcode')
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        gcode.register_mux_command(
            "SET_TEMPERATURE_FAN_TARGET", "TEMPERATURE_FAN", self.name,
            self.cmd_SET_TEMPERATURE_FAN_TARGET,
            desc=self.cmd_SET_TEMPERATURE_FAN_TARGET_help)

    def handle_connect(self):
        # Heater lookup
        pheaters = self.printer.lookup_object('heaters')
        if self.heater_names is None:
            self.heaters = [pheaters.lookup_heater(n) for n in
                            pheaters.available_heaters]
        else:
            self.heaters = [pheaters.lookup_heater(n) for n in
                            self.heater_names]
        # Stepper lookup
        all_steppers = self.stepper_enable.get_steppers()
        if self.stepper_names is None:
            self.stepper_names = all_steppers
            return
        if not all(x in all_steppers for x in self.stepper_names):
            raise self.printer.config_error(
                "One or more of these steppers are unknown: "
                "%s (valid steppers are: %s)"
                % (self.stepper_names, ", ".join(all_steppers)))
    def handle_ready(self):
        reactor = self.printer.get_reactor()
        reactor.register_timer(self.callback, reactor.monotonic()+PIN_MIN_TIME)
    def set_speed(self, read_time, value):
        if value <= 0.:
            value = 0.
        elif value < self.min_speed:
            value = self.min_speed
        if self.target_temp <= 0.:
            value = 0.
        if ((read_time < self.next_speed_time or not self.last_speed_value)
                and abs(value - self.last_speed_value) < 0.05):
            # No significant change in value - can suppress update
            return
        speed_time = read_time + self.speed_delay
        self.next_speed_time = speed_time + 0.75 * MAX_FAN_TIME
        self.last_speed_value = value
        self.fan.set_speed(speed_time, value)
    def callback(self, eventtime):
        speed = self.idle_speed
        active = False
        for name in self.stepper_names:
            active |= self.stepper_enable.lookup_enable(name).is_motor_enabled()
        for heater in self.heaters:
            _, target_temp = heater.get_temp(eventtime)
            if target_temp:
                active = True
        if active:
            self.last_on = 0
            speed = self.fan_speed
        elif self.idle_timeout != -1:
            if self.last_on >= self.idle_timeout:
                speed = 0.
            else:
                self.last_on += 1
        if speed != self.controller_speed:
            self.controller_speed = speed
        return eventtime + 1.
    def temperature_callback(self, read_time, temp):
        self.last_temp = temp
        self.control.temperature_callback(read_time,
                                          temp,
                                          self.controller_speed)
    def get_temp(self, eventtime):
        return self.last_temp, self.target_temp
    def get_min_speed(self):
        return self.min_speed
    def get_max_speed(self):
        return self.max_speed
    def get_status(self, eventtime):
        status = self.fan.get_status(eventtime)
        status["temperature"] = round(self.last_temp, 2)
        status["target"] = self.target_temp
        return status
    cmd_SET_TEMPERATURE_FAN_TARGET_help = (
        "Sets a temperature fan target and fan speed limits"
    )
    def cmd_SET_TEMPERATURE_FAN_TARGET(self, gcmd):
        temp = gcmd.get_float('TARGET', self.target_temp_conf)
        self.set_temp(temp)
        min_speed = gcmd.get_float('MIN_SPEED', self.min_speed)
        max_speed = gcmd.get_float('MAX_SPEED', self.max_speed)
        if min_speed > max_speed:
            raise self.printer.command_error(
                "Requested min speed (%.1f) is greater than max speed (%.1f)"
                % (min_speed, max_speed))
        self.set_min_speed(min_speed)
        self.set_max_speed(max_speed)

    def set_temp(self, degrees):
        if degrees and (degrees < self.min_temp or degrees > self.max_temp):
            raise self.printer.command_error(
                "Requested temperature (%.1f) out of range (%.1f:%.1f)"
                % (degrees, self.min_temp, self.max_temp))
        self.target_temp = degrees

    def set_min_speed(self, speed):
        if speed and (speed < 0. or speed > 1.):
            raise self.printer.command_error(
                "Requested min speed (%.1f) out of range (0.0 : 1.0)"
                % (speed))
        self.min_speed = speed

    def set_max_speed(self, speed):
        if speed and (speed < 0. or speed > 1.):
            raise self.printer.command_error(
                "Requested max speed (%.1f) out of range (0.0 : 1.0)"
                % (speed))
        self.max_speed = speed

######################################################################
# Bang-bang control algo
######################################################################

class ControlBangBang:
    def __init__(self, temperature_fan, config):
        self.temperature_fan = temperature_fan
        self.max_delta = config.getfloat('max_delta', 2.0, above=0.)
        self.heating = False
    def temperature_callback(self, read_time, temp, speed=0.):
        current_temp, target_temp = self.temperature_fan.get_temp(read_time)
        if (self.heating
            and temp >= target_temp+self.max_delta):
            self.heating = False
        elif (not self.heating
              and temp <= target_temp-self.max_delta):
            self.heating = True
        tempspeed = 0. if self.heating else self.temperature_fan.get_max_speed()
        finalspeed = max(speed, tempspeed)
        self.temperature_fan.set_speed(read_time, finalspeed)

######################################################################
# Proportional Integral Derivative (PID) control algo
######################################################################

PID_SETTLE_DELTA = 1.
PID_SETTLE_SLOPE = .1

class ControlPID:
    def __init__(self, temperature_fan, config):
        self.temperature_fan = temperature_fan
        self.Kp = config.getfloat('pid_Kp') / PID_PARAM_BASE
        self.Ki = config.getfloat('pid_Ki') / PID_PARAM_BASE
        self.Kd = config.getfloat('pid_Kd') / PID_PARAM_BASE
        self.min_deriv_time = config.getfloat('pid_deriv_time', 2., above=0.)
        self.temp_integ_max = 0.
        if self.Ki:
            self.temp_integ_max = self.temperature_fan.get_max_speed() / self.Ki
        self.prev_temp = AMBIENT_TEMP
        self.prev_temp_time = 0.
        self.prev_temp_deriv = 0.
        self.prev_temp_integ = 0.
    def temperature_callback(self, read_time, temp, speed=0.):
        current_temp, target_temp = self.temperature_fan.get_temp(read_time)
        time_diff = read_time - self.prev_temp_time
        # Calculate change of temperature
        temp_diff = temp - self.prev_temp
        if time_diff >= self.min_deriv_time:
            temp_deriv = temp_diff / time_diff
        else:
            temp_deriv = (self.prev_temp_deriv * (self.min_deriv_time-time_diff)
                          + temp_diff) / self.min_deriv_time
        # Calculate accumulated temperature "error"
        temp_err = target_temp - temp
        temp_integ = self.prev_temp_integ + temp_err * time_diff
        temp_integ = max(0., min(self.temp_integ_max, temp_integ))
        # Calculate output
        co = self.Kp*temp_err + self.Ki*temp_integ - self.Kd*temp_deriv
        bounded_co = max(0., min(self.temperature_fan.get_max_speed(), co))
        tempspeed = max(self.temperature_fan.get_min_speed(),
                        self.temperature_fan.get_max_speed() - bounded_co)
        finalspeed = max(speed, tempspeed)
        self.temperature_fan.set_speed(read_time, finalspeed)
        # Store state for next measurement
        self.prev_temp = temp
        self.prev_temp_time = read_time
        self.prev_temp_deriv = temp_deriv
        if co == bounded_co:
            self.prev_temp_integ = temp_integ

def load_config_prefix(config):
    return ControllerTemperatureFan(config)
