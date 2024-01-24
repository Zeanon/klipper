# Support for enable pins on stepper motor drivers
#
# Copyright (C) 2019-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

PIN_MIN_TIME = 0.100
MAX_ENABLE_TIME = 5.0
RESEND_HOST_TIME = 0.800 + PIN_MIN_TIME
DISABLE_STALL_TIME = 0.100

# Tracking of shared stepper enable pins
class StepperEnablePin:
    def __init__(self, mcu_enable,
                 enable_count,
                 printer,
                 max_on_time=0.):
        self.printer = printer
        self.reactor = self.printer.get_reactor()
        self.mcu_enable = mcu_enable
        self.enable_count = enable_count
        self.is_dedicated = True
        self.last_value = 0
        self.resend_timer = None
        self.resend_interval = (max_on_time - RESEND_HOST_TIME
                                if
                                max_on_time else 0.)
        self.last_print_time = 0.
    def set_enable(self, print_time):
        if self.mcu_enable is not None:
            if not self.enable_count:
                self._set_pin(print_time, 1)
            self.enable_count += 1
    def set_disable(self, print_time):
        if self.mcu_enable is not None:
            self.enable_count -= 1
            if not self.enable_count:
                self._set_pin(print_time, 0)
    def _set_pin(self, print_time, value, is_resend=False):
        if (value == self.last_value
                and not is_resend):
            return
        self.mcu_enable.set_digital(print_time, value)
        self.last_value = value
        self.last_print_time = print_time
        if self.resend_interval and self.resend_timer is None:
            self.resend_timer = self.reactor.register_timer(
                self._resend_current_val, self.reactor.NOW)

    def _resend_current_val(self, eventtime):
        if self.last_value == 0:
            self.reactor.unregister_timer(self.resend_timer)
            self.resend_timer = None
            return self.reactor.NEVER

        systime = self.reactor.monotonic()
        print_time = self.mcu_enable.get_mcu().estimated_print_time(systime)
        time_diff = (self.last_print_time + self.resend_interval) - print_time
        if time_diff > 0.:
            # Reschedule for resend time
            return systime + time_diff
        self._set_pin(print_time + PIN_MIN_TIME,
                      self.last_value, True)
        return systime + self.resend_interval


def setup_enable_pin(printer, pin, max_on_time=0.):
    if pin is None:
        # No enable line (stepper always enabled)
        enable = StepperEnablePin(None, 9999, printer)
        enable.is_dedicated = False
        return enable
    ppins = printer.lookup_object('pins')
    pin_params = ppins.lookup_pin(pin, can_invert=True,
                                  share_type='stepper_enable')
    enable = pin_params.get('class')
    if enable is not None:
        # Shared enable line
        enable.is_dedicated = False
        return enable
    mcu_enable = pin_params['chip'].setup_pin('digital_out', pin_params)
    mcu_enable.setup_max_duration(max_on_time)
    # mcu_enable.setup_start_value(0, 0)
    enable = pin_params['class'] = StepperEnablePin(mcu_enable,
                                                    0,
                                                    printer,
                                                    max_on_time)
    return enable

# Enable line tracking for each stepper motor
class EnableTracking:
    def __init__(self, stepper, enable):
        self.stepper = stepper
        self.enable = enable
        self.callbacks = []
        self.is_enabled = False
        self.stepper.add_active_callback(self.motor_enable)
    def register_state_callback(self, callback):
        self.callbacks.append(callback)
    def motor_enable(self, print_time):
        if not self.is_enabled:
            for cb in self.callbacks:
                cb(print_time, True)
            self.enable.set_enable(print_time)
            self.is_enabled = True
    def motor_disable(self, print_time):
        if self.is_enabled:
            # Enable stepper on future stepper movement
            for cb in self.callbacks:
                cb(print_time, False)
            self.enable.set_disable(print_time)
            self.is_enabled = False
            self.stepper.add_active_callback(self.motor_enable)
    def is_motor_enabled(self):
        return self.is_enabled
    def has_dedicated_enable(self):
        return self.enable.is_dedicated

# Global stepper enable line tracking
class PrinterStepperEnable:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.enable_lines = {}
        self.kinematics = config.getsection('printer').get('kinematics')
        self.printer.register_event_handler("gcode:request_restart",
                                            self._handle_request_restart)
        # Register M18/M84 commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("M18", self.cmd_M18)
        gcode.register_command("M84", self.cmd_M18)
        gcode.register_command("SET_STEPPER_ENABLE",
                               self.cmd_SET_STEPPER_ENABLE,
                               desc=self.cmd_SET_STEPPER_ENABLE_help)
    def register_stepper(self, config, mcu_stepper):
        name = mcu_stepper.get_name()
        max_on_time = config.getfloat('max_on_time', 0.,
                                      minval=0.500,
                                      maxval=MAX_ENABLE_TIME)
        disable_on_error = config.getboolean('disable_on_error', False)
        if disable_on_error:
            config.deprecate('disable_on_error')
            if not max_on_time:
                max_on_time = 5.
        enable = setup_enable_pin(self.printer,
                                  config.get('enable_pin', None),
                                  max_on_time)
        self.enable_lines[name] = EnableTracking(mcu_stepper, enable)
    def stepper_off(self, stepper_name, print_time, rail_name):
        el = self.enable_lines[stepper_name]
        el.motor_disable(print_time)
        if rail_name != 'extruder':
            self.printer.send_event("stepper_enable:disable_%s"
                                    % rail_name.lower(),
                                    print_time)
    def motor_off(self):
        self.axes_off()
    def axes_off(self, axes=None):
        if axes is None:
            axes = [0, 1, 2, 3]
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.dwell(DISABLE_STALL_TIME)
        print_time = toolhead.get_last_move_time()
        kin = toolhead.get_kinematics()
        if 3 in axes:
            if 'extruder' in self.enable_lines:
                self.stepper_off('extruder',
                                 print_time,
                                 'extruder')
                for i in range(1, 99):
                    extruder_name = "extruder" + str(i)
                    if extruder_name not in self.enable_lines:
                        break
                    self.stepper_off(extruder_name,
                                     print_time,
                                     'extruder')
        for axis in axes:
            try:
                rail = kin.get_rails()[axis]
                steppers = rail.get_steppers()
                rail_name = rail.mcu_stepper.get_name(True)
                for stepper in steppers:
                    self.stepper_off(stepper.get_name(),
                                     print_time,
                                     rail_name)
            except IndexError:
                continue
        self.printer.send_event(
            "stepper_enable:axes_off",
            print_time
        )
        toolhead.dwell(DISABLE_STALL_TIME)
    def motor_debug_enable(self, steppers, enable, notify=True):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.dwell(DISABLE_STALL_TIME)
        print_time = toolhead.get_last_move_time()
        kin = toolhead.get_kinematics()
        for stepper_name in steppers:
            el = self.enable_lines[stepper_name]
            if enable:
                el.motor_enable(print_time)
            else:
                el.motor_disable(print_time)
                if notify:
                    for rail in kin.get_rails():
                        for stepper in rail.get_steppers():
                            if stepper.get_name() == stepper_name:
                                self.printer.send_event(
                                    "stepper_enable:disable_%s" %
                                    rail
                                    .mcu_stepper
                                    .get_name(True)
                                    .lower(),
                                    print_time)
        logging.info("%s have been manually %s", steppers,
                     "enabled" if enable else "disabled")
        toolhead.dwell(DISABLE_STALL_TIME)
    def get_status(self, eventtime):
        steppers = { name: et.is_motor_enabled()
                           for (name, et) in self.enable_lines.items() }
        return {'steppers': steppers}
    def _handle_request_restart(self, print_time):
        self.motor_off()
    def cmd_M18(self, gcmd):
        axes = []
        for pos, axis in enumerate('XYZE'):
            if gcmd.get(axis, None) is not None:
                axes.append(pos)
        if not axes:
            axes = [0, 1, 2, 3]
        # Turn off motors
        self.axes_off(axes)
    cmd_SET_STEPPER_ENABLE_help = "Enable/disable individual stepper by name"
    def cmd_SET_STEPPER_ENABLE(self, gcmd):
        steppers_str = gcmd.get('STEPPERS', None)
        stepper_enable = gcmd.get_int('ENABLE', 1)
        notify = gcmd.get_int('NOTIFY', 1)
        if steppers_str is None:
            steppers = [None]
            old_stepper_str = gcmd.get('STEPPER', None)
            if old_stepper_str is not None:
                steppers = old_stepper_str.split(',')
                gcmd.respond_info('"STEPPER" parameter is deprecated')
        else:
            steppers = steppers_str.split(',')
        for stepper_name in steppers:
            if stepper_name not in self.enable_lines:
                gcmd.respond_info('SET_STEPPER_ENABLE: Invalid stepper "%s"'
                                  % stepper_name)
                return
        self.motor_debug_enable(steppers, stepper_enable, notify)
    def lookup_enable(self, name):
        if name not in self.enable_lines:
            raise self.printer.config_error("Unknown stepper '%s'" % (name,))
        return self.enable_lines[name]
    def get_steppers(self):
        return list(self.enable_lines.keys())

def load_config(config):
    return PrinterStepperEnable(config)
