# Support for servos
#
# Copyright (C) 2017-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import ast
import logging

from klippy.extras.display import display

SERVO_SIGNAL_PERIOD = 0.020
PIN_MIN_TIME = 0.100
RENDER_TIME = 0.500

class PrinterServo:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.min_width = config.getfloat('minimum_pulse_width', .001,
                                         above=0., below=SERVO_SIGNAL_PERIOD)
        self.max_width = config.getfloat('maximum_pulse_width', .002,
                                         above=self.min_width,
                                         below=SERVO_SIGNAL_PERIOD)
        self.max_angle = config.getfloat('maximum_servo_angle', 180.)
        self.angle_to_width = (self.max_width - self.min_width) / self.max_angle
        self.width_to_value = 1. / SERVO_SIGNAL_PERIOD
        self.last_value = self.last_value_time = 0.
        initial_pwm = 0.
        iangle = config.getfloat('initial_angle', None, minval=0., maxval=360.)
        if iangle is not None:
            initial_pwm = self._get_pwm_from_angle(iangle)
        else:
            iwidth = config.getfloat('initial_pulse_width', 0.,
                                     minval=0., maxval=self.max_width)
            initial_pwm = self._get_pwm_from_pulse_width(iwidth)
        # Setup mcu_servo pin
        ppins = self.printer.lookup_object('pins')
        self.mcu_servo = ppins.setup_pin('pwm', config.get('pin'))
        self.mcu_servo.setup_max_duration(0.)
        self.mcu_servo.setup_cycle_time(SERVO_SIGNAL_PERIOD)
        self.mcu_servo.setup_start_value(initial_pwm, 0.)
        # Register commands
        servo_name = config.get_name().split()[1]
        gcode = self.printer.lookup_object('gcode')
        dtemplates = display.lookup_display_templates(config)
        self.templates = dtemplates.get_display_templates()
        self.active_template = None
        self.render_timer = None
        gcode_macro = self.printer.lookup_object("gcode_macro")
        self.create_template_context = gcode_macro.create_template_context
        gcode.register_mux_command("SET_SERVO", "SERVO", servo_name,
                                   self.cmd_SET_SERVO,
                                   desc=self.cmd_SET_SERVO_help)
        gcode.register_mux_command("SET_SERVO_TEMPLATE", "SERVO", servo_name,
                                   self.cmd_SET_SERVO_TEMPLATE,
                                   desc=self.cmd_SET_SERVO_TEMPLATE_help)
    def get_status(self, eventtime):
        return {'value': self.last_value}
    def _set_pwm(self, print_time, value):
        if value == self.last_value:
            return
        print_time = max(print_time, self.last_value_time + PIN_MIN_TIME)
        self.mcu_servo.set_pwm(print_time, value)
        self.last_value = value
        self.last_value_time = print_time
    def _get_pwm_from_angle(self, angle):
        angle = max(0., min(self.max_angle, angle))
        width = self.min_width + angle * self.angle_to_width
        return width * self.width_to_value
    def _get_pwm_from_pulse_width(self, width):
        if width:
            width = max(self.min_width, min(self.max_width, width))
        return width * self.width_to_value
    def _activate_timer(self):
        if self.render_timer is not None or self.active_template is None:
            return
        reactor = self.printer.get_reactor()
        self.render_timer = reactor.register_timer(self._render, reactor.NOW)
    def _activate_template(self, template, lparams):
        if template is not None:
            self.active_template = (template, lparams)
        else:
            self.active_template = None
    def _render(self, eventtime):
        if self.active_template is None:
            # Nothing to do - unregister timer
            reactor = self.printer.get_reactor()
            reactor.register_timer(self.render_timer)
            self.render_timer = None
            return reactor.NEVER
        # Setup gcode_macro template context
        context = self.create_template_context(eventtime)
        def render(name, **kwargs):
            return self.templates[name].render(context, **kwargs)
        context['render'] = render
        # Render all templates
        template = self.active_template[0]
        lparams = self.active_template[1]
        text = template.render(context, **lparams)
        parts = [t.strip() for t in text.split(':', 2)]
        context.clear()  # Remove circular references for better gc
        if parts[0].upper() == 'ANGLE':
            # Transmit pending changes
            angle = min(self.max_angle, float(parts[1].strip()))
            self._set_pwm(eventtime, self._get_pwm_from_angle(angle))
        elif parts[0].upper() == 'WIDTH':
            width = max(self.min_width, min(self.max_width,
                                            float(parts[1].strip())))
            self._set_pwm(eventtime, self._get_pwm_from_pulse_width(width))
        return eventtime + RENDER_TIME
    cmd_SET_SERVO_help = "Set servo angle"
    def cmd_SET_SERVO(self, gcmd):
        if self.active_template is not None:
            return
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        width = gcmd.get_float('WIDTH', None)
        if width is not None:
            self._set_pwm(print_time, self._get_pwm_from_pulse_width(width))
        else:
            angle = gcmd.get_float('ANGLE')
            self._set_pwm(print_time, self._get_pwm_from_angle(angle))
    cmd_SET_SERVO_TEMPLATE_help = "Assign a display_template to a SERVO"
    def cmd_SET_SERVO_TEMPLATE(self, gcmd):
        template = None
        lparams = {}
        tpl_name = gcmd.get("TEMPLATE")
        if tpl_name:
            template = self.templates.get(tpl_name)
            if template is None:
                raise gcmd.error("Unknown display_template '%s'" % (tpl_name,))
            tparams = template.get_params()
            for p, v in gcmd.get_command_parameters().items():
                if not p.startswith("PARAM_"):
                    continue
                p = p.lower()
                if p not in tparams:
                    raise gcmd.error("Invalid display_template parameter: %s"
                                     % (p,))
                try:
                    lparams[p] = ast.literal_eval(v)
                except ValueError as e:
                    raise gcmd.error("Unable to parse '%s' as a literal" % (v,))
        self._activate_template(template, lparams)
        self._activate_timer()

def load_config_prefix(config):
    return PrinterServo(config)
