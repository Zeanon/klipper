import logging

DISABLE_STALL_TIME = 0.100

class Unhome:
    def __init__(self, config):
        self.printer = config.get_printer()
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("D28", self.cmd_D28)
        gcode.register_command("MARK_AS_UNHOMED",
                               self.cmd_MARK_AS_UNHOMED,
                               desc=self.cmd_MARK_AS_UNHOMED_help)
    def unhome_axes(self, axes):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.dwell(DISABLE_STALL_TIME)
        print_time = toolhead.get_last_move_time()
        for axis in axes:
            self.printer.send_event("unhome:mark_as_unhomed_%s"
                                    % axis.lower(),
                                    print_time)
        if axes:
            logging.info("UNHOME axes: %s" % axes)
        toolhead.dwell(DISABLE_STALL_TIME)
    def cmd_D28(self, gcmd):
        axes = []
        for axis in ['X', 'Y', 'Z']:
            if gcmd.get(axis, None) is not None:
                axes.append(axis)
        if not axes:
            axes = ['X', 'Y', 'Z']
        self.unhome_axes(axes)
    cmd_MARK_AS_UNHOMED_help = "Manually set a specific axis as unhomed"
    def cmd_MARK_AS_UNHOMED(self, gcmd):
        axes_str = gcmd.get('AXES', None)
        if axes_str is None:
            axes = ['X', 'Y', 'Z']
        else:
            axes = axes_str.split(',')
        invalid_axis = []
        for axis in axes:
            if (axis.lower() != 'x'
                    and axis.lower() != 'y'
                    and axis.lower() != 'z'):
                invalid_axis.append(axis)
        if invalid_axis:
            gcmd.respond_info('UNHOME: Invalid axis %s'
                              % invalid_axis)
            return
        self.unhome_axes(axes)


def load_config(config):
    return Unhome(config)
