import logging


class ColdExtrude:
    def __init__(self, config):
        self.printer = config.get_printer()
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("M302", self.cmd_M302)
        gcode.register_command("COLD_EXTRUDE",
                               self.cmd_COLD_EXTRUDE,
                               desc=self.cmd_COLD_EXTRUDE_help)
    def cmd_M302(self, gcmd):
        self.set_cold_extrude(gcmd, 'T', 'P', 'S')
    cmd_COLD_EXTRUDE_help = "Control cold extrusions"
    def cmd_COLD_EXTRUDE(self, gcmd):
        self.set_cold_extrude(gcmd, 'EXTRUDER', 'ENABLE', 'MIN_EXTRUDE_TEMP')
    def set_cold_extrude(self,
                         gcmd,
                         index_name,
                         cold_extrude_name,
                         min_extrude_temp_name):
        index = gcmd.get_int(index_name, None, minval=0)
        if index is not None:
            section = 'extruder'
            if index:
                section = ('extruder%d' % index)
            extruder = self.printer.lookup_object(section, None)
            if extruder is None:
                raise gcmd.error("Extruder not configured")
        else:
            extruder = self.printer.lookup_object('toolhead').get_extruder()
        heater = extruder.get_heater()
        cold_extrude = gcmd.get_int(cold_extrude_name,
                                    None,
                                    minval=0,
                                    maxval=1)
        min_extrude_temp = gcmd.get_float(min_extrude_temp_name,
                                          None,
                                          minval=heater.min_temp,
                                          maxval=heater.max_temp)
        if cold_extrude is None and min_extrude_temp is None:
            gcmd.respond_info("Cold extrudes are %s (min temp %.2f\xb0C)"
                              % ("enabled" if heater.cold_extrude
                                 else "disabled",
                                 heater.min_extrude_temp))
            return
        heater.cold_extrude = True if cold_extrude else False
        logging.info("COLD_EXTRUDE set to [%s]" % cold_extrude)
        if min_extrude_temp is not None:
            heater.min_extrude_temp = min_extrude_temp
            heater.configfile.set(heater.name,
                                  'min_extrude_temp',
                                  heater.min_extrude_temp)
            gcmd.respond_info("min_extrude_temp has been set to %.2fC "
                              "for [%s] for the current session.\n"
                              "The SAVE_CONFIG command will update the "
                              "printer config file and restart the "
                              "printer."
                              % (heater.min_extrude_temp, heater.name))
            logging.info("COLD_EXTRUDE min_extrude_temp set to [%f]"
                         %
                         min_extrude_temp)
        heater.can_extrude = (heater.smoothed_temp >= heater.min_extrude_temp
                              or heater.cold_extrude)


def load_config(config):
    return ColdExtrude(config)
