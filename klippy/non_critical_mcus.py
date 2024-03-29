class NonCriticalMcus:
    def __init__(self, config):
        self.enabled = config.getboolean("enable_non_critical_mcus", False)


def add_printer_objects(config):
    config.get_printer().add_object('non_critical_mcus',
                                    NonCriticalMcus(
                                        config
                                        .getsection('non_critical_mcus')
                                    ))
