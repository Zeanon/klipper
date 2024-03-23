class NonCriticalMcus:
    def __init__(self):
        self.enabled = True

def add_printer_objects(config):
    config.get_printer().add_object('non_critical_mcus', NonCriticalMcus)
