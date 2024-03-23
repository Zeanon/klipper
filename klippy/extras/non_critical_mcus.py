class NonCriticalMcus:
    def __init__(self, config):
        self.enabled = True

def load_config(config):
    return NonCriticalMcus(config)