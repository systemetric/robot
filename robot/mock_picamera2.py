class Picamera2:
    """ Mock Picamera2 class """

    ERROR = None

    camera_properties = {"Model": "Mock Picamera2"}

    def __init__(self):
        pass

    def configure(self, _):
        pass

    def start(self):
        pass

    def stop(self):
        pass

    def close(self):
        pass

    def capture_array(self):
        return None

    def create_still_configuration(self, _):
        return {}

    @staticmethod
    def set_logging(_):
        pass

print("<<< WARN: using mock picamera2 >>>")
