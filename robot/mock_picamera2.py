class Picamera2:
    """ Mock Picamera2 class """

    ERROR = None

    camera_properties = {"Model": "Mock Picamera2"}

    def __init__(self):
        print("mock_picamera2.Picamera2.__init__()")

    def configure(self, _):
        print("mock_picamera2.Picamera2.configure(_)")

    def start(self):
        print("mock_picamera2.Picamera2.start()")
        pass

    def stop(self):
        print("mock_picamera2.Picamera2.stop()")
        pass

    def close(self):
        print("mock_picamera2.Picamera2.close()")
        pass

    def capture_array(self):
        print("mock_picamera2.Picamera2.capture_array(): None")
        return None

    def create_still_configuration(self, _):
        print("mock_picamera2.Picamera2.create_still_configuration(_): {}")
        return {}

    @staticmethod
    def set_logging(_):
        print("mock_picamera2.Picamera2.set_logging(_)")

print("<<< WARN: using mock picamera2 >>>")
