class PwmOutput:
    def set_microseconds(self, us: int):
        raise RuntimeError(
            "Method not implemented. Maybe using an abstract class?")

    def get_microseconds(self) -> int:
        raise RuntimeError(
            "Method not implemented. Maybe using an abstract class?")

    @property
    def microseconds(self) -> int:
        return self.get_microseconds()

    @microseconds.setter
    def microseconds(self, value: int):
        self.set_microseconds(value)

    @property
    def min_pwm(self):
        return
