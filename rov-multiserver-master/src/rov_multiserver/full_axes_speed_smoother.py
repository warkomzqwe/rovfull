import threading
from enum import Enum
from time import perf_counter, sleep
from typing import Callable, Union

from rov_logger.logger import get_rov_logger

logger = get_rov_logger()

# Custom types
NumericType = Union[int, float]
MovementsCallbackType = Callable[[NumericType, NumericType, NumericType,
                                  NumericType, NumericType, NumericType],
                                 None]


class MovementAxis(Enum):
    PITCH = 0
    ROLL = 1
    VERTICAL = 2
    YAW = 3
    FORWARD = 4
    LATERAL = 5


class SpeedSmootherLoop(threading.Thread):
    def __init__(self, movements_cb: MovementsCallbackType,
                 min_update_freq: NumericType, max_update_freq: NumericType,
                 accel_rate: NumericType, accel_threshold: NumericType,
                 accel_start_threshold: NumericType, decel_rate: NumericType,
                 decel_threshold: NumericType,
                 decel_start_threshold: NumericType):
        threading.Thread.__init__(self)
        self.__no_changes_counter = 0
        self.daemon = True
        self.name = 'SpeedSmootherLoop'
        self.__move = movements_cb
        self.__min_freq = min_update_freq
        self.__max_interval = 1/min_update_freq
        self.__max_freq = max_update_freq
        self.__min_interval = 1/max_update_freq
        self.__acceleration_rate = accel_rate
        self.__acceleration_threshold = accel_threshold
        self.__acceleration_start_threshold = accel_start_threshold
        self.__deceleration_rate = decel_rate
        self.__deceleration_threshold = decel_threshold
        self.__deceleration_start_threshold = decel_start_threshold
        self.__target_updated = threading.Event()
        self.__updating = threading.Lock()
        self.__axes = [MovementAxis.PITCH, MovementAxis.ROLL,
                       MovementAxis.VERTICAL, MovementAxis.YAW,
                       MovementAxis.FORWARD, MovementAxis.LATERAL]
        self.__current = {axis: 0.0 for axis in self.__axes}
        self.__target = {axis: 0.0 for axis in self.__axes}
        self.__last_t = None

    def __calculate_output(self, axis: MovementAxis, dt: float) -> float:
        target = self.__target[axis]
        current = self.__current[axis]
        accel_rate = self.acceleration_rate
        accel_threshold = self.acceleration_threshold
        accel_start_threshold = self.acceleration_start_threshold
        decel_rate = self.deceleration_rate
        decel_threshold = self.deceleration_threshold
        decel_start_threshold = self.deceleration_start_threshold
        if abs(target) > abs(current):  # Acceleration required
            state = "Acceleration required"
            if abs(current) >= accel_threshold:
                output = target
            elif abs(current) < accel_start_threshold:
                if abs(target) <= accel_start_threshold:
                    output = target
                else:
                    output = accel_start_threshold
                    if target < 0:
                        output *= -1.0
            else:  # Controlled acceleration
                if current != 0 and target != 0 and abs(current) + abs(target) > abs(current + target):
                    current *= -1
                step = accel_rate * dt
                if abs(target - current) < abs(step):
                    step = abs(target - current)
                if target < 0:
                    step *= -1.0
                output = current + step
        elif abs(target) < abs(current): # Deceleration required
            state = "Deceleration required"
            if abs(current) <= abs(decel_threshold):
                output = target
            elif abs(current) > decel_start_threshold:
                if abs(target) >= decel_start_threshold:
                    output = target
                else:
                    output = decel_start_threshold
                    if target < 0 or (target >= 0 > current):
                        output *= -1.0
            else:
                step = decel_rate * dt
                if (abs(current) - abs(step)) < abs(target):
                    step = abs(current) - abs(target)
                if current > 0:
                    step *= -1.0
                output = current + step
        elif target == current:
            state = "Target reached"
            output = current
        else:  # target != current and abs(target) == abs(current)
            state = "Opposite sign required"
            output = -current
        if current != target and False:
            logger.debug(f"{axis.name} axis - In: {target:.1f}% Out: "
                         f"{current:.1f}% -> {output:.1f}% ({state})")
        return output

    def __update_outputs(self) -> bool:
        t_now = perf_counter()
        dt = t_now - self.__last_t
        self.__last_t = t_now
        new_values = []
        changed = []
        for axis in self.__axes:
            with self.__updating:
                new_val = self.__calculate_output(axis, dt)
                new_values.append(new_val)
                changed.append(new_val != self.__current[axis])
                self.__current[axis] = new_val
        self.__move(*new_values)
        if True not in changed:
            self.__no_changes_counter += 1
        else:
            self.__no_changes_counter = 0
        return self.__no_changes_counter < 5 * self.max_freq

    def __update_target(self, axis: MovementAxis, new_value: NumericType,
                        positive: bool):
        if abs(new_value) != self.__current[axis]:
            logger.debug(f"Setting {axis.name} set point to:"
                         f" {'+' if positive else '-'}{new_value:.1f}%")
        with self.__updating:
            self.__target[axis] = new_value if positive else -new_value
            self.target_updated.set()

    def update_pitch(self, speed: NumericType, direction_positive: bool):
        self.__update_target(MovementAxis.PITCH, speed, direction_positive)

    def update_roll(self, speed: NumericType, direction_positive: bool):
        self.__update_target(MovementAxis.ROLL, speed, direction_positive)

    def update_vertical(self, speed: NumericType, direction_positive: bool):
        self.__update_target(MovementAxis.VERTICAL, speed, direction_positive)

    def update_yaw(self, speed: NumericType, direction_positive: bool):
        self.__update_target(MovementAxis.YAW, speed, direction_positive)

    def update_forward(self, speed: NumericType, direction_positive: bool):
        self.__update_target(MovementAxis.FORWARD, speed, direction_positive)

    def update_lateral(self, speed: NumericType, direction_positive: bool):
        self.__update_target(MovementAxis.LATERAL, speed, direction_positive)

    def run(self) -> None:
        self.__move(0, 0, 0, 0, 0, 0)
        self.__last_t = perf_counter()
        while True:
            self.target_updated.wait(self.max_interval)
            time_elapsed = perf_counter() - self.__last_t
            if time_elapsed < self.min_interval:
                sleep(self.min_interval - time_elapsed)
            if self.__update_outputs():  # Some channel changed
                sleep(self.min_interval)
                continue
            self.target_updated.clear()

    @property
    def min_freq(self):
        return self.__min_freq

    @property
    def max_freq(self):
        return self.__max_freq

    @property
    def min_interval(self):
        return self.__min_interval

    @property
    def max_interval(self):
        return self.__max_interval

    @property
    def acceleration_rate(self):
        return self.__acceleration_rate

    @acceleration_rate.setter
    def acceleration_rate(self, value):
        self.__acceleration_rate = value

    @property
    def acceleration_threshold(self):
        return self.__acceleration_threshold

    @acceleration_threshold.setter
    def acceleration_threshold(self, value):
        self.__acceleration_threshold = value

    @property
    def acceleration_start_threshold(self):
        return self.__acceleration_start_threshold

    @acceleration_start_threshold.setter
    def acceleration_start_threshold(self, value):
        self.__acceleration_start_threshold = value

    @property
    def deceleration_rate(self):
        return self.__deceleration_rate

    @deceleration_rate.setter
    def deceleration_rate(self, value):
        self.__deceleration_rate = value

    @property
    def deceleration_threshold(self):
        return self.__deceleration_threshold

    @deceleration_threshold.setter
    def deceleration_threshold(self, value):
        self.__deceleration_threshold = value

    @property
    def deceleration_start_threshold(self):
        return self.__deceleration_start_threshold

    @deceleration_start_threshold.setter
    def deceleration_start_threshold(self, value):
        self.__deceleration_start_threshold = value

    @property
    def target_updated(self):
        return self.__target_updated
