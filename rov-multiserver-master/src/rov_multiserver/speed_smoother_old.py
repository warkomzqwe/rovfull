import threading
from time import sleep

from rov_logger.logger import get_rov_logger

logger = get_rov_logger()


class SpeedSmoother(threading.Thread):
    def __init__(self, movement_func):
        super().__init__()
        self.current = 0.0
        self.target_updated = threading.Event()
        self.target = 0.0
        self.should_stop = False
        self.step_size = 5
        self.step_window = 0.04
        self.daemon = True
        self.name = "VerticalSmootherThread"
        self.movement_func = movement_func
        logger.info("Starting Speed Smoother")

    def iteration(self):
        self.target_updated.clear()
        logger.debug("New iteration, current: {}, target: {}".format(
            self.current, self.target))
        if self.target == self.current:
            logger.debug("Nothing to do")
            pass
        elif (self.current == 0 or
              (0 < self.current < self.target) or
              (0 > self.current > self.target)):
            # **"Accelerating" scenarios:**
            # 1. current == 0 (and target != current).
            # 2. current > 0 and target > 0.
            # 3. current < 0 and target < 0.
            logger.debug("No steps")
            self.current = self.target
            self.callback()
            sleep(self.step_window)
        else:
            # Controlled deceleration
            if ((self.current < 0 < self.target) or
                    (self.current > 0 > self.target)):
                target = 0
            else:
                target = self.target
            sign = 1 if target > self.current else -1
            dist = abs(self.current) - abs(target)
            if dist > self.step_size:
                self.current += (sign * self.step_size)
                logger.debug("stepping 5%")
                self.callback()
                sleep(self.step_window)
                return
            else:
                self.current += (sign * dist)
                logger.debug("stepping: {}%".format(self.current))
                self.callback()
                sleep(self.step_window)

    def callback(self):
        self.movement_func(abs(self.current), self.current >= 0)

    @property
    def target(self):
        return self._target

    @target.setter
    def target(self, new_target):
        if isinstance(new_target, (int, float)) and 100 >= new_target >= -100:
            self._target = new_target
            self.target_updated.set()

    def run(self):
        while not self.should_stop:
            self.iteration()
            if self.current == self.target:
                logger.debug("Will wait...")
                self.target_updated.wait()
                logger.debug("done waiting")

    def stop(self):
        self.should_stop = True
        self.target_updated.set()
