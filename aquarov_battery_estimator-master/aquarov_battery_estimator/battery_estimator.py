from collections import Counter
from threading import Event

from pymessaginglib import set_broadcast_address
from rov_logger.logger import get_rov_logger

from aquarov_battery_estimator.battery_msg_listener import start_battery_msg_listener, _battery_info_event, _bus, _topic
from aquarov_battery_estimator.notification_service import LowBatteryNotificationService

# TODO add ip broadcast
# TODO correct centiamperes conversion

_TOP_VOLTAGE = 16.5
_BOTTOM_VOLTAGE = 13.5
_INTERNAL_R = 0.16
_CURRENT_THRESHOLD = 5
_BROADCAST_ADD = "192.168.0.255"
_exit = Event()
_notification_service = LowBatteryNotificationService()
_logger = get_rov_logger("DEBUG")

bat_estimation_array = []
len_estimation_array = 6
bat_estimation_after_mode_arr = []
len_after_mode_arr = 10

# initial values
for i in range(0, len_estimation_array):
    bat_estimation_array.append(100)

for i in range(0, len_after_mode_arr):
    bat_estimation_after_mode_arr.append(100)


@_bus.on(_battery_info_event)
def _report_battery_estimation(voltage_raw, current_raw):

    # input current informed as amperes
    current = abs(current_raw)

    # if current is too high we skip battery measurements
    if current > _CURRENT_THRESHOLD:
        return

    # adjusted voltage
    voltage = voltage_raw + current * 0.096

    # ocv = _calculate_open_circuit_voltage(voltage, current)

    # we normalize value between 0.0 and 1.0
    ocv_normalized = (voltage - _BOTTOM_VOLTAGE) / (_TOP_VOLTAGE -
                                                    _BOTTOM_VOLTAGE)

    # # Filter data
    battery_estimation = round(ocv_normalized*100.0)

    # Appending measurement to array and removing possible outliers
    # using mode
    bat_estimation_array.pop(0)
    bat_estimation_array.append(battery_estimation)

    data_counter = Counter(bat_estimation_array)
    bat_estimation_after_mode_arr.pop(0)
    bat_estimation_after_mode_arr.append(data_counter.most_common(1)[0][0])

    # More filtering... getting averages after mode
    battery_estimation_avg = sum(bat_estimation_after_mode_arr) / len_after_mode_arr
    battery_estimation_avg = round(battery_estimation_avg)

    dic = dict(battery_charge=battery_estimation_avg)

    _topic.send(dic)
    _logger.debug("Battery estimation: " + str(dic))

    _notification_service.update_measure(battery_estimation_avg)

# def _calculate_open_circuit_voltage(voltage, current):
#     ocv = _INTERNAL_R * current + voltage
#
#     if ocv > _TOP_VOLTAGE:
#         return _TOP_VOLTAGE
#
#     elif ocv < _BOTTOM_VOLTAGE:
#         return _BOTTOM_VOLTAGE
#
#     else:
#         return ocv


def main():

    set_broadcast_address(_BROADCAST_ADD)
    _logger.debug("Starting battery listener.")

    start_battery_msg_listener()
    try:
        _logger.debug("Waiting for incoming voltage/current measurements.")
        _exit.wait()

    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
