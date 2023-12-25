import csv
from threading import Thread
from time import sleep

from aquarov_battery_estimator.battery_estimator import main as battery_estimator_main
from aquarov_battery_estimator.battery_msg_listener import _topic
from pymessaginglib import set_broadcast_address

filename = "measurements3.csv"
set_broadcast_address("192.168.0.255")

time_between_measurements = 0.1  # seconds, real time is 1 second

timestamp_list = []
voltage_list = []
current_list = []
data = [timestamp_list, voltage_list, current_list]

with open(filename) as csv_file:
    csv_reader = csv.DictReader(csv_file, delimiter=',')
    try:
        for row in csv_reader:
            timestamp_list.append(round(float(row.get("timestamp"))))
            voltage_list.append(float(row.get("voltage"))*1000.0)
            current_list.append(float(row.get("current")))

    except csv.Error:
        pass


def run_estimator():
    battery_estimator_main()


t1 = Thread(name="battery_estimator_thread", target=run_estimator)
t1.daemon = True
t1.start()
count = 700

while 1:

    dic = dict(
        voltage=voltage_list[count],
        current=current_list[count] / 100.0  # current informed as centiamper
    )

    _topic.send(dic)

    count += 1
    sleep(time_between_measurements)

