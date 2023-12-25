## Hourmeter daemon

This microservice aims to count the vehicle's active time for maintenance 
warning purposes.

### Installation

This microservice is installed along with rov-multiserver.

### Description

This microservice counts time periodically by a fixed interval (15 seconds 
by default). Then, every time it counts time it updates the value on a file 
for non-volatile behaviour ( *'~/.rov_multiserver/data/hour_meter.data'* by 
default) and publishes the time in a topic (*/vehicle/hourmeter/* by default).

The published messages will have the following format:

```json
{"secs":  36810.45}
```

In which: "secs" field is the elapsed time in seconds.

### Usage

```plain
Usage: rov-hourmeterd [OPTIONS]

  Initializes a daemon that counts the vehicle's active time.

Options:
  -i, --interval INTEGER       The minimum time interval in seconds to count
                               (defaults to 15)
  -f, --time-file TEXT         The file to store the time (defaults to '~/.
                               rov_multiserver/data/hour_meter.data')
  -t, --topic TEXT             The topic to publish hour meter data to
                               (defaults to '/vehicle/hourmeter/')
  -o, --output-interface TEXT  The network interface to publish to (defaults
                               to 'eth0')
  -v, --verbose                Increases verbosity. Set multiple times for
                               even more verbosity. From log level ERROR with
                               0 times to log level DEBUG with 3 times (-vvv)
                               [x>=0]
  --help                       Show this message and exit.
```

### Restarting time

In case the time is required to be restarted the software should be stopped 
and then the file should be deleted. The time will start from 0 in the next 
launch.