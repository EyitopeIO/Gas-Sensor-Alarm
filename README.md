# Gas-Sensor-Alarm
Final year project for a mining engineering student. He was trying to
demonstrate something I can no longer remember. I only helped him put
the circuit as it was quite a simple thing to do.

The calibration code was gotten from https://sandboxelectronics.com/?p=191

The potentiometer resistance on the MQ6 is 10k.


# Operation
The system starts by checking if a saved value exists in memory. If not, it
defaults to 1000. Then the gas sensor is warmed and calibration takes place.
It attempts to get its GPS location, quitting after 10 seconds. From here
the endless loop of checking and showing the gas concentration on the LCD,
checking the state of the buttons. If the gas concentration threshold is
exceeded, the alarm goes off and the system sends SMS to two phone numbers.
