# Gas-Sensor-Alarm
1) Final year project for mining engineering student at FUTA

2) The calibration code was gotten from https://sandboxelectronics.com/?p=191

3) The potentiometer resistance is on the MQ6 is 10k.

The system starts by checking if a saved value exists in memory. If not, it defaults to 1000. Then the gas sensor is warmed and calibration takes place. It attempts to get its GPS location, quitting after 10 second.
From here the endless loop of checking and showing the gas concentration on the LCD, checking the state of the buttons. If the gas concentration threshold is exceeded, the alarm goes off and the system sends SMS to two phone numbers.
