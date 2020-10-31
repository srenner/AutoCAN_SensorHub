# AutoCAN_SensorHub
Automotive sensor hub using Arduino. Simplifies wiring and communication when adding multiple aftermarket sensors to a vehicle. Outputs data to the CAN bus to be used by the ECU or other microcontrollers, and analog outputs to be used by traditional gauges.

Development Hardware
-
* [Arduino-compatible AST-CAN485 Dev Board](https://www.sparkfun.com/products/14483)
* DC buck converter to supply all hardware with 5v power
* [VR Conditioner Board](http://jbperf.com/dual_VR/v2_1.html) for handling VSS pulses
* DAC for analog output (only outputting AFR at this point)
* 3 axis accelerometer with compass (I2C communication)
* GPS module (I2C communication)
* Temperature sensor using 1-wire communication protocol
* Optoisolators to take 12v inputs

Inputs
-
* Fuel pressure (Honeywell MLH100PGB06A sensor)
* VSS (Ford 8k pulse/mile sensor)
* CAN ECU data (from MegaSquirt)
* Triple axis accelerometer
* GPS
* Compass
* Reverse trigger wire from transmission
* Ambient temperature sensor

Outputs
-
* Fuel pressure on CAN bus
* VSS on CAN bus
* Trip odometer on CAN bus
* Reverse indicator on CAN bus
* Analog AFR signal
* Accelerometer data on CAN bus
* GPS coordinates, time, date on CAN bus
* Compass heading (degrees) and text representation ("NW", etc.) on CAN bus
* Ambient outdoor temperature
* Aux cabin heater relay signal
