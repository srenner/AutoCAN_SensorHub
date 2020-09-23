# AutoCAN_SensorHub
Automotive sensor hub using Arduino. Simplifies wiring and communication when adding multiple aftermarket sensors to a vehicle. Outputs data to the CAN bus to be used by the ECU or other microcontrollers, and analog outputs to be used by traditional gauges.

Development Hardware
-
* [Arduino-compatible AST-CAN485 Dev Board](https://www.sparkfun.com/products/14483)
* [VR Conditioner Board](http://jbperf.com/dual_VR/v2_1.html) for handling VSS pulses
* DAC for analog output
* 3 axis accelerometer (I2C communication)
* GPS with compass (I2C communication)
* Spartan 2 wideband controller for O2 sensor status

Inputs
-
* Fuel pressure (Honeywell MLH100PGB06A sensor)
* VSS (Ford 8k pulse/mile sensor)
* CAN ECU data (from MegaSquirt)
* Triple axis accelerometer
* GPS
* Compass
* Oxygen sensor temperature status (signal that would normally drive a solid or blinking LED)

Outputs
-
* Fuel pressure on CAN bus
* VSS on CAN bus
* Trip odometer on CAN bus
* Analog AFR signal
* Accelerometer data on CAN bus
* Oxygen sensor temperature status on CAN bus
* GPS coordinates on CAN bus
* Compass heading on CAN bus
