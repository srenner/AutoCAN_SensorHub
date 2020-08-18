# AutoCAN_SensorHub
Automotive sensor hub using Arduino.

Development Hardware
-
* [Arduino-compatible AST-CAN485 Dev Board](https://www.sparkfun.com/products/14483)

Inputs
-
* Fuel pressure (Honeywell MLH100PGB06A sensor)
* VSS (Ford 8k/mile sensor)
* CAN ECU data (from MegaSquirt)
* (Maybe) Steering angle sensor

Outputs
-
* Fuel pressure on CAN bus
* VSS on CAN bus
* AFR analog voltage signal
* (Maybe) Steering angle on CAN bus
* (Maybe) Steering angle delta on CAN bus
