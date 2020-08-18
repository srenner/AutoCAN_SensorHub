# AutoCAN_SensorHub
Automotive sensor hub using Arduino.

Development Hardware
-
* [Arduino-compatible AST-CAN485 Dev Board](https://www.sparkfun.com/products/14483)
* DAC for analog output, probably a MCP4725

Inputs
-
* Fuel pressure (Honeywell MLH100PGB06A sensor)
* VSS (Ford 8k/mile sensor)
* CAN ECU data (from MegaSquirt)

Outputs
-
* Fuel pressure on CAN bus
* VSS on CAN bus
* Analog AFR signal
* (Maybe) Analog fuel pressure signal
