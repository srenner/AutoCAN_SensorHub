# AutoCAN_SensorHub
Automotive sensor hub using Arduino.

Development Hardware
-
* [Arduino-compatible AST-CAN485 Dev Board](https://www.sparkfun.com/products/14483)
* [VR Conditioner Board](http://jbperf.com/dual_VR/v2_1.html)
* (Maybe) DAC for analog output (MCP4725)

Inputs
-
* Fuel pressure (Honeywell MLH100PGB06A sensor)
* VSS (Ford 8k/mile sensor)
* CAN ECU data (from MegaSquirt)

Outputs
-
* Fuel pressure on CAN bus
* VSS on CAN bus
* Analog AFR signal (for traditional gauge)
* (Maybe) Analog fuel pressure signal (for traditional gauge)
