### Hardware components used to develop this system:
- CYMA Glock 18C (CM.030)
- ESP32 (bare chip microcontroller, WROOM variant)

- TP5100 (2S battery charger Mmodule)
- SX1308 (DC-DC Step Up Converter)
- 2S BMS 10A (Battery Management System with high Aperage troughput)
- CP2102 (USB-TTL Module for programming the esp32)
- USB C Connector (Female with solder pins, USB 2.0)
- Infrared Optocoupler (IR-Interruption detection Module)
- 2x IRL8721 (N-Channel Mosfet in TO-220 Package)
- 2x ICR18350 (Li-Ion Battery with Amperage)

It should also fit into other softair guns from the manufacturer cyma, as the internals are probably pretty much the same by mechanical design. Also feel free to use any other microcontroller instead of the ESP32 (as long as it fits into the housing, of course).
