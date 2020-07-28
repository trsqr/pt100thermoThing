# pt100thermoThing
ESP32 + MAX31865 + PT100 thermometer thing

This Arduino code can be used to make a PT100-based thermometer that reports its values over MQTT to a host or they can be queried over HTTP.

# Required libraries

* MQTT
* IotWebConf
* Adafruit_MAX38165
* pt100rtd

Others can be found from Arduino's library manager, pt100rtd is here: https://github.com/drhaney/pt100rtd

# Hardware

The idea is that you can add multiple (up to 4, but you could extend easily by modifying the code just a little bit) MAX31865 on the same project. Wire the SDI, SDO and CLK of all the MAXs to your HW SPI (on ESP32 that would be 18, 19 and 23). Then have individual GPIO for each CS. The CS selects which MAX31865 are we talking to.

The PT100 sensors can be in a 2-wire, 3-wire or 4-wire configuration. The MAX31865 might require soldering of a jumper and cutting of a bridge between two pins depending on your configuration, read the data sheets carefully.
