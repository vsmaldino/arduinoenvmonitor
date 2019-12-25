# arduinoenvmonitor
An environmental monitor based on Wemos D1

It is an environmental data monitor, based on a Wemos D1, powered by a photovoltaic panel via a backup battery, installed on the terrace, which provides the following data:
1) Brightness (TSL2561)
2) Temperature (DS18B20)
3) Temperature, pressure, humidity (BME280)
4) Battery voltage

The sensors are placed in a separate container, equipped with a transparent cover (for the brightness sensor) and a pair of push-pull fans which wash the internal air before reading.

All collected data are sent via WiFi to an MQTT broker (PubSubClient library) in order to be used in a flexible and decoupled way.
