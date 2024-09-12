# ESP32 Environment Data Recorder ("Tricorder")

Transmits environment data (pressure, temperature, humidity) over BLE using a BME280 sensor connected to and ESP32 microcontroller via I2C.

### Setup
1. Install and set up ESP-IDF extension for VSCode
2. Open an ESP-IDF terminal (can be found in extension menu)
3. Connect an ESP32 board to a USB port on your computer
3. Run `idf.py -p /dev/ttyUSB0 flash monitor` to build, run, and monitor the project on the board.If on Windows, replace `/dev/ttyUSB0` with the name of the COM port the ESP32 is connected to.
4. Use any BLE scanning app to read the pressure, temperature, and humidity characteristics.