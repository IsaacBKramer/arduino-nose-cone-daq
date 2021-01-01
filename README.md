# arduino-nose-cone-daq

This sketch runs a data acquisition board for a launch vehicle nose cone. It handles data logging for a 6 axis accelerometer/gyroscope, a load cell, and two pressure transducers. 

The 6 axis IMU connects to the Arduino via I2C. It is used for launch detection and also collects in flight data. Registers are written to and read from via the Arduino Wire library. 

The load cell connects to the Arduino via an HX711 analog-to-digital converter (ADC) and amplifier. 

The two pressure transducers connect to the Arduino via a separate ADC board and I2C communication.

The SD card is fed raw data from all of the sensors via SPI communication. 