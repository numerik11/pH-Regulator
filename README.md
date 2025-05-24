🌿 pH-Regulator


An ESP8266 based pH regulation system using an I2C pH sensor, DHT22, and 20x4 LCD interface. Designed for hydroponics, aquaponics, and water dosing automation with real-time feedback and calibration.


📦 Features


Accurate pH Measurement

temperature compensation

Live readings with smoothing buffer and voltage averaging

20x4 LCD shows pH, temperature, tank level, and status

I2C Keypad Control

User-friendly menu for calibration, parameter entry, and adjustments

Soft restart support via keypad

Calibration Options

Manual slope/offset input

Guided auto-calibration using pH4 and pH7 buffers

Auto-adjusted dosing time based on tank level

Mixing delay and tank enforcement logic included

Optional safety limits for extreme pH values

Tank Level Monitoring

Analog pressure transducer with voltage scaling

Smoothing filter and configurable tank height via EEPROM

Stores slope, offset, dose duration, tolerance, mix time, and tank size

Fail-safe Reset Mechanism

Keypad-driven resets ensure fast configuration changes



🔧 Hardware Requirements


ESP8266

I2C pH Sensor (ADC)

DHT22 Temperature Sensor

20x4 I2C LCD Display

I2C Keypad 

Analog Pressure Sensor (Optional)

Relay module
