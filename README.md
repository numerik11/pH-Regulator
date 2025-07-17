🌿 pH-Regulator
----

An Arduino based pH regulation system using an I2C pH sensor, DHT22, and 20x4 LCD interface. Designed for hydroponics, and aquaponics with real-time feedback and calibration.


📦 Features
----

- Accurate pH Measurement

- Temperature compensation

- Live readings with smoothing buffer and voltage averaging

- 20x4 LCD shows pH, temperature, tank level, and status

- I2C Keypad Control

- User-friendly menu for calibration, parameter entry, and adjustments

- Soft restart support via keypad

- Calibration Options

- Manual slope/offset input

- Guided auto-calibration using pH4 and pH7 buffers

- Auto-adjusted dosing time based on tank level

- Mixing delay and tank enforcement logic included

- Optional safety limits for extreme pH values

- Tank Level Monitoring

- Analog pressure transducer with voltage scaling

- Smoothing filter and configurable tank height via EEPROM

- Stores slope, offset, dose duration, tolerance, mix time, and tank size

- Fail-safe Reset Mechanism

- Keypad-driven resets ensure fast configuration changes



🔧 Hardware Requirements

----

-Aduino Uno
- I2C pH Sensor (ADC) and quality pH Probe

- I2C Keypad 

- I2C LCD Display 20x4

- 12v AC/DC Adaptor
 
- Buck Convertor 

- DHT22 Temperature Sensor

- Analog Pressure Sensor (Optional)

- Parasidic pump

- 5v Relay module



![pH Reg](https://github.com/user-attachments/assets/ec10667a-b4d3-46e3-af85-576bc69abe67)

