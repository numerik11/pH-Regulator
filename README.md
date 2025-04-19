🌿 pH-Regulator
An Arduino-based pH regulation system using an I2C pH sensor, DHT22, and LCD interface. Designed for hydroponics, aquaponics, and water dosing automation with real-time feedback and calibration tools.

📦 Features
Accurate pH Measurement

I2C-based pH probe with temperature compensation

Live readings with smoothing buffer and voltage averaging

LCD Display with Centered Text

20x4 LCD shows pH, temperature, tank level, and status

Custom bar graph progress indicators during dosing and mixing

I2C Keypad Control

User-friendly menu for calibration, parameter entry, and adjustments

Soft restart support via keypad

Calibration Options

Manual slope/offset input

Guided auto-calibration using pH4 and pH7 buffers

Dosing Control

Auto-adjusted dosing time based on tank level

Mixing delay and tank enforcement logic included

Optional safety limits for extreme pH values

Tank Level Monitoring

Analog pressure transducer with voltage scaling

Smoothing filter and configurable tank height via EEPROM

EEPROM Persistence

Stores slope, offset, dose duration, tolerance, mix time, and tank height

Fail-safe Reset Mechanism

Keypad-driven resets ensure fast configuration changes

🔧 Hardware Requirements
ESP8266 / Arduino (Uno/Mega)

I2C pH Sensor (ADC)

DHT22 Temperature Sensor

20x4 I2C LCD Display

I2C Keypad

Analog Pressure Sensor (Tank Level)

Relay or Pump Driver

EEPROM-capable board (or use emulation)
