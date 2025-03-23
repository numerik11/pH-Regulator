#include <DHT22.h>
#include <Arduino.h>
#include <stdint.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad_I2C.h>
#include <Keypad.h>
#include <Wire.h>
#include <EEPROM.h>
#include "ADC.h"

// I2C Addresses
#define I2CADDR 0x26
#define ADC_I2C_ADDRESS 0x24
#define PIN_DATA 10

// Pin definitions
const int LED = 5;
const int ThermistorPin = A1;
const int PumpPin = A0;
#define SensorPin A4
const int tankSensorPin = A2;  // Tank level sensor

// Process and calibration variables stored in EEPROM addresses:
const int mixTimeAddress = 8;
const int ToleranceAddress = 12;
const int DoseDurationAddress = 16;
int MixTime;
int DoseDuration;
float pHTolerance;

// Sampling parameters
const int WINDOW_SIZE = 5;    // pH sensor circular buffer size
const int WINDOW_SIZE2 = 30;  // (Not used in this version)
unsigned long samplingTime = 5;

// I2C pH Module settings and conversion factor
static const float adc_vref = 2.5;
static const uint8_t i2c_address = ADC_I2C_ADDRESS;
static const uint16_t eoc_timeout = 300;
static const float adc_offset = 0;

// Calibration variables
float slope;
float offset;
float pHTarget;

// Temperature and humidity variables
int t; // from DHT22
int h;
float Tc; // Temperature (Celsius) computed from thermistor
float Tf; // Temperature in Fahrenheit (if needed)
float Tempoffset;  // Temperature compensation offset

// pH sensor reading arrays and index
float pHReadings[WINDOW_SIZE];    // Circular buffer for raw ADC voltage (mV)
float pHavg[10];                  // Smoothed pH readings from multiple averages
float avgValue;
float pHValue;
int INDEX = 0;  // Global index for pHReadings

// Additional variables for calibration readings
float pH4_reading;
float pH7_reading;

// Thermistor constants and variables
int Vo;
const float R1 = 10000.0;
float R2;
float logR2, Tcalc;
const float c1 = 0.0012400, c2 = 0.000209125, c3 = 0.0000002956720;

// Keypad and LCD configuration
LiquidCrystal_I2C lcd(0x27, 20, 4);
const byte ROWS = 4;
const byte COLS = 3;
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'.','0','#'}
};
byte rowPins[ROWS] = {5, 0, 1, 3};
byte colPins[COLS] = {4, 6, 2};
Keypad_I2C keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS, I2CADDR);
DHT22 dht22(PIN_DATA);

// Startup graphic for LCD
byte StartupScreen[] = {
  B00000,
  B00110,
  B01001,
  B10001,
  B10001,
  B01001,
  B00110,
  B00000
};

// Helper Function: Wait for specific key
void waitForKey(char target) {
  char key = NO_KEY;
  while (true) {
    key = keypad.getKey();
    if (key == target) {
      delay(200); // debounce delay
      break;
    }
  }
}

///////////////////////////
//   Setup & Main Loop   //
///////////////////////////
void setup() {
  lcd.init();
  lcd.backlight();

  // Display startup screen graphic
  lcd.setCursor(0, 0);
  lcd.print("pH Regulator");
  for (int i = 0; i < 24; i++) {
    lcd.scrollDisplayRight();
    delay(60);
  }

  Serial.begin(115200);

  // Set pin modes
  pinMode(PumpPin, OUTPUT);
  pinMode(SensorPin, INPUT);
  pinMode(LED, OUTPUT);

  Wire.begin();
  keypad.begin();

  // Load calibration and process parameters from EEPROM
  EEPROM.get(0, slope);
  EEPROM.get(4, offset);
  EEPROM.get(mixTimeAddress, MixTime);
  EEPROM.get(ToleranceAddress, pHTolerance);
  EEPROM.get(DoseDurationAddress, DoseDuration);

  // Prompt user for target pH (if not set)
  pH_set_point();
}

void loop() {
  // Update DHT22 readings
  //t = dht22.getTemperature();
  //h = dht22.getHumidity();

  delay(500);
  char key = keypad.getKey();
  if (key == '.') {
    digitalWrite(PumpPin, HIGH);
    digitalWrite(LED, HIGH);
    delay(5000);
  } else {
    digitalWrite(PumpPin, LOW);
    digitalWrite(LED, LOW);
  }
  if (key == '3') { // Restart command
    lcd.clear();
    lcd.print("Restarting...");
    delay(2000);
    asm volatile ("jmp 0");  // Using software reset for Arduino
  }
  if (key == '6') {
    setValues();  // Enter manual calibration/settings mode
  }

  // Read pH sensor and update display/dosing logic
  uint8_t ack = read_adc();
  if (ack) {
    Serial.println(F("***** I2C ERROR *****"));
  }
}

//////////////////////////////////////////////
//  pH Reading, Averaging and Temperature   //
//////////////////////////////////////////////
int8_t read_adc() {
  int32_t adc_code = 0;
  float ProbeVoltage = 0;
  uint8_t ack = 0;

  // Read ADC value from pH sensor module
  ack |= adc_read(i2c_address, &adc_code, eoc_timeout);
  ProbeVoltage = adc_code_to_voltage(adc_code, adc_vref) * 1000 + adc_offset;

  // Take multiple samples and compute a moving average using a circular buffer
  for (int a = 0; a < 10; a++) {
    pHReadings[INDEX] = ProbeVoltage;
    INDEX = (INDEX + 1) % WINDOW_SIZE;
    float sum = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
      sum += pHReadings[i];
    }
    pHavg[a] = sum / WINDOW_SIZE;
    digitalWrite(LED, LOW);
  }

  // Sort the 10 averaged samples to remove extreme values
  for (int a = 0; a < 9; a++) {
    for (int b = a + 1; b < 10; b++) {
      if (pHavg[a] > pHavg[b]) {
        float tempVal = pHavg[a];
        pHavg[a] = pHavg[b];
        pHavg[b] = tempVal;
      }
    }
  }

  // Average the middle 6 values for a stable reading
  avgValue = 0;
  for (int a = 2; a < 8; a++) {
    avgValue += pHavg[a];
  }

  // Convert voltage to pH value (adjust conversion factor as needed)
  const float conversionFactor = 5.0 / (-1024.0 * 6.0);
  pHValue = avgValue * conversionFactor;
  pHValue = (slope * 3.5 * pHValue + offset);

  // Read thermistor and compute temperature (in Celsius)
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  Tcalc = 1.0 / (c1 + c2 * logR2 + c3 * pow(logR2, 3));
  Tc = Tcalc - 273.15;
  Tf = (t * 9.0) / 5.0 + 32.0;
  delay(300);

  // Temperature compensation: adjust pH value
  if (t < 25) {
    Tempoffset = (25 - Tc) * 0.0010;
  } else if (t > 25) {
    Tempoffset = (t - 25) * 0.0015;
  }
  if (t > 25)
    pHValue += Tempoffset;
  else if (t < 25)
    pHValue -= Tempoffset;

  Serial.print("pH: ");
  Serial.println(pHValue, 2);
  Serial.print("Voltage: ");
  Serial.print(ProbeVoltage, 2);
  Serial.println(" mV");

  // Update LCD display and dosing logic
  display_and_dosing_logic(pHValue, Tc, Tempoffset);
  return ack;
}

//////////////////////////////////////////////
//  Display & Dosing Logic (with Tank Level)  //
//////////////////////////////////////////////
void display_and_dosing_logic(float pHValue, float Tc, float Tempoffset) {

    // Update LCD with current readings
  lcd.setCursor(4, 0);
  lcd.print("Cur pH:");
  lcd.setCursor(12, 0);
  lcd.print(pHValue, 2);

  lcd.setCursor(4, 1);
  lcd.print("Tgt pH:");
  lcd.setCursor(12, 1);
  lcd.print(pHTarget, 2);

  lcd.setCursor(4, 2);
  lcd.print("Temp:");
  lcd.setCursor(12, 2);
  lcd.print(Tc, 1);
  lcd.print((char)223); // Degree symbol
  lcd.print("C");

  // Read and display tank level
  float tankLevel = readTankLevel();
  lcd.setCursor(4, 3);
  lcd.print("Tank:");
  lcd.setCursor(12, 3);
  lcd.print(tankLevel, 1);
  lcd.print("%");

  // Do not dose if the pH is too far off target
  if (abs(pHValue - pHTarget) > 3.00) {
    digitalWrite(PumpPin, LOW);
    digitalWrite(LED, LOW);
    return;
  }

  // Dosing logic: if pH is above target+tolerance, dose pH down.
  // Abort dosing if tank level is below 30%
  if (pHValue > pHTarget + pHTolerance) {
    if (tankLevel < 30.0) {
      Serial.println("Tank level too low. Dosing aborted.");
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("Low Tank: No Dose");
      return;
    }
    
    float adjustedDoseDuration = DoseDuration;
    if (tankLevel > 70.0) {
      adjustedDoseDuration = DoseDuration * 1.5;
    }
    
    digitalWrite(PumpPin, HIGH);
    digitalWrite(LED, HIGH);
    lcd.clear();
    lcd.setCursor(2, 1);
    lcd.print("*Adding pH Down*");
    lcd.setCursor(2, 2);
    lcd.print("Tank: ");
    lcd.print(tankLevel, 1);
    lcd.print("%");
    lcd.setCursor(6, 3);
    lcd.print(adjustedDoseDuration);
    lcd.print(" sec");

    Serial.print("Dosing for ");
    Serial.print(adjustedDoseDuration);
    Serial.println(" seconds.");

    delay(adjustedDoseDuration * 1000);
    digitalWrite(PumpPin, LOW);
    delay(3000);
    digitalWrite(LED, LOW);
    lcd.clear();

 // Mixing period: update display during the mix without clearing the whole screen
 for (int i = MixTime * 60; i > 0; i--) {
  // Update current pH reading
  pHValue = get_pH_reading();

  // Update Mixing Time on line 3 (fixed format: MM:SS)
  lcd.setCursor(1, 3);
  lcd.print("Mixing Time: ");
  lcd.print(i / 60);
  lcd.print(":");
  if (i % 60 < 10) lcd.print("0");
  lcd.print(i % 60);

  // Update Current pH on line 0 (overwrite previous value)
  lcd.setCursor(2, 0);
  lcd.print("Current pH: ");
  // Clear previous numeric value (assumes up to 7 characters)
  lcd.print("       ");
  lcd.setCursor(14, 0);  // Adjust cursor to overwrite numeric portion
  lcd.print(pHValue, 2);

  // Update Target pH on line 1 (overwrite previous value)
  lcd.setCursor(2, 1);
  lcd.print("Target pH: ");
  lcd.print("      ");  // Clear previous numeric value
  lcd.setCursor(14, 1);
  lcd.print(pHTarget, 1);

  // Optional: Flash LED for visual effect
  digitalWrite(LED, LOW);
  delay(1000);
  digitalWrite(LED, HIGH);

  // Check for manual dosing or restart commands during mix
  char key = keypad.getKey();
  if (key == '.') {
    digitalWrite(PumpPin, HIGH);
    digitalWrite(LED, HIGH);
    delay(4500);
    digitalWrite(PumpPin, LOW);
    digitalWrite(LED, LOW);
  } else {
    digitalWrite(PumpPin, LOW);
    digitalWrite(LED, LOW);
  }
  if (key == '3') {
    lcd.clear();
    lcd.print("Restarting...");
    delay(2000);
    asm volatile ("jmp 0");
   }  
  }
  lcd.clear();
 }
}


float get_pH_reading() {
  int32_t adc_code = 0;
  float ProbeVoltage = 0;
  uint8_t ack = 0;

  // Read ADC value from pH sensor module
  ack |= adc_read(i2c_address, &adc_code, eoc_timeout);
  ProbeVoltage = adc_code_to_voltage(adc_code, adc_vref) * 1000 + adc_offset;

  // Take multiple samples and compute a moving average using a circular buffer
  for (int a = 0; a < 10; a++) {
    pHReadings[INDEX] = ProbeVoltage;
    INDEX = (INDEX + 1) % WINDOW_SIZE;
    float sum = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
      sum += pHReadings[i];
    }
    pHavg[a] = sum / WINDOW_SIZE;
    digitalWrite(LED, LOW);
  }

  // Sort the 10 averaged samples to remove extreme values
  for (int a = 0; a < 9; a++) {
    for (int b = a + 1; b < 10; b++) {
      if (pHavg[a] > pHavg[b]) {
        float tempVal = pHavg[a];
        pHavg[a] = pHavg[b];
        pHavg[b] = tempVal;
      }
    }
  }

  // Average the middle 6 values for a stable reading
  float avgVal = 0;
  for (int a = 2; a < 8; a++) {
    avgVal += pHavg[a];
  }

  // Convert voltage to pH value (adjust conversion factor as needed)
  const float conversionFactor = 5.0 / (-1024.0 * 6.0);
  float currentPH = avgVal * conversionFactor;
  currentPH = (slope * 3.5 * currentPH + offset);

  // Read thermistor and compute temperature (in Celsius)
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  Tcalc = 1.0 / (c1 + c2 * logR2 + c3 * pow(logR2, 3));
  Tc = Tcalc - 273.15;
  Tf = (t * 9.0) / 5.0 + 32.0;
  delay(300);

  // Temperature compensation: adjust pH value
  if (t < 25) {
    Tempoffset = (25 - Tc) * 0.0010;
  } else if (t > 25) {
    Tempoffset = (t - 25) * 0.0015;
  }
  if (t > 25)
    currentPH += Tempoffset;
  else if (t < 25)
    currentPH -= Tempoffset;

  return currentPH;
}


//////////////////////////////////////////////
//        Tank Level Reading Function       //
//////////////////////////////////////////////
float readTankLevel() {
  int sensorValue = analogRead(tankSensorPin);
  float levelPercent = (sensorValue / 1023.0) * 100.0;
  Serial.print("Tank Level: ");
  Serial.print(levelPercent, 1);
  Serial.println("%");
  return levelPercent;
}

//////////////////////////////////////////////
//           User Input Menu                //
//////////////////////////////////////////////
void pH_set_point() {
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Enter PH Target.");
  lcd.setCursor(3, 3);
  lcd.print("Then Press '#'");
  Serial.println("Enter pH Target and '#'");

  String pH_string = "";
  char key;
  while (true) {
    if ((key = keypad.getKey()) != NO_KEY) {
      if (key == '#') break;
      pH_string += key;
      lcd.clear();
      lcd.setCursor(5, 0);
      lcd.print("Target pH:");
      lcd.setCursor(9, 1);
      lcd.print(pH_string);
      delay(100);
    }
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == '#' || c == '\n') break;
      pH_string += c;
      lcd.setCursor(0, 1);
      lcd.print("                ");
      lcd.setCursor(0, 1);
      lcd.print(pH_string);
      delay(1500);
    }
  }
  pHTarget = pH_string.toFloat();
  delay(1500);

  // Confirmation screen and options
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("pH Target -- ");
  lcd.print(pHTarget);
  lcd.setCursor(0, 1);
  lcd.print("1.Start Regulator");
  lcd.setCursor(0, 2);
  lcd.print("2.Settings");
  lcd.setCursor(0, 3);
  lcd.print("3.Re-Enter Target");
  delay(500);

  while (true) {
    key = keypad.getKey();
    if (key == '2') {
      lcd.clear();
      lcd.setCursor(3, 0);
      lcd.print("Calibrating...");
      calibrate();
      break;
    } else if (key == '1') {
      lcd.clear();
      lcd.setCursor(4, 0);
      lcd.print("Regulator...");
      float stored_slope, stored_offset;
      EEPROM.get(0, stored_slope);
      EEPROM.get(4, stored_offset);
      EEPROM.get(mixTimeAddress, MixTime);
      EEPROM.get(ToleranceAddress, pHTolerance);
      EEPROM.get(DoseDurationAddress, DoseDuration);
      slope = stored_slope;
      offset = stored_offset;

      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print("Saved Slope: ");
      lcd.print(slope, 2);
      lcd.setCursor(1, 1);
      lcd.print("Saved Offset: ");
      lcd.print(offset, 2);
      lcd.setCursor(1, 2);
      lcd.print("Mix Delay: ");
      lcd.print(MixTime);
      lcd.setCursor(1, 3);
      lcd.print("Tolerance: ");
      lcd.print(pHTolerance, 2);
      delay(3000);
      lcd.clear();
      return; // Return to main loop
    } else if (key == '3') {
      lcd.clear();
      lcd.setCursor(5, 0);
      lcd.print("Restart...");
      delay(1000);
      pH_set_point(); // Re-run target entry
      lcd.clear();
      return;
    }
  }
}

//////////////////////////////////////////////
//         Calibration Functions            //
//////////////////////////////////////////////
// Manual calibration: user enters new slope and offset
void setValues() {
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Slope: ");
  lcd.print(slope, 2);
  lcd.setCursor(0, 2);
  lcd.print("Offset: ");
  lcd.print(offset, 2);
  delay(2000);
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Enter Slope:");
  lcd.setCursor(0, 3);
  lcd.print("Current: ");
  lcd.print(slope, 2);

  char input[8] = {0};
  int i = 0, column = 15;
  char key;
  while (true) {
    key = keypad.getKey();
    if (key == '#') {
      input[i] = '\0';
      break;
    }
    if (key != NO_KEY) {
      lcd.setCursor(8, 1);
      input[i++] = key;
      lcd.setCursor(column, 0);
      lcd.print(key);
      column++;
      delay(200);
    }
  }
  slope = atof(input);

  lcd.clear();
  lcd.setCursor(0, 3);
  lcd.print("Enter Offset:");
  i = 0;
  column = 15;
  while (true) {
    key = keypad.getKey();
    if (key == '#') {
      input[i] = '\0';
      break;
    }
    if (key != NO_KEY) {
      lcd.setCursor(8, 1);
      input[i++] = key;
      lcd.setCursor(column, 0);
      lcd.print(key);
      column++;
      delay(200);
    }
  }
  offset = atof(input);

  EEPROM.put(0, slope);
  EEPROM.put(4, offset);
  EEPROM.get(0, slope);
  EEPROM.get(4, offset);

  lcd.clear();
  lcd.setCursor(3, 1);
  lcd.print("Slope: ");
  lcd.print(slope, 2);
  lcd.setCursor(3, 2);
  lcd.print("Offset: ");
  lcd.print(offset, 2);
  delay(2000);
  lcd.clear();
  lcd.print("Values saved.");
  delay(2000);
  asm volatile ("jmp 0");  // Software reset for Arduino
}

// Adjust dose duration (user input)
void adjustDoseDuration() {
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Dose Duration:");
  String inputDuration = "";
  char key;
  while (true) {
    key = keypad.getKey();
    if (key != NO_KEY) {
      if (key >= '0' && key <= '9') {
        inputDuration += key;
        lcd.setCursor(inputDuration.length() + 9, 2);
        lcd.print(key);
        delay(200);
      } else if (key == '#') {
        break;
      }
    }
  }
  DoseDuration = inputDuration.toInt();
  EEPROM.put(DoseDurationAddress, DoseDuration);
  EEPROM.get(DoseDurationAddress, DoseDuration);
  lcd.clear();
  lcd.print("Dose Duration = ");
  lcd.print(DoseDuration);
  lcd.print(" sec");
  delay(2000);
  return;
}

// Change pH tolerance (user input)
void changePHTolerance() {
  lcd.clear();
  lcd.print("Enter pH Tolerance:");
  String inputTolerance = "";
  char key;
  while (true) {
    key = keypad.getKey();
    if (key != NO_KEY) {
      if ((key >= '0' && key <= '9') || key == '.') {
        inputTolerance += key;
        lcd.setCursor(inputTolerance.length() + 9, 1);
        lcd.print(key);
        delay(200);
      } else if (key == '#') {
        break;
      }
    }
  }
  pHTolerance = inputTolerance.toFloat();
  EEPROM.put(ToleranceAddress, pHTolerance);
  EEPROM.get(ToleranceAddress, pHTolerance);
  lcd.clear();
  lcd.print("Tolerance = ");
  lcd.print(pHTolerance, 2);
  delay(2000);
  adjustDoseDuration();
}

// Change mix time (user input) and then pH tolerance and dose duration
void changeMixTime() {
  lcd.clear();
  lcd.print("Enter Mix Time:");
  String inputTime = "";
  char key;
  while (true) {
    key = keypad.getKey();
    if (key != NO_KEY) {
      if (key >= '0' && key <= '9') {
        inputTime += key;
        lcd.print(key);
        delay(200);
      } else if (key == '#' || key == '*') {
        break;
      }
    }
  }
  MixTime = inputTime.toInt();
  EEPROM.put(mixTimeAddress, MixTime);
  EEPROM.get(mixTimeAddress, MixTime);
  lcd.clear();
  lcd.print("Mix Time = ");
  lcd.print(MixTime);
  lcd.print(" mins");
  delay(2000);
  changePHTolerance();
}

// Calibration process with three modes: Manual, Auto, or Mix/Tolerance/Dose
void calibrate() {
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Calibration Mode");
  delay(500);
  
  lcd.setCursor(0, 1);
  lcd.print("1:Manual 2:Auto");
  lcd.setCursor(0, 2);
  lcd.print("3:Mix/Tol/Dose");
  
  char key = NO_KEY;
  while ((key = keypad.getKey()) == NO_KEY) {
    // Wait for input
  }
  
  if (key == '1') {
    lcd.clear();
    setValues();
    lcd.clear();
    return;
  } else if (key == '3') {
    lcd.clear();
    changeMixTime();
    lcd.clear();
    return;
  } else if (key == '2') {
    // Auto Calibration
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Immerse in pH4");
    lcd.setCursor(0, 2);
    lcd.print("Press 1 when ready");
    waitForKey('1');

    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Reading pH4...");
    float reading_pH4 = getBufferReading(offset, slope);
    delay(1500);

    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("pH4: ");
    lcd.print(reading_pH4, 2);
    delay(1500);

    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Immerse in pH7");
    lcd.setCursor(0, 2);
    lcd.print("Press 1 when ready");
    waitForKey('1');

    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Reading pH7...");
    float reading_pH7 = getBufferReading(offset, slope);
    delay(1500);

    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("pH7: ");
    lcd.print(reading_pH7, 2);
    delay(1500);

    // Validate readings (adjust thresholds as needed)
    bool valid = true;
    if (reading_pH4 < 3.0 || reading_pH4 > 5.0) {
      lcd.clear();
      lcd.print("pH4 reading error!");
      valid = false;
    }
    if (reading_pH7 < 5.0 || reading_pH7 > 9.0) {
      lcd.clear();
      lcd.print("pH7 reading error!");
      valid = false;
    }
    if (!valid) {
      delay(3000);
      lcd.clear();
      lcd.print("Retry calibration");
      delay(2000);
      calibrate();
      return;
    }

    float newSlope = (reading_pH7 - reading_pH4) / 3.0;
    float newOffset = reading_pH4 - (newSlope * 4.0);
    EEPROM.put(0, newSlope);
    EEPROM.put(4, newOffset);
    EEPROM.get(0, slope);
    EEPROM.get(4, offset);

    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("New Slope: ");
    lcd.print(slope, 2);
    lcd.setCursor(0, 2);
    lcd.print("New Offset: ");
    lcd.print(offset, 2);
    delay(5000);
    return;
  }
}

// Buffer reading: average multiple ADC samples to obtain a stable pH value
float getBufferReading(float offset, float slope) {
  const int numReadings = 10;
  float sumVoltages = 0;
  for (int i = 0; i < numReadings; i++) {
    int32_t adc_code = 0;
    uint8_t ack = adc_read(i2c_address, &adc_code, eoc_timeout);
    float ProbeVoltage = adc_code_to_voltage(adc_code, adc_vref) * 1000 + adc_offset;
    sumVoltages += ProbeVoltage;
    delay(100);
  }
  float avgVoltage = sumVoltages / numReadings;
  float local_pH = avgVoltage * (5.0 / (-1024.0 * 6.0));
  local_pH = (1.1 * 3.5 * local_pH + 6.00);
  
  // Temperature compensation
  if (t < 25) {
    Tempoffset = (25 - Tc) * 0.0010;
  } else if (t > 25) {
    Tempoffset = (t - 25) * 0.0015;
  }
  if (t > 25)
    local_pH += Tempoffset;
  else
    local_pH -= Tempoffset;
  
  Serial.print("Buffer pH: ");
  Serial.println(local_pH, 2);
  return local_pH;
}
