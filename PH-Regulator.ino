#include <DHT22.h>
#include <Arduino.h>
#include <stdint.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad_I2C.h>
#include <Keypad.h>
#include <Wire.h>
#include <EEPROM.h>
#include "ADC.h"

// ——— GLOBALS & CONSTANTS ———

// I²C Addresses
#define I2CADDR               0x26  // i2c address for keypad
#define ADC_I2C_ADDRESS       0x24  // i2c addressc for pH Probe
#define PIN_DATA              10    // DHT22 data pin (if used)

// Analog pins
const int SensorPin           = A3;  // analog pH sensor input (if used)
const int tankSensorPin       = A2;  // Tank level sensor

// Digital pins
const int PumpPin             = A0;
const int LED                 = 5;   
const int ThermistorPin       = A1;

// EEPROM addresses
const int mixTimeAddress      = 8;
const int ToleranceAddress    = 12;
const int DoseDurationAddress = 16;
const int tankCheckAddress    = 20;
const int tankSizeAddress     = 24;



// Moving‑average window
#define WINDOW_SIZE 3

// ADC (pH module) settings
static const float adc_vref = 5.0f; // Correct ADC reference to 5.0V
static const float   adc_offset   = 0.0f;
static const uint8_t i2c_addr     = ADC_I2C_ADDRESS;
static const uint16_t eoc_to      = 300;

// Calibration & live readings
float slope, offset, pHTarget;
float pHValue;
float pHVoltages[WINDOW_SIZE];
int   INDEX = 0;
float pHavg[10];
float avgValue = 0.0f;

// Thermistor constants & live temp
const float R1 = 10000.0f,
            c1 = 0.0012400f,
            c2 = 0.000209125f,
            c3 = 0.0000002956720f;
float Tc, Tempoffset;
int   Vo;
float R2, logR2, Tcalc;

// Tank‑level enforcement
bool enforceTankLevel = true;
float tankHeight;  // tank height in meters

// Process parameters (loaded from EEPROM)
int MixTime;
int DoseDuration;
float pHTolerance;

// Tank‑sensor constants for smoothing
const float MAX_TANK_HEIGHT = 0.10f,
            SENSOR_OFFSET_V = 0.5f,
            SENSOR_SPAN_V   = 4.0f,
            SENSOR_MAX_P    = 0.5f,
            ALPHA           = 0.10f,
            ADC_REF_V       = 5.0f;


// Buffers
float pHReadings[WINDOW_SIZE];

// LCD & keypad
LiquidCrystal_I2C lcd(0x27, 20, 4);
const byte ROWS = 4, COLS = 3;
char keys[ROWS][COLS] = {{'1','2','3'},{'4','5','6'},{'7','8','9'},{'.','0','#'}};
byte rowPins[ROWS] = {5,0,1,3};
byte colPins[COLS] = {4,6,2};
Keypad_I2C keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS, I2CADDR);
DHT22 dht22(PIN_DATA);

// ——— Helpers ———
void clearRow(uint8_t row) {
  lcd.setCursor(0, row);
  for (uint8_t i = 0; i < 20; ++i) lcd.print(' ');
}

void printCentered(uint8_t row, const String &txt) {
  const uint8_t LCD_W = 20;
  uint8_t len   = txt.length();
  uint8_t start = (len < LCD_W) ? (LCD_W - len) / 2 : 0;
  clearRow(row);
  lcd.setCursor(start, row);
  lcd.print(txt);
}

void waitForKey(char target) {
  char key;
  while ((key = keypad.getKey()) != target) {
    // wait
  }
  delay(200);
}

// ——— Prototypes ———
float readLevelPercent();
int8_t read_adc();
void display_and_dosing_logic(float pHValue, float Tc, float Tempoffset);
void startup();
void setValues();
void changeMixTime();
void changePHTolerance();
void adjustDoseDuration();
void calibrate();
float getBufferReading(float offset, float slope);

void setup() {
  lcd.init(); lcd.backlight();
  lcd.print("pH Regulator");
  for (int i = 0; i < 24; i++) { lcd.scrollDisplayRight(); delay(60); }
  Serial.begin(115200);
  pinMode(PumpPin, OUTPUT);
  pinMode(SensorPin, INPUT);
  pinMode(LED, OUTPUT);
  Wire.begin(); keypad.begin();
  EEPROM.get(0, slope);
  EEPROM.get(4, offset);
  EEPROM.get(mixTimeAddress, MixTime);
  EEPROM.get(ToleranceAddress, pHTolerance);
  EEPROM.get(DoseDurationAddress, DoseDuration);
  EEPROM.get(tankCheckAddress, enforceTankLevel);
  if (enforceTankLevel != true && enforceTankLevel != false) enforceTankLevel = true;
  EEPROM.get(tankSizeAddress, tankHeight);
  if (isnan(tankHeight) || tankHeight <= 0.01f || tankHeight > 5.0f) { // Default safety check
  tankHeight = 0.10f;  // default to 0.1 meters if EEPROM is invalid
  EEPROM.put(tankSizeAddress, tankHeight);
}
  startup();
}

unsigned long previousLcdUpdate = 0;
const unsigned long lcdInterval = 500; // Update LCD every 3 seconds

void loop() {
  char key = keypad.getKey();
  if (key == '.') {
    digitalWrite(PumpPin, HIGH);
    digitalWrite(LED, HIGH);
    delay(5000);
  } else {
    digitalWrite(PumpPin, LOW);
    digitalWrite(LED, LOW);
  }

  if (key == '3') {
    lcd.clear();
    lcd.setCursor(5, 0);
    lcd.print("Restart...");
    delay(1000);
    asm volatile("jmp 0"); // Reset Arduino
  }

  if (key == '6') setValues();

  int8_t ack = read_adc();
  if (ack) Serial.println(F("***** I2C ERROR *****"));
  else display_and_dosing_logic(pHValue, Tc, Tempoffset);
}

void display_and_dosing_logic(float pHValue, float Tc, float Tempoffset) {
  printCentered(0, "Current pH: " + String(pHValue, 1));
  printCentered(1, "Target pH: " + String(pHTarget, 1));
  printCentered(2, "Temp: "   + String(Tc, 1) + String((char)223) + "C");
  printCentered(3, "Tank Level: "   + String(readLevelPercent(), 1) + "%");
  if (abs(pHValue - pHTarget) > 4.0) { digitalWrite(PumpPin, LOW); digitalWrite(LED, LOW); return; }

  if (pHValue > pHTarget + pHTolerance) {
    float level = readLevelPercent();
    if (enforceTankLevel && level < 35.0) {
      printCentered(1, "Low Tank: No Dose."); delay(5000);
      printCentered(1, "Tgt pH: " + String(pHTarget, 2));
      return;
    }
    float scale = constrain(0.5 + (level - 35.0)/65.0, 0.5, 1.5);
    unsigned long totalMs = (unsigned long)(DoseDuration * scale * 1000UL);
    unsigned long startMs = millis();
    printCentered(0, "Dosing...");
    digitalWrite(PumpPin, HIGH); digitalWrite(LED, HIGH);
    while (millis() - startMs < totalMs) {
      unsigned long el = millis() - startMs;
      uint8_t bar = (el * 20) / totalMs;
      clearRow(2);
      lcd.setCursor(0,2);
      for (uint8_t i=0;i<bar;i++) lcd.write(byte(255));
      printCentered(3, String((el*100)/totalMs) + "%");
      }
    digitalWrite(PumpPin, LOW); digitalWrite(LED, LOW); delay(500);
    delay(1500);
    lcd.clear();
    printCentered(0, "Mixing..");
    unsigned long mixMs = MixTime * 60UL * 1000UL;
    unsigned long ms0 = millis();
    while (millis() - ms0 < mixMs) {
      read_adc(); // <- Get updated pH & temp values
      float cph = pHReadings[INDEX];
      float cl = readLevelPercent();

      unsigned long el = millis() - ms0;
      uint8_t mbar = (el * 20) / mixMs;
      clearRow(2);
      lcd.setCursor(0, 2);
      for (uint8_t i = 0; i < mbar; i++) lcd.write(byte(255));

      uint32_t secs = (mixMs - el) / 1000;
      String tm = String(secs / 60) + ":" + (secs % 60 < 10 ? "0" : "") + String(secs % 60);
      printCentered(3, tm);

      printCentered(1, "Current pH: " + String(cph, 1)); // <- Show pH during mix
      delay(800);
      digitalWrite(LED, ((el / 500) & 1) ? HIGH : LOW);
    }

    digitalWrite(LED, LOW);
    lcd.clear();
    unsigned long s0 = millis(); int cnt=0; float sp=0, st=0;
    while (millis()-s0 < 5000) {
      read_adc(); float cph=pHReadings[INDEX], cl=readLevelPercent();
      sp+=cph; st+=cl; cnt++;
      printCentered(0, "Cur pH: " + String(cph,1));
      printCentered(3, "Tank: " + String(cl,1) + "%");
      }
    pHValue = sp/cnt; float lvl2 = st/cnt;
    printCentered(0, "Current pH: " + String(pHValue,1));
    printCentered(1, "Target pH: " + String(pHTarget,1));
    printCentered(2, "Temp: " + String(Tc,1) + String((char)223) + "C");
    printCentered(3, "Tank Level: " + String(lvl2,1) + "%");
  }
}

int8_t read_adc() {
  int32_t adc_code = 0;
  uint8_t ack = 0;
  float ProbeVoltage = 0.0f;
  const int samples = 5; // more samples improves accuracy
  float voltageSum = 0.0f;

  for (int i = 0; i < samples; i++) {
    ack |= adc_read(i2c_addr, &adc_code, eoc_to);
    ProbeVoltage = adc_code_to_voltage(adc_code, adc_vref) + (adc_offset / 1000.0f); // volts
    voltageSum += ProbeVoltage;
    delay(10);
  }
  avgValue = voltageSum / samples; // in volts now

  // Corrected pH calculation (Volts used, not millivolts)
  pHValue = slope * avgValue + offset;

  // Thermistor reading unchanged (temp calculation remains same)
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0f / float(Vo) - 1.0f);
  logR2 = log(R2);
  Tcalc = 1.0f / (c1 + c2 * logR2 + c3 * pow(logR2, 3));
  Tc = Tcalc - 273.15f;

  if (Tc < 25.0f) pHValue -= (25.0f - Tc) * 0.0010f;
  else if (Tc > 25.0f) pHValue += (Tc - 25.0f) * 0.0015f;

  Serial.print("pH: "); Serial.println(pHValue, 2);
  Serial.print("Avg Voltage: "); Serial.print(avgValue * 1000.0f, 2); Serial.println(" mV");
  Serial.print("Temperature: "); Serial.print(Tc, 2); Serial.println(" C");

  pHReadings[INDEX] = pHValue;
  INDEX = (INDEX + 1) % WINDOW_SIZE;

  return ack;
}


float readLevelPercent() {
  long tot=0; for(int i=0;i<10;i++){ tot+=analogRead(tankSensorPin); delay(5);} 
  float raw=tot/10.0;
  float V = raw*(ADC_REF_V/1023.0);
  float P = constrain((V-SENSOR_OFFSET_V)*(SENSOR_MAX_P/SENSOR_SPAN_V),0.0,SENSOR_MAX_P);
  float h = P/0.0098;
  float lvl = constrain((h / tankHeight) * 100.0f, 0.0f, 100.0f);
  static float s=lvl;
  s = ALPHA*lvl + (1-ALPHA)*s;
  return s;
}

void setTankSize() {
  lcd.clear();
  printCentered(0, "Enter Tank Size:");
  printCentered(1, "(cm, then '#')");

  String input = "";
  char key;
  while (true) {
    key = keypad.getKey();
    if (key != NO_KEY) {
      if ((key >= '0' && key <= '9') || key == '.') {
        input += key;
        lcd.setCursor(input.length() + 8, 2);
        lcd.print(key);
        delay(200);
      } else if (key == '#') {
        break;
      }
    }
  }

  float enteredSize = input.toFloat();
  if (enteredSize < 1.0f || enteredSize > 500.0f) { // Reasonable limits: 1 cm to 500 cm (5 meters)
    lcd.clear();
    printCentered(1, "Invalid Size!");
    delay(2000);
    return setTankSize(); // retry if invalid
  }

  tankHeight = enteredSize / 100.0f; // convert cm to meters
  EEPROM.put(tankSizeAddress, tankHeight);

  lcd.clear();
  printCentered(1, "Tank Size Set:");
  printCentered(2, input + " cm");
  delay(2000);
}


void startup() {
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
      lcd.setCursor(11, 2);
      lcd.print(MixTime);
      lcd.print(" Mins");
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
      startup(); // Re-run target entry
      lcd.clear();
      return;
    }
  }
}

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
      lcd.setCursor(5, 2);
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

void calibrate() {
  lcd.clear();
  printCentered(0, "--Settings--");
  delay(500);

printCentered(1, "1:Manual 2:Auto");
printCentered(2, "3:Mix- Tol- Dose-");
printCentered(3, "4:TankCh 5:Tank Size");

  char key = NO_KEY;
  while ((key = keypad.getKey()) == NO_KEY) {}

  switch (key) {
    case '5':
    setTankSize();
    asm volatile("jmp 0"); // Reset to apply changes immediately
    break;

    case '4':
      enforceTankLevel = !enforceTankLevel;
      EEPROM.put(tankCheckAddress, enforceTankLevel);
      EEPROM.get(tankCheckAddress, enforceTankLevel);
      lcd.clear();
      printCentered(1, "Tank Check:");
      printCentered(2, enforceTankLevel ? "ENABLED" : "DISABLED");
      delay(2000);
      asm volatile("jmp 0");
      break;

    case '1':
      lcd.clear();
      setValues();
      return;

    case '3':
      lcd.clear();
      changeMixTime();
      return;

    case '2': {
      // Auto Calibration
      float reading_pH4 = 0.0f, reading_pH7 = 0.0f;

      // pH 4 Calibration
      lcd.clear();
      printCentered(0, "Immerse in pH4");
      printCentered(3, "Press '1' ready");
      char key = NO_KEY;
      while ((key = keypad.getKey()) != '1') {
        read_adc();
        printCentered(1, "Cur pH: " + String(pHValue, 2));
        printCentered(2, "Voltage: " + String(avgValue, 3) + "mV");
        delay(250);
      }
      lcd.clear();
      printCentered(1, "Reading pH4...");
      reading_pH4 = getBufferReading();  // in Volts
      lcd.clear();
      printCentered(1, "pH4 Voltage:");
      printCentered(2, String(reading_pH4 * 1000.0f, 2) + " mV");
      delay(2000);

      // pH 7 Calibration
      lcd.clear();
      printCentered(0, "Immerse in pH7");
      printCentered(3, "Press '1' ready");
      key = NO_KEY;
      while ((key = keypad.getKey()) != '1') {
        read_adc();
        printCentered(1, "Cur pH: " + String(pHValue, 2));
        printCentered(2, "Voltage: " + String(avgValue, 3) + "mV");
        delay(250);
      }
      lcd.clear();
      printCentered(1, "Reading pH7...");
      reading_pH7 = getBufferReading();  // in Volts
      lcd.clear();
      printCentered(1, "pH7 Voltage:");
      printCentered(2, String(reading_pH7 * 1000.0f, 2) + " mV");
      delay(2000);

      // Check voltage validity
      if (abs(reading_pH7 - reading_pH4) < 0.05f) {  // realistic minimum difference ~50mV
        lcd.clear();
        printCentered(1, "Calibration Error!");
        printCentered(2, "Retrying...");
        delay(3000);
        return calibrate();  // Retry
      }

      // Correct slope/offset calculation
      float newSlope = (7.0f - 4.0f) / (reading_pH7 - reading_pH4);
      float newOffset = 7.0f - newSlope * reading_pH7;

      EEPROM.put(0, newSlope);
      EEPROM.put(4, newOffset);
      slope = newSlope;
      offset = newOffset;

      lcd.clear();
      printCentered(0, "Calibration Done!");
      lcd.setCursor(0, 2);
      lcd.print("Slope: "); lcd.print(slope, 4);
      lcd.setCursor(0, 3);
      lcd.print("Offset: "); lcd.print(offset, 4);
      delay(5000);
      return;
    }

    default:
      lcd.clear();
      printCentered(1, "Invalid Option");
      delay(2000);
      return;
  }
}

float getBufferReading() {
  const int numReadings = 20;
  float sumVoltages = 0.0f;
  int32_t adc_code = 0;

  for (int i = 0; i < numReadings; i++) {
    adc_read(i2c_addr, &adc_code, eoc_to);
    float probeVoltage = adc_code_to_voltage(adc_code, adc_vref) + (adc_offset / 1000.0f); // now volts
    sumVoltages += probeVoltage;
    delay(100);
  }

  float avgVoltage = sumVoltages / numReadings;

  Serial.print("Calibration Voltage: ");
  Serial.print(avgVoltage * 1000.0f, 2);
  Serial.println(" mV");

  return avgVoltage;  // in volts for correct slope calc
}





