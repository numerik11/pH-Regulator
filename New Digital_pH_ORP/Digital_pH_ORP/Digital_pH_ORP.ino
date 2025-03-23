
#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include "ADC.h"

//! I2C address of the pH/ORP module. 
//! Configured by tying the CA0, CA1 pins to H( High ) or L( Low ). Or to be left open for Float.

// I2C Slave Address               //  CA1       CA0    
// #define ADC_I2C_ADDRESS 0x14    //  Low       High       
// #define ADC_I2C_ADDRESS 0x15    //  Low       Float  
// #define ADC_I2C_ADDRESS 0x17    //  Float     High  
   #define ADC_I2C_ADDRESS 0x24    //  Float     Float(default)
// #define ADC_I2C_ADDRESS 0x26    //  High      High
// #define ADC_I2C_ADDRESS 0x27    //  High      Float

// Global variables
static float    adc_vref = 2.5;                      //!< The ADC reference voltage
static uint8_t  i2c_address = ADC_I2C_ADDRESS;       //!< I2C address in 7 bit format for part
static uint16_t eoc_timeout = 300;                   //!< Timeout in ms

static float adc_offset = 0;                         //!< Add an offset value to the ADC output.

void setup()
{
  Wire.begin();                                      // wake up I2C bus
  Serial.begin(9600);                                // Initialize the serial port to the PC
}
  
void loop()
{
  uint8_t acknowledge = 0;

  acknowledge |= read_adc();
  
  if (acknowledge)
  Serial.println(F("***** I2C ERROR *****"));
  
  delay(1000);
}

//! @return 0 if successful, 1 is failure
int8_t read_adc()
{          
  int32_t adc_code = 0;           
  float   adc_voltage = 0;        
  uint8_t ack = 0;
   
          
  ack |= adc_read( i2c_address, &adc_code, eoc_timeout );
  
  adc_voltage = adc_code_to_voltage( adc_code, adc_vref ) * 1000 + adc_offset;  //!< Convert the ADC code to mV voltage

  //!< Convert the mV signal to pH value. 
  //!< ......

  if ( !ack )
  {
    Serial.print( F("*************************\n" ) ); 
    Serial.print( "BNC Input Voltage: " );      
    Serial.print( adc_voltage, 2 );
    Serial.print( F( "mV\n" ) );
  }
  else
  {
    Serial.print( F( "Error in read" ) );
    return 1;
  }
      
  return( 0 );
}
