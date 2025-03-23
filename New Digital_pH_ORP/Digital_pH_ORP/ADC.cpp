#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>
#include "ADC.h"


// Write two command bytes, then receive a block of data
int8_t i2c_one_byte_command_read_block(uint8_t address, uint8_t command, uint8_t length, uint8_t *values)
{
  int8_t ret = 0;
  uint8_t i = (length-1);
  uint8_t readBack = 0;

  Wire.beginTransmission(address);
  //Wire.write(byte(command));

  if (Wire.endTransmission(false)) // endTransmission(false) is a repeated start
  {
    // endTransmission returns zero on success
    Wire.endTransmission();
    return(1);
  }
  readBack = Wire.requestFrom((uint8_t)address, (uint8_t)length, (uint8_t)true);

  if (readBack == length)
  {
    while (Wire.available())
    {
      values[i] = Wire.read();
      if (i == 0)
        break;
      i--;
    }
    return (0);
  }
  else
  {
    return (1);
  }
}


// Read a block of data, no command byte, reads length number of bytes and stores it in values.
int8_t i2c_read_block_data(uint8_t address, uint8_t length, uint8_t *values)
{
	uint8_t i = (length-1);
	int8_t ret = 0;

	ret = Wire.requestFrom((uint8_t)address, (uint8_t)length, (uint8_t)true);

	while (Wire.available())
	{
		values[i] = Wire.read();
		i--;
		if (i == 0)
		break;
	}
	delay(100);
	if(ret == 0)
	return (1);		// Unsuccessful
	else
	return(0);		// Successful
}





//! Reads from the ADC that accepts a 8 bit configuration and returns a 24 bit result.
//! Returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
int8_t adc_i2c_24bit_data(uint8_t i2c_address,int32_t *adc_code,uint16_t eoc_timeout)
{
  int8_t ack;
  uint16_t  timer_count = 0;        // Timer count to wait for ACK
  LT_UNION_INT32_4BYTES data;       // ADC data
  
  while(1)
  {
    ack = i2c_read_block_data(i2c_address, 3, data.LT_BYTE);
    if(!ack) break; // !ack indicates success
    if (timer_count++>eoc_timeout)     // If timeout, return 1 (failure)
      return(1);
    else
      delay(1);
  }

  data.LT_BYTE[3] = data.LT_BYTE[2]; // Shift bytes up by one. We read out 24 bits,
  data.LT_BYTE[2] = data.LT_BYTE[1]; // which are loaded into bytes 2,1,0. Need to left-
  data.LT_BYTE[1] = data.LT_BYTE[0]; // justify.
  data.LT_BYTE[0] = 0x00;
  data.LT_UINT32 >>= 2;  // Shifts data 2 bits to the right; operating on unsigned member shifts in zeros.
  data.LT_BYTE[3] = data.LT_BYTE[3] & 0x3F; // Clear upper 2 bits JUST IN CASE. Now the data format matches the SPI parts.
  *adc_code = data.LT_INT32;
  return(ack); // Success
}

// Calculates the voltage corresponding to an adc code, given the reference voltage (in volts)
// This function handles all differential input parts, including the "single-ended" mode on multichannel
// differential parts. Data from I2C parts must be right-shifted by two bit positions such that the MSB
// is in bit 28 (the same as the SPI parts.)
float adc_diff_code_to_voltage(int32_t adc_code, float vref)
{
  float voltage;

  adc_code -= 0x20000000;             //! 1) Converts offset binary to binary
  voltage=(float) adc_code;
  voltage = voltage / 536870912.0;    //! 2) This calculates the input as a fraction of the reference voltage (dimensionless)
  voltage = voltage * vref;           //! 3) Multiply fraction by Vref to get the actual voltage at the input (in volts)
  return(voltage);
}

// Reads from the ADC.
uint8_t adc_read(uint8_t i2c_address, int32_t *adc_code, uint16_t timeout)
{
  return (adc_i2c_24bit_data(i2c_address, adc_code, timeout));
}

// Calculates the voltage corresponding to an ADC code, given the reference (in volts)
float adc_code_to_voltage(int32_t adc_code, float vref)
{
  return (adc_diff_code_to_voltage(adc_code, vref));
}
