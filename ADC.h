#ifndef ADC_H
#define ADC_H


//! This union splits one int32_t (32-bit signed integer) or uint32_t (32-bit unsigned integer)
//! four uint8_t's (8-bit unsigned integers) and vice versa.
union LT_UNION_INT32_4BYTES
{
  int32_t  LT_INT32;       //!< 32-bit signed integer to be converted to four bytes
  uint32_t LT_UINT32;     //!< 32-bit unsigned integer to be converted to four bytes
  uint8_t  LT_BYTE[4];     //!< 4 bytes (unsigned 8-bit integers) to be converted to a 32-bit signed or unsigned integer
};


// Select rejection frequency - 50 and 60, 50, or 60Hz
#define ADC_R50         0b00000010
#define ADC_R60         0b00000100
#define ADC_R50_R60     0b00000000

// Speed settings is bit 7 in the 2nd byte
#define SLOW 0b00000000 // slow output rate with autozero
#define FAST 0b00000001 // fast output rate with no autozero


/*Commands
Construct a channel / resolution control word by bitwise ORing one choice from the channel configuration
and one choice from the Oversample ratio configuration. You can also enable 2Xmode, which will increase
sample rate by a factor of 2 but introduce an offset of up to 2mV. */

//! Reads from ADC.
//! @return  1 if no acknowledge, 0 if acknowledge
uint8_t adc_read(uint8_t i2c_address,   //!< I2C address (7-bit format) for part
                 int32_t *adc_code,     //!< 4 byte conversion code read from ADC
                 uint16_t timeout       //!< Timeout in ms
                    );

//! Calculates the voltage corresponding to an adc code, given the reference (in volts)
//! @return Returns voltage calculated from the ADC code.
float adc_code_to_voltage(int32_t adc_code,     //!< Code read from adc
                          float vref            //!< VRef (in volts)
                             );

#endif  // ADC_H