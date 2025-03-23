Note: This simple guide and the related example code are provided to help you quickly get started with the pH/ORP module. Detailed             or professional implementation of a pH/ORP meter is not included. You can search around to learn more about 2-point calibration,         automatic temperature compensation(ATC) when needed.

Q & As for beginners:

Q: How does the module work?
A: This module uses a precision op amp to buffer the high impedance output of a pH/ORP sensor with a unity gain. The buffered analog mV     signal is sampled and converted to a 16-bit digital code by a ADC chip. Data could be read out by a microcontroller via the I2C    interface.

Q: What about the range of the sensor input?
A: For hardware version since 2021, the input signal was limited to -1250mV ~ +1250mV.
   Warning: The bipolar range doesn't mean it accepts bipolar signals which is referenced to the system ground GND. 
   Only a standard pH or ORP sensor could be connected to the BNC connector.

Q: How to power the module?
A: It can be powered by any supply from +3.3V to +5V. Of course, a clean source is preferred when you need a good noise performance.

Q: How to connect it to the MCU like an Arduino ?
A: Microcontrollers with an I2C hardware block can be directly interfaced to this module.
   The signals of the I2C interface( screw terminal)  are listed below:

   1. VCC: Power input;  Should be connected to the +3.3V or +5V pin of an Arduino
   2. GND: Power ground; Should be connected to the GND pin of an Arduino
   3. SCL: I2C clock; Should be connected to the SCL pin of an Arduino
   4. SDA: I2C data;  Should be connected to the SDA pin of an Arduino

   Both the SCL and SDA have onboard 10K pull-up resistors.


Q: How to program the slave address when multiple modules would be connected to the same I2C bus? 
A：The slave address could be configured by setting the two hardware address pins( CA1, CA0 ) to three different logic states:

   1. Float: Leave the address pin unconnected.
   2. High:  Connect the address pin to " H ".
   3. Low:   Connect the address pin to " L ".

   Each module supports 6 possible addresses. Example code provides address " #define " for easy configuration.




Please follow the steps below to start your first test:

1. Prepare standard buffer solutions like pH 4 and pH 7 for test/calibration.

2. Connect the pH electrode to the BNC connector and wire the I2C signals to your Arduino. Remember not to reverse VCC and GND. 

3. Power your Arduino. The PWR LED must be lit. 

4. Upload the "Digital_pH_ORP"  under the example folder.

5. Open the "Serial Monitor" terminal and set the baudrate to 9600. The terminal begins to print the sensor output in random mV. 

6. Place the electrode in the pH 7 buffer solution. The value should be appraching to the ideal 0mV zero point. Wait for several       senconds. The offset/zero point of the electrode can be calculated.

7. Remove the electrode from the pH 7 buffer solution and rinse it carefully using DI water. Place the electrode in the second buffer       solution pH 4. The sensor output should be around +180mV( 59.16 mV/pH * 3pH ).

Again: The example code and steps above are only used to help you get started. You're responsible to build a pH/ORP meter in your coding        style.

Enjoy! Please send a message to us if you need any help. Thanks!