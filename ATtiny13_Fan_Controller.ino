/*
  ATtiny13_Fan_Controller.ino - Fan controller based on Dallas Semiconductors DS18B20 digital temperature sensor and ATtiny13 microcontroller
  
  MIT License

  Copyright (c) 2022 Dmitry Muravyev (youtube.com/@DmitryMuravyev)

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constants and Types
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//

const uint16_t PWM_top = 255;                   // PWM Top for 19.5 KHz frequency: 5 MHz / 256 = 19.531 KHz
const int16_t f_min_rps_diff = 25;              // 25 pulses = 12.5 RPS = 750 RPM - the minimum difference between 100% and 0% PWM Duty to detect if PWM and Hall sensor are available
const int16_t f_min_rps = 4;                    // 4 pulses = 2 RPS = 120 RPM - the minimum RPM to check if Fan runs
const int8_t f_max_restarts = 10;               // The maximum number of attempts to start the fan, after which the controller goes into power management mode (On-Off)
const int8_t f_acc_dec_time = 80;               // 80 x 0.05s = 4s for Fan acceleration/deceleration (this can be increased for larger Fans)
const int8_t f_measurement_time = 20;           // 20 x 0.05s = 1s for RPS measurement

// We get from the DS18B20 sensor the temperature measured in 1/16 of degree (Celsius). For greater accuracy and less code, we will use integer values.
const int16_t t_lower_threshold = 400;          // 25C (25 x 16 = 400) - below this temperature the Fan is Off or runs at minimum speed (0%)
const int16_t t_upper_threshold = 640;          // 40C (40 x 16 = 640) - above this temperature the Fan is On or runs at maximum speed (100%)
const int16_t t_critical_threshold = 800;       // 50C (50 x 16 = 800) - alarm temperature
const int16_t t_no_pwm_trigger = 480;           // 30C (30 x 16 = 480) - switch-on threshold, used in the power management mode (On-Off)

const uint8_t pwm_pin = 0;                      // Pin of the timer channel A (PWM)
const uint8_t tach_pin = 1;                     // INT0 pin
const uint8_t beeper_pin = 2;                   // Buzzer pin
const uint8_t dc_dc_enable_pin = 3;             // Pin of the DC-DC converter's EN input
const uint8_t one_wire_pin = 4;                 // DS18B20 sensor data pin
const uint8_t program_pin = 5;                  // list of sensors programming mode activation pin (active level is low)
const uint8_t skip_ROM_command = 0xCC;          // Skip ROM command
const uint8_t read_ROM_command = 0x33;          // Read ROM command
const uint8_t match_ROM_command = 0x55;         // Match ROM command
const uint8_t convert_command = 0x44;           // Start temperature conversion command
const uint8_t read_scratchpad_command = 0xBE;   // Read Scratchpad command

const uint8_t EEPROM_size = 64;                 // EEPROM capacity


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Variables and definitions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//

static uint8_t fanControlMode = 0;              // Control mode (PWM / On-Off) + counter of Fan start attempts. The default mode is power management (On-Off).
volatile int16_t sensorPulses;                  // Hall sensor pulses counter

static union {
  uint8_t dataBytes[9];                         // Buffer for DS18B20 sensor data
  int16_t temperature;                          // Temperature bytes
};

#define ResetResult(x)  (x)[2]                  // Use the 3rd byte to temporarily store the results of resetting OneWire 
#define ConfigReg(x)  (x)[4]                    // DS18B20 configuration register
#define SensorError    { fanOn(PWM_top); beep(2); return; }           // Sensor error Macro


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Interrupt handlers
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// INT0 HW interrupt handler
/*
ISR(INT0_vect) {
  sensorPulses++;
}
*/

// The compiler generates a several unnecessary push/pop, so let's write our own handler
ISR(INT0_vect, ISR_NAKED) {
  __asm__ __volatile__ (
    "push r24                   \n"
    "in   r24, __SREG__         \n"
    "push r24                   \n"
    "push r25                   \n"

    "lds  r24, %[counter]       \n"
    "lds  r25, %[counter]+1     \n"
    "adiw  r24, 0x01            \n"             // We didn't have this operation in ATtiny10, so we can save an additional 2 bytes of flash memory
 //   "subi r24,0xFF              \n"
 //   "sbci r25,0xFF              \n"
    "sts  %[counter], r24       \n"
    "sts  %[counter]+1, r25     \n"

    "pop  r25                   \n"
    "pop  r24                   \n"
    "out  __SREG__, r24         \n"
    "pop  r24                   \n"
    "reti                       \n"
    : 
    : [counter] "m" (sensorPulses)
    : //"r24", "r25"            // We will take care of the registers ourselves
  );
}

// We can also use PCINT0 interrupt (Pin Change), but it triggers on any change, and the INT0 can be configured to trigger on the rising edge of the pulse
/*
ISR(PCINT0_vect) {
  sensorPulses++;
}
*/


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//

void setup() {

// Calibrate Clock
  OSCCAL = 89;                  // 0-127 Oscillator Frequency calibration value: set the frequency to ~5 MHz (should be calibrated manually)

// Set PWM output
  TCCR0A = _BV(COM0A1) | _BV(WGM01) | _BV(WGM00);                     // Channel_A Fast PWM (mode 3). Clear OC0A on match, set at bottom, TOP = 0xFF.
  TCCR0B = _BV(CS00);           // Fast PWM (mode 3), PWM prescaler = 1

// Configure pins
  // Set PB0, PB2, PB3, PB4 as OUTPUTs, PB1, PB5 as INPUTs
  DDRB = (0 << program_pin) | (1 << one_wire_pin) | (1 << dc_dc_enable_pin) | (1 << beeper_pin) | (0 << tach_pin) | (1 << pwm_pin);    
  PORTB = (1 << program_pin) | (1 << dc_dc_enable_pin);               // Turn on DC/DC converter, pull-up PB5 to VCC, other pins to 0

// Welcome beep
  beep(1);

// Enter programming mode if PB5 is pulled to the ground
//  if (false) {                // Debugging mode
  if ((PINB & (1 << program_pin)) == 0) {
    
    for (uint8_t i = 0; i < EEPROM_size; i++) EEPROM_write(i, 0xFF);  // Erase EEPROM before new programming

    while (true) {
      delay005s(f_measurement_time);
      oneWireReset();                           // Reset OneWire

      if (ResetResult(dataBytes) == 0) {
        oneWireWrite(read_ROM_command);         // In the programming mode, the sensors are supposed to be connected one by one
        oneWireReadBytes(8);                    // Read sensor's ROM
    
        if (oneWireCRC(8) == 0) {               // Check ROM CRC

          uint8_t i = 0;
          bool notFound;
          do {                                  // Search for the sensor's ROM in the saved list
            notFound = false;
            for (uint8_t j = 0; j < 8; j++) {
              uint8_t val = EEPROM_read(i + j);
//              if ((~val | j) == 0) {          // Beware of integer promotion when performing bitwise operations on integer types smaller than int:
                                                // https://wiki.sei.cmu.edu/confluence/display/c/EXP14-C.+Beware+of+integer+promotion+when+performing+bitwise+operations+on+integer+types+smaller+than+int
              if (((val ^ 0xFF) | j) == 0) {    // End of list reached
                notFound = true;
                goto writeEEPROM;
              }

              if (val != dataBytes[j]) {        // Doesn't match. Go to the next one.
                notFound = true;
                break;
              }
            }
          } while (notFound && ((i += 8) < EEPROM_size));

writeEEPROM:
          
          if (notFound && (i < EEPROM_size)) {            // If the ROM is not found and we have not reached the EEPROM limits
            beep(1);
            uint8_t j = 0;
            for (j = 0; j < 8; j++) {
              EEPROM_write(i + j, dataBytes[j]);          // Save sensor's ROM to the list
            }
          } else {
            beep(2);                            // ROM was found or EEPROM is full
          }
        } else {
          beep(2);                              // Sensor's ROM CRC Error
        }

      }
    }
  }

// Configure interrupts
  MCUCR |= ((1 << ISC01) | (1 << ISC00));       // The rising edge of INT0 will generate an interrupt request
  GIMSK = 1 << INT0;                            // Enable hardware INT0 interrupts

// We can also use PCINT0 interrupt (Pin Change), but it triggers on any change, and the INT0 can be configured to trigger on the rising edge of the pulse
//  PCMSK = (1 << tach_pin);                    // Set PCINT pins
//  GIMSK = 1 << PCIE0;                         // Enable Pin Change interrupts


// Detect if PWM and Hall sensor are available
  OCR0A = PWM_top;                              // PWM 100%
  delay005s(f_acc_dec_time);                    // Fan acceleration
  measureRPS();
  int16_t highRPS = sensorPulses;
  OCR0A = 0;                                    // PWM 0%
  delay005s(f_acc_dec_time);                    // 4s for deceleration (this can be increased for larger Fans)
  measureRPS();

  if ((highRPS - sensorPulses) > f_min_rps_diff) {
    fanControlMode = 1;                         // PWM control mode
  }

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Delay functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Delay for microseconds
// Since the only timer is occupied by PWM - we cannot use it directly, so we will use workaround based on processor cycles.
void delayMicros(uint16_t microseconds) {

// Small cycle for less than 10 microseconds
  if (microseconds < 10) {
    __asm__ __volatile__ (
      "nop                      \n"             // 1 cycle
      "1: dec  %A[counter]      \n"             // 1 cycle
      "brne  1b                 \n"             // 2 cycles
      :
      // Output operands
      :
      // Input operands
      [counter] "r" (microseconds)
      :
      // Clobbers
    );
    return;
  }

  microseconds -= 3;            // Correction for function initialization time (you can play with this parameter to get the best results over the entire range)

// Large cycle for large delays
  __asm__ __volatile__ (
    "1: nop                     \n"             // 1 cycle
    "subi  %A[counter], 1       \n"             // 1 cycle
    "sbci  %B[counter], 0       \n"             // 1 cycle
    "brcc  1b                   \n"             // 2 cycles

    //
    // We have 5 CPU cycles in total, so DELAY = 5MHz / (microseconds * 5 cycles)
    //
    
    :
    // Output operands
    :
    // Input operands
    [counter] "r" (microseconds)
    :
    // Clobbers
  );
}

// Delay for N x 0.05s
void delay005s(uint8_t milliseconds) {
  while (--milliseconds) delayMicros(50000);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// OneWire functions, based on this project: http://www.technoblogy.com/show?2G8A (Licensed under a Creative Commons Attribution 4.0 International license: http://creativecommons.org/licenses/by/4.0/ )
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Configure OneWire pin as OUTPUT (low level was pre-configured in setup())
inline void pinLow() {
  DDRB = DDRB | (1 << one_wire_pin);
}

// Configure OneWire pin as INPUT
inline void pinRelease() {
  DDRB = DDRB & ~(1 << one_wire_pin);
}

// Read OneWire pin
inline uint8_t pinRead() {
  return ((PINB >> one_wire_pin) & 1);
}

// Transmit inverted 1-0 sequence
void pinLowRelease(uint16_t low, uint16_t high) {
  pinLow();
  delayMicros(low);
  pinRelease();
  delayMicros(high);
}

// Initialize OneWire
void oneWireReset() {
  pinLowRelease(480, 70);
  ResetResult(dataBytes) = pinRead();
  delayMicros(410);
}

// Write OneWire pin
void pinWrite(uint8_t data) {
  uint16_t low, high;
  if (data == 0) {
    low = 65;
    high = 5;
  } else {
    low = 10;
    high = 55;
  }
  pinLowRelease(low, high);
}

// Write byte
void oneWireWrite(uint8_t data) {
  for (uint8_t i = 0; i < 8; i++) {
    pinWrite(data & 1);
    data = data >> 1;
  }
}

// Read byte
uint8_t oneWireRead() {
  uint8_t data = 0;
  for (uint8_t i = 0; i < 8; i++) {
    pinLowRelease(3, 10);
    data = data | (pinRead() << i);
    delayMicros(53);
  }
  return data;
}

// Read bytes into the buffer
void oneWireReadBytes(uint8_t bytes) {
  for (uint8_t i = 0; i < bytes; i++) {
    dataBytes[i] = oneWireRead();
  }
}

// Calculate CRC over the buffer - 0x00 is correct
uint8_t oneWireCRC(uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t j = 0; j < len; j++) {
    crc = crc ^ dataBytes[j];
    for (uint8_t i = 0; i < 8; i++) crc = (crc >> 1) ^ ((crc & 1) ? 0x8c : 0);
  }
  return crc;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// EEPROM functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Write byte to EEPROM
void EEPROM_write(uint8_t ucAddress, uint8_t ucData) {
  while(EECR & (1<<EEPE));      // Wait for completion of previous write
  EECR = (0<<EEPM1)|(0>>EEPM0); // Set Programming mode
  EEARL = ucAddress;            // Set up address and data registers
  EEDR = ucData;
  EECR |= (1<<EEMPE);           // Write logical one to EEMPE
  EECR |= (1<<EEPE);            // Start eeprom write by setting EEPE
}

// Read byte from EEPROM
unsigned char EEPROM_read(uint8_t ucAddress) {
  while(EECR & (1<<EEPE));      // Wait for completion of previous write
  EEARL = ucAddress;            // Set up address register
  EECR |= (1<<EERE);            // Start eeprom read by writing EERE
  return EEDR;                  // Return data from data register
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Other functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Switch DC/DC On and set PWM
inline void fanOn(uint8_t duty) {
  OCR0A = duty;
  PORTB = PORTB | (1 << dc_dc_enable_pin);
}

// Switch DC/DC Off and set PWM to 0%
inline void fanOff() {
  OCR0A = 0;
  PORTB = PORTB & ~(1 << dc_dc_enable_pin);
}

// Measure Fan RPS
void measureRPS () {
  sensorPulses = 0;
  sei();                                        // Enable interrupts SREG |= (1<<SREG_I)
  delay005s(f_measurement_time);                // RPS measurement
  cli();                                        // Disable interrupts
}

// Beep N times
void beep(uint8_t count) {
  while (count--) {
    PORTB = PORTB | (1 << beeper_pin);          // Turn buzzer on
    delay005s(2);                               // 100 ms
    PORTB = PORTB & ~(1 << beeper_pin);         // Turn buzzer off
    delay005s(4);                               // 200 ms
  }
}

// Pulse generation for oscillator calibration
void pulse(uint16_t duration) {
  PORTB = PORTB | (1 << one_wire_pin);
  delayMicros(duration);
  PORTB = PORTB & ~(1 << one_wire_pin);
  delayMicros(duration);  
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//

void loop() {

  oneWireReset();                               // Reset OneWire

  if (ResetResult(dataBytes) == 0) {
    oneWireWrite(skip_ROM_command);             // Skip ROM so that the next command addresses all sensors on the bus
    oneWireWrite(convert_command);              // Start temperature conversion
  }

//  delay005s(15);                              // We can just wait, but it is better to check if the Fan is spinning during the conversion
  measureRPS();                                 // Measure RPS while we're waiting for the conversion to complete (we will use DS18B20 configured for 12 bit accuracy, it takes 750ms)

// Read temperature
  int16_t highestTemp = 0xFFFF;                 // We will search for the highest temperature across the sensors

  if (ResetResult(dataBytes) == 0) {

    uint8_t i = 0;
    // Iterate through the sensors list
    while ((EEPROM_read(i) != 0xFF) && (i < EEPROM_size)) {
      oneWireReset();
      oneWireWrite(match_ROM_command);          // Select sensor by it's ROM
      do {
        oneWireWrite(EEPROM_read(i++));
      } while (i & 0b111);

      oneWireWrite(read_scratchpad_command);    // Read Scratchpad
      oneWireReadBytes(9);
      
      // Check data presence and CRC
      if ((ConfigReg(dataBytes) == 0) || (oneWireCRC(9) != 0)) {      // Check for low level on data pin: lower 5 bits of DS18B20 configuration register (dataBytes[4]) should be set to 1
                                                // ...also the 5th register should be set to 0xFF, but my exemplar has the value 0xA5, so we will use the config reg.
        SensorError;                            // If the sensor unavailable or CRC error - set Fan to maximum RPM
      }
  
      if (temperature > highestTemp) highestTemp = temperature;
    }
    
    if (i == 0) SensorError;                    // If no sensor is saved in the EEPROM list - set Fan to maximum RPM

  } else SensorError;                           // If no sensor available - set Fan to maximum RPM

// Check if we can switch off the Fan
  if (highestTemp < t_lower_threshold) {
    fanOff();                                   // Too cold
    return;
  } 

// Depending on the control mode (PWM or power On-Off)
  if (fanControlMode > 0) {

    if (PORTB & (1 << dc_dc_enable_pin)) {      // Check RPS only in PWM mode and running Fan
      if (sensorPulses < f_min_rps) {
        fanOn(PWM_top);                         // Try to start Fan if it's not spinning
        delay005s(f_measurement_time);
        if (++fanControlMode > f_max_restarts) {
          beep(3);
          fanControlMode = 0;                   // Switch to the power management mode (On-Off)
        }
      } else {
        fanControlMode = 1;
      }
    }

    if (highestTemp >= t_upper_threshold) {
      fanOn(PWM_top);                           // Too hot
    } else {

      // Calculate and set the PWM Duty cycle relative to the current temperature.
      // In ATtiny13, we have only 8-bit timer, so we don't have to worry about exceeding 16 bits here.

      fanOn((highestTemp - t_lower_threshold) * PWM_top / (t_upper_threshold - t_lower_threshold));

    }

  } else {

    if (highestTemp >= t_no_pwm_trigger) {      // Switch-on threshold exceeded
      fanOn(PWM_top);
    }

  }

  if (highestTemp >= t_critical_threshold) {    // Critical temperature alarm
    beep(5);
  }

/*
// Pulse generation for oscillator calibration

  delay005s(20);

  pulse(480);
  pulse(410);
  pulse(70);
  pulse(65);
  pulse(55);
  pulse(53);
  pulse(10);
  pulse(5);
  pulse(3);

*/


}


// END-OF-FILE
