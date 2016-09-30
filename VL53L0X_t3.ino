/* VL53L0X_t3 Basic Example Code
 by: Kris Winer
 date: August 29, 2016
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrates basic VL53L0X proximity sensor functionality including parameterizing 
 the register addresses, initializing the sensor, getting range data out.
 Eventually will use this sensor to recognize hand gestures and control 
 functions through a microcontroller like Teensy 3.1.
 
 Sketch runs on the Teensy 3.1.
 
 This sketch is intended specifically for the VL53L0X breakout board and Add-On Shield for the Teensy 3.1.
 It uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
 
 The Add-on shield can also be used as a stand-alone breakout board for any Arduino, Teensy, or 
 other microcontroller.
 
 The sensor communicates via I2C at 400 Hz or higher.
 SDA and SCL have 2K2 pull-up resistors (to 3.3 V) on the breakout board.
 
 Hardware setup:
 Breakout Board --------- Arduino/Teensy
 3V3 ---------------------- VIN or any digital GPIO pin with digitalWrite(HIGH)!
 SDA -----------------------A4/17
 SCL -----------------------A5/16
 GND ---------------------- GND or any digital GPIO pin with digitalWrite(LOW)!
 
 Note: The VL53L0X breakout board is an I2C sensor and uses the Arduino Wire or Teensy i2c_t3.h library. 
 Even though the voltages up to 5 V, the sensor is not 5V tolerant and we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility filefor the Pro Mini useage.
 The Teensy has no internal pullups and we are using the Wire.begin function of the i2c_t3.h library
 to select 400 Hz i2c speed.
 */
//#include <Wire.h>   
#include <i2c_t3.h>

// VL53L0X Data Sheet
// http://www.st.com/content/ccc/resource/technical/document/datasheet/group3/b2/1e/33/77/c6/92/47/6b/DM00279086/files/DM00279086.pdf/jcr:content/translations/en.DM00279086.pdf
//
/* Device register map */
/** @defgroup VL53L0X_DefineRegisters_group Define Registers
 *  @brief List of all the defined registers
 *  @{
 */
// Record the current time to check an upcoming timeout against
uint16_t timeout_start_ms;
uint16_t io_timeout;
#define startTimeout() (timeout_start_ms = millis())

// Check if timeout is enabled (set to nonzero value) and has expired
#define checkTimeoutExpired() (io_timeout > 0 && ((uint16_t)millis() - timeout_start_ms) > io_timeout)

// These are the registers defined in ST Microelecronics' API
// Note the registers have 2-bytes addresses, but for some reason I can only get sensible data out of the VL53L0X when
// I assume an 8-bit register address using only the LSbyte
//
#define VL53L0X_REG_SYSRANGE_START                        0x0000
  /** mask existing bit in #VL53L0X_REG_SYSRANGE_START*/
  #define VL53L0X_REG_SYSRANGE_MODE_MASK          0x0F
  /** bit 0 in #VL53L0X_REG_SYSRANGE_START write 1 toggle state in
   * continuous mode and arm next shot in single shot mode */
  #define VL53L0X_REG_SYSRANGE_MODE_START_STOP    0x01
  /** bit 1 write 0 in #VL53L0X_REG_SYSRANGE_START set single shot mode */
  #define VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT    0x00
  /** bit 1 write 1 in #VL53L0X_REG_SYSRANGE_START set back-to-back
   *  operation mode */
  #define VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK    0x02
  /** bit 2 write 1 in #VL53L0X_REG_SYSRANGE_START set timed operation
   *  mode */
  #define VL53L0X_REG_SYSRANGE_MODE_TIMED         0x04
  /** bit 3 write 1 in #VL53L0X_REG_SYSRANGE_START set histogram operation
   *  mode */
  #define VL53L0X_REG_SYSRANGE_MODE_HISTOGRAM     0x08


#define VL53L0X_REG_SYSTEM_THRESH_HIGH               0x000C
#define VL53L0X_REG_SYSTEM_THRESH_LOW                0x000E


#define VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG    0x0001
#define VL53L0X_REG_SYSTEM_RANGE_CONFIG     0x0009
#define VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD  0x0004


#define VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO                  0x000A
  #define VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_DISABLED  0x00
  #define VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_LEVEL_LOW 0x01
  #define VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_LEVEL_HIGH  0x02
  #define VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_OUT_OF_WINDOW 0x03
  #define VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY  0x04

#define VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH                       0x0084


#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR                        0x000B

/* Result registers */
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS                       0x0013
#define VL53L0X_REG_RESULT_RANGE_STATUS                           0x0014

#define VL53L0X_REG_RESULT_CORE_PAGE  1
#define VL53L0X_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN         0x00BC
#define VL53L0X_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN          0x00C0
#define VL53L0X_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF         0x00D0
#define VL53L0X_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_REF          0x00D4
#define VL53L0X_REG_RESULT_PEAK_SIGNAL_RATE_REF                   0x00B6

/* Algo register */

#define VL53L0X_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM             0x0028

#define VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS                      0x008a

/* Check Limit registers */
#define VL53L0X_REG_MSRC_CONFIG_CONTROL                           0x0060

#define VL53L0X_REG_PRE_RANGE_CONFIG_MIN_SNR                      0X0027
#define VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW              0x0056
#define VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH             0x0057
#define VL53L0X_REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT            0x0064

#define VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_SNR                    0X0067
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW            0x0047
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH           0x0048
#define VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT   0x0044


#define VL53L0X_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI              0X0061
#define VL53L0X_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_LO              0X0062

/* PRE RANGE registers */
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD                 0x0050
#define VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI            0x0051
#define VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO            0x0052

#define VL53L0X_REG_SYSTEM_HISTOGRAM_BIN                          0x0081
#define VL53L0X_REG_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT         0x0033
#define VL53L0X_REG_HISTOGRAM_CONFIG_READOUT_CTRL                 0x0055

#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD               0x0070
#define VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI          0x0071
#define VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO          0x0072
#define VL53L0X_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS         0x0020

#define VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP                    0x0046


#define VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N                   0x00BF
#define VL53L0X_WHO_AM_I                                          0x00C0   // should be 0x40              
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID                       0x00C0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID                    0x00C2

#define VL53L0X_REG_OSC_CALIBRATE_VAL                             0x00F8


#define VL53L0X_SIGMA_ESTIMATE_MAX_VALUE                          65535
/* equivalent to a range sigma of 655.35mm */

#define VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH          0x032

#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0   0x0B0
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_1   0x0B1
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_2   0x0B2
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_3   0x0B3
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_4   0x0B4
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_5   0x0B5

#define VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT   0xB6
#define VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD 0x4E /* 0x14E */
#define VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET    0x4F /* 0x14F */
#define VL53L0X_REG_POWER_MANAGEMENT_GO1_POWER_FORCE    0x80
     #define VL53L0X_POWERMODE_STANDBY        0x00 //set power mode to standby level 1
     #define VL53L0X_POWERMODE_IDLE           0x01 //set power mode to idle level 1


/*
 * Speed of light in um per 1E-10 Seconds
 */

#define VL53L0X_SPEED_OF_LIGHT_IN_AIR 2997

#define VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV     0x0089

#define VL53L0X_REG_ALGO_PHASECAL_LIM                         0x0030 /* 0x130 */
#define VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT              0x0030

// Using the VL53L0X breakout board/Teensy 3.1 Add-On Shield, device address is 0x29
#define VL53L0X_ADDRESS  0x29   //  Device address of VL53L0X  
  
// Pin definitions
int myLed = 13;
int intPin = 9;

bool newData = false;
uint8_t rangeData[14];
uint8_t rawData[4] = {0, 0, 0, 0};
uint32_t lastUpdate = 0, Now = 0;          // used to calculate sample data rate


#define verboseMode  true

void setup()
{
//  Wire.begin();
//  TWBR = 12;  // 400 kbit/sec I2C speed for Pro Mini
  // Setup for Master mode, pins 16/17, external pullups, 400kHz for Teensy 3.1
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(4000);
  Serial.begin(38400);

  // Set up the led indicator
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW);
  pinMode(intPin, INPUT);

  byte HVI2C = readByte(VL53L0X_ADDRESS, VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV);
  writeByte(VL53L0X_ADDRESS, VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, HVI2C | 0x01); // set I2C HIGH to 2.8 V

  I2Cscan();
  
  delay(1000);

  // Read the WHO_AM_I register, this is a good test of communication
  Serial.println("VL53L0X proximity sensor...");
  byte c = readByte(VL53L0X_ADDRESS, VL53L0X_WHO_AM_I);  // Read WHO_AM_I register for VL53L0X
  Serial.print("VL53L0X "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0xEE, HEX);
 
  // Get info about the specific device
  byte revID = readByte(VL53L0X_ADDRESS, VL53L0X_REG_IDENTIFICATION_REVISION_ID);  // Read Revision ID register for VL53L0X
  Serial.print("VL53L0X revision ID 0X"); Serial.println(revID, HEX);  
  delay(1000); 

  if (c == 0xEE) // VL53L0X WHO_AM_I should always be 0xEE
  {  
  
  Serial.println("VL53L0X is online...");

  writeByte(VL53L0X_ADDRESS, VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N, 0x01);  // reset device
  delay(100);

  HVI2C = readByte(VL53L0X_ADDRESS, VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV);
  writeByte(VL53L0X_ADDRESS, VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, HVI2C | 0x01); // set I2C HIGH to 2.8 V

  if ((HVI2C & ~0xFE) == 0x00) Serial.println("I2C IO voltage is 1V8");
  if ((HVI2C & ~0xFE) == 0x01) Serial.println("I2C IO voltage is 2V8");

  // "Set I2C standard mode"
  writeByte(VL53L0X_ADDRESS, 0x88, 0x00);

  writeByte(VL53L0X_ADDRESS, 0x80, 0x01);
  writeByte(VL53L0X_ADDRESS, 0xFF, 0x01);
  writeByte(VL53L0X_ADDRESS, 0x00, 0x00);
  uint8_t stop_variable = readByte(VL53L0X_ADDRESS, 0x91);
  writeByte(VL53L0X_ADDRESS, 0x00, 0x01);
  writeByte(VL53L0X_ADDRESS, 0xFF, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x80, 0x00);


// Taken from Pololu Arduino version of ST's API code

  uint8_t spad_count;
  bool spad_type_is_aperture;
  getSpadInfo(&spad_count, &spad_type_is_aperture);
//  if (!getSpadInfo(&spad_count, &spad_type_is_aperture)) { return false; }

  // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
  // the API, but the same data seems to be more easily readable from
  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
  uint8_t ref_spad_map[6];
  readBytes(VL53L0X_ADDRESS, VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6, ref_spad_map);

  // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

  writeByte(VL53L0X_ADDRESS, 0xFF, 0x01);
  writeByte(VL53L0X_ADDRESS, VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  writeByte(VL53L0X_ADDRESS, VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  writeByte(VL53L0X_ADDRESS, 0xFF, 0x00);
  writeByte(VL53L0X_ADDRESS, VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
  uint8_t spads_enabled = 0;

  for (uint8_t i = 0; i < 48; i++)
  {
    if (i < first_spad_to_enable || spads_enabled == spad_count)
    {
      // This bit is lower than the first one that should be enabled, or
      // (reference_spad_count) bits have already been enabled, so zero this bit
      ref_spad_map[i / 8] &= ~(1 << (i % 8));
    }
    else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
    {
      spads_enabled++;
    }
  }

  writeByte(VL53L0X_ADDRESS, VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map[0]);
  writeByte(VL53L0X_ADDRESS, VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_1, ref_spad_map[1]);
  writeByte(VL53L0X_ADDRESS, VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_2, ref_spad_map[2]);
  writeByte(VL53L0X_ADDRESS, VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_3, ref_spad_map[3]);
  writeByte(VL53L0X_ADDRESS, VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_4, ref_spad_map[4]);
  writeByte(VL53L0X_ADDRESS, VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_5, ref_spad_map[5]);

  // -- VL53L0X_set_reference_spads() end
/*
  // -- VL53L0X_load_tuning_settings() begin
  // DefaultTuningSettings from vl53l0x_tuning.h

  writeByte(VL53L0X_ADDRESS, 0xFF, 0x01);
  writeByte(VL53L0X_ADDRESS, 0x00, 0x00);

  writeByte(VL53L0X_ADDRESS, 0xFF, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x09, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x10, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x11, 0x00);

  writeByte(VL53L0X_ADDRESS, 0x24, 0x01);
  writeByte(VL53L0X_ADDRESS, 0x25, 0xFF);
  writeByte(VL53L0X_ADDRESS, 0x75, 0x00);

  writeByte(VL53L0X_ADDRESS, 0xFF, 0x01);
  writeByte(VL53L0X_ADDRESS, 0x4E, 0x2C);
  writeByte(VL53L0X_ADDRESS, 0x48, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x30, 0x20);

  writeByte(VL53L0X_ADDRESS, 0xFF, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x30, 0x09);
  writeByte(VL53L0X_ADDRESS, 0x54, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x31, 0x04);
  writeByte(VL53L0X_ADDRESS, 0x32, 0x03);
  writeByte(VL53L0X_ADDRESS, 0x40, 0x83);
  writeByte(VL53L0X_ADDRESS, 0x46, 0x25);
  writeByte(VL53L0X_ADDRESS, 0x60, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x27, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x50, 0x06);
  writeByte(VL53L0X_ADDRESS, 0x51, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x52, 0x96);
  writeByte(VL53L0X_ADDRESS, 0x56, 0x08);
  writeByte(VL53L0X_ADDRESS, 0x57, 0x30);
  writeByte(VL53L0X_ADDRESS, 0x61, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x62, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x64, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x65, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x66, 0xA0);

  writeByte(VL53L0X_ADDRESS, 0xFF, 0x01);
  writeByte(VL53L0X_ADDRESS, 0x22, 0x32);
  writeByte(VL53L0X_ADDRESS, 0x47, 0x14);
  writeByte(VL53L0X_ADDRESS, 0x49, 0xFF);
  writeByte(VL53L0X_ADDRESS, 0x4A, 0x00);

  writeByte(VL53L0X_ADDRESS, 0xFF, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x7A, 0x0A);
  writeByte(VL53L0X_ADDRESS, 0x7B, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x78, 0x21);

  writeByte(VL53L0X_ADDRESS, 0xFF, 0x01);
  writeByte(VL53L0X_ADDRESS, 0x23, 0x34);
  writeByte(VL53L0X_ADDRESS, 0x42, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x44, 0xFF);
  writeByte(VL53L0X_ADDRESS, 0x45, 0x26);
  writeByte(VL53L0X_ADDRESS, 0x46, 0x05);
  writeByte(VL53L0X_ADDRESS, 0x40, 0x40);
  writeByte(VL53L0X_ADDRESS, 0x0E, 0x06);
  writeByte(VL53L0X_ADDRESS, 0x20, 0x1A);
  writeByte(VL53L0X_ADDRESS, 0x43, 0x40);

  writeByte(VL53L0X_ADDRESS, 0xFF, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x34, 0x03);
  writeByte(VL53L0X_ADDRESS, 0x35, 0x44);

  writeByte(VL53L0X_ADDRESS, 0xFF, 0x01);
  writeByte(VL53L0X_ADDRESS, 0x31, 0x04);
  writeByte(VL53L0X_ADDRESS, 0x4B, 0x09);
  writeByte(VL53L0X_ADDRESS, 0x4C, 0x05);
  writeByte(VL53L0X_ADDRESS, 0x4D, 0x04);

  writeByte(VL53L0X_ADDRESS, 0xFF, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x44, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x45, 0x20);
  writeByte(VL53L0X_ADDRESS, 0x47, 0x08);
  writeByte(VL53L0X_ADDRESS, 0x48, 0x28);
  writeByte(VL53L0X_ADDRESS, 0x67, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x70, 0x04);
  writeByte(VL53L0X_ADDRESS, 0x71, 0x01);
  writeByte(VL53L0X_ADDRESS, 0x72, 0xFE);
  writeByte(VL53L0X_ADDRESS, 0x76, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x77, 0x00);

  writeByte(VL53L0X_ADDRESS, 0xFF, 0x01);
  writeByte(VL53L0X_ADDRESS, 0x0D, 0x01);

  writeByte(VL53L0X_ADDRESS, 0xFF, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x80, 0x01);
  writeByte(VL53L0X_ADDRESS, 0x01, 0xF8);

  writeByte(VL53L0X_ADDRESS, 0xFF, 0x01);
  writeByte(VL53L0X_ADDRESS, 0x8E, 0x01);
  writeByte(VL53L0X_ADDRESS, 0x00, 0x01);
  writeByte(VL53L0X_ADDRESS, 0xFF, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x80, 0x00);

// -- VL53L0X_load_tuning_settings() end
 */
  
  // Configure GPIO1 for interrupt, active LOW
  byte actHIGH = readByte(VL53L0X_ADDRESS, VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH);
  writeByte(VL53L0X_ADDRESS, VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04); // enable data ready interrupt 
  writeByte(VL53L0X_ADDRESS, VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH, actHIGH & ~0x10); // GPIO1 interrupt active LOW
  writeByte(VL53L0X_ADDRESS, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01); // clear interrupt   

  // Get some basic information about the sensor
  byte val1 = readByte(VL53L0X_ADDRESS, VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD);
  Serial.print("PRE_RANGE_CONFIG_VCSEL_PERIOD="); Serial.println(val1); 
  Serial.print(" decode: "); Serial.println(VL53L0X_decode_vcsel_period(val1));

  val1 = readByte(VL53L0X_ADDRESS, VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD);
  Serial.print("FINAL_RANGE_CONFIG_VCSEL_PERIOD="); Serial.println(val1);
  Serial.print(" decode: "); Serial.println(VL53L0X_decode_vcsel_period(val1));

  readBytes(VL53L0X_ADDRESS, VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD, 4, &rawData[0]);
  uint32_t IMPeriod = (((uint32_t) rawData[0]) << 24 | ((uint32_t) rawData[1]) << 16 | ((uint32_t) rawData[2]) << 8 | rawData[3]);
  Serial.print(" System Intermeasurement period = "); Serial.print(IMPeriod); Serial.println(" ms");
  
  writeByte(VL53L0X_ADDRESS, VL53L0X_REG_SYSRANGE_START, 0x02); // continuous mode and arm next shot

  attachInterrupt(intPin, myinthandler, FALLING);  // define interrupt for GPI01 pin output of VL53L0X

  }
  else
  {
    Serial.print("Could not connect to VL53L0X: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}

void loop()
{    
//  byte intStatus = readByte(VL53L0X_ADDRESS, VL53L0X_REG_RESULT_RANGE_STATUS);// Poll for data ready
//  if(intStatus & 0x01) // poll for data ready
//  {
    
 if(newData) // wait for data ready interrupt  
 {
 newData = false;
 Now = micros();
 Serial.print("data rate = "); Serial.print(1000000./(Now - lastUpdate)); Serial.println(" Hz");
 writeByte(VL53L0X_ADDRESS, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01); // clear interrupt   

 readBytes(VL53L0X_ADDRESS, VL53L0X_REG_RESULT_RANGE_STATUS, 14, &rangeData[0]); // continuous ranging

 Serial.print("byte 1 = "); Serial.println(rangeData[0]);
 Serial.print("byte 2 = "); Serial.println(rangeData[1]);
 Serial.print("byte 3 = "); Serial.println(rangeData[2]);
 Serial.print("byte 4 = "); Serial.println(rangeData[3]);
 Serial.print("byte 5 = "); Serial.println(rangeData[4]);
 Serial.print("byte 6 = "); Serial.println(rangeData[5]);
 Serial.print("byte 7 = "); Serial.println(rangeData[6]);
 Serial.print("byte 8 = "); Serial.println(rangeData[7]);
 Serial.print("byte 9 = "); Serial.println(rangeData[8]);
 Serial.print("byte 10 = "); Serial.println(rangeData[9]);
 Serial.print("byte 11 = "); Serial.println(rangeData[10]);
 Serial.print("byte 12 = "); Serial.println(rangeData[11]);
 Serial.print("byte 13 = "); Serial.println(rangeData[12]);
 Serial.print("byte 14 = "); Serial.println(rangeData[13]);
 Serial.println("  ");

  byte devError = (rangeData[0] & 0x78) >> 3; // Check for errors
 
  if(devError == 0)     Serial.println("Data OK!");// No device error
  if(devError == 0x01)  Serial.println("VCSEL CONTINUITY TEST FAILURE!");
  if(devError == 0x02)  Serial.println("VCSEL WATCHDOG TEST FAILURE!");
  if(devError == 0x03)  Serial.println("NO VHV VALUE FOUND!");
  if(devError == 0x04)  Serial.println("MSRC NO TARGET!");
  if(devError == 0x05)  Serial.println("SNR CHECK!");
  if(devError == 0x06)  Serial.println("RANGE PHASE CHECK!");
  if(devError == 0x07)  Serial.println("SIGMA THRESHOLD CHECK!");
  if(devError == 0x08)  Serial.println("TCC!");
  if(devError == 0x09)  Serial.println("PHASE CONSISTENCY!");
  if(devError == 0x0A)  Serial.println("MIN CLIP!");
  if(devError == 0x0B)  Serial.println("RANGE COMPLETE!");
  if(devError == 0x0C)  Serial.println("ALGO UNDERFLOW!");
  if(devError == 0x0D)  Serial.println("ALGO OVERFLOW!");
  if(devError == 0x0E)  Serial.println("RANGE IGNORE THRESHOLD!");
  
  Serial.print("Effective SPAD Return Count = "); Serial.println( ((float) (rangeData[2]) + (float)rangeData[3]/255.), 2); // 8.8 format
  Serial.print("Signal Rate = "); Serial.print( (uint16_t) ((uint16_t) rangeData[6] << 8) | rangeData[7]); Serial.println(" mega counts per second");
  Serial.print("Ambient Rate = "); Serial.print( (uint16_t) ((uint16_t) rangeData[8] << 8) | rangeData[9]); Serial.println(" mega counts per second");
  Serial.print("Distance = "); Serial.print( (uint16_t) ((uint16_t) rangeData[10] << 8) | rangeData[11]); Serial.println(" mm");

   lastUpdate = Now;
   
   digitalWrite(myLed, !digitalRead(myLed));
    }
}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void myinthandler()
{
  newData = true;
}

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
bool getSpadInfo(uint8_t * count, bool * type_is_aperture)
{
  uint8_t tmp;

  writeByte(VL53L0X_ADDRESS, 0x80, 0x01);
  writeByte(VL53L0X_ADDRESS, 0xFF, 0x01);
  writeByte(VL53L0X_ADDRESS, 0x00, 0x00);

  writeByte(VL53L0X_ADDRESS, 0xFF, 0x06);
  writeByte(VL53L0X_ADDRESS, 0x83, readByte(VL53L0X_ADDRESS, 0x83) | 0x04);
  writeByte(VL53L0X_ADDRESS, 0xFF, 0x07);
  writeByte(VL53L0X_ADDRESS, 0x81, 0x01);

  writeByte(VL53L0X_ADDRESS, 0x80, 0x01);

  writeByte(VL53L0X_ADDRESS, 0x94, 0x6b);
  writeByte(VL53L0X_ADDRESS, 0x83, 0x00);
  startTimeout();
  while (readByte(VL53L0X_ADDRESS, 0x83) == 0x00)
  {
    if (checkTimeoutExpired()) { return false; }
  }
  writeByte(VL53L0X_ADDRESS, 0x83, 0x01);
  tmp = readByte(VL53L0X_ADDRESS, 0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  writeByte(VL53L0X_ADDRESS, 0x81, 0x00);
  writeByte(VL53L0X_ADDRESS, 0xFF, 0x06);
  writeByte(VL53L0X_ADDRESS, 0x83, readByte(VL53L0X_ADDRESS,  0x83  & ~0x04));
  writeByte(VL53L0X_ADDRESS, 0xFF, 0x01);
  writeByte(VL53L0X_ADDRESS, 0x00, 0x01);

  writeByte(VL53L0X_ADDRESS, 0xFF, 0x00);
  writeByte(VL53L0X_ADDRESS, 0x80, 0x00);

  return true;
}

uint16_t VL53L0X_decode_vcsel_period(short vcsel_period_reg) {
  // Converts the encoded VCSEL period register value into the real
  // period in PLL clocks
  uint16_t vcsel_period_pclks = (vcsel_period_reg + 1) << 1;
  return vcsel_period_pclks;
}

// I2C scan function
void I2Cscan()
{
// scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
    
}

// I2C read/write functions for the VL53L0X sensor
// VL53L0X has 16-bit register addresses, so the MSB of the register is sent first, then the LSB

void writeByte(uint8_t address, uint16_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);      // Initialize the Tx buffer
//	Wire.write((subAddress >> 8) & 0xFF); // Put MSB of 16-bit slave register address in Tx buffer
	Wire.write(subAddress & 0xFF);        // Put LSB of 16-bit slave register address in Tx buffer
	Wire.write(data);                     // Put data in Tx buffer
	Wire.endTransmission();               // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint16_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
//	Wire.write((subAddress >> 8) & 0xFF);    // Put MSB of 16-bit slave register address in Tx buffer
	Wire.write(subAddress & 0xFF);           // Put LSB of 16-bit slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.requestFrom(address, 1);  // Read one byte from slave register address 
	Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint16_t subAddress, uint8_t count, uint8_t * dest)
{  
	Wire.beginTransmission(address);      // Initialize the Tx buffer
//	Wire.write((subAddress >> 8) & 0xFF); // Put MSB of 16-bit slave register address in Tx buffer
	Wire.write(subAddress & 0xFF);        // Put LSB of 16-bit slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);     // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);          // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
//        Wire.requestFrom(address, count);   // Read bytes from slave register address 
        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
	while (Wire.available()) {
        delay(1);
        dest[i++] = Wire.read(); }            // Put read results in the Rx buffer
}
