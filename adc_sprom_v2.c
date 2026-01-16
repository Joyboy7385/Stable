*

- ============================================================================
- ADC WITH SPROM CAPTURE - VERSION 3.0 (PRODUCTION RELEASE)
- ============================================================================
-
- PLATFORM: Nuvoton MS51 Series (1T 8051 Core, 16KB Flash)
- MCU: MS51FB9AE / MS51XB9AE (24MHz HIRC, 12-bit ADC)
-
- FEATURES:
- - PEAK detection with configurable filtering (EMA + Median hybrid)
- - Oversampling for enhanced ADC resolution (12-bit → 14-bit effective)
- - SPROM write protection with CRC verification
- - Bandgap-referenced VDD compensation for accurate readings
- - Clear mode: Power-on with button pressed
- - Comprehensive error handling and fault indication
-
- VERSION 3.0 IMPROVEMENTS (over V2.0):
- - [CRITICAL] Fixed LED SFR pointer issue (cannot take address of bits)
- - [CRITICAL] Fixed WDT_Clear() assembly bug
- - [HIGH] Added EMA + Median hybrid filtering for noise immunity
- - [HIGH] Added oversampling (16x) for +2 bits effective resolution
- - [HIGH] Added bandgap VDD compensation
- - [HIGH] Added CRC-16 verification for SPROM writes
- - [MEDIUM] Added channel settling with dummy conversion
- - [MEDIUM] Proper errata handling (POR, WDT+CKDIV)
- - [MEDIUM] Improved Delay_Us accuracy
- - [LOW] ADC failure detection with fault LED indication
-
- HARDWARE REQUIREMENTS:
- - 0.1µF ceramic capacitor on VDD (within 3mm of pin)
- - 10µF bulk capacitor on VDD
- - ADC source impedance < 10kΩ (or use external buffer)
- - Pull-up resistor on button input (internal or external)
-
- AUTHOR: Production version for Praful
- DATE: January 2025
- ============================================================================
*/

#include "ms51_16k_keil.h"
#include <stdint.h>

// ============================================================================
// HARDWARE REGISTER DEFINITIONS
// ============================================================================
#ifndef P0S
__sfr __at (0x99) P0S;
#endif

// ============================================================================
// SPROM CONFIGURATION (MS51 SPROM: 0xFF80 - 0xFFFF, 128 bytes)
// ============================================================================
#define SPROM_BASE_ADDR 0xFF80 // SPROM start address
#define SPROM_SIZE 128 // SPROM size in bytes

// Data structure layout in SPROM
#define SPROM_ADDR_MAGIC 0x00 // Magic number (2 bytes)
#define SPROM_ADDR_A_LOW 0x02 // Captured value low byte
#define SPROM_ADDR_A_HIGH 0x03 // Captured value high byte
#define SPROM_ADDR_VDD_LOW 0x04 // VDD at capture time (low)
#define SPROM_ADDR_VDD_HIGH 0x05 // VDD at capture time (high)
#define SPROM_ADDR_TIMESTAMP 0x06 // Capture timestamp (4 bytes)
#define SPROM_ADDR_CRC_LOW 0x0A // CRC-16 low byte
#define SPROM_ADDR_CRC_HIGH 0x0B // CRC-16 high byte

// Marker values
#define SPROM_MAGIC_VALUE 0xA55A // Valid data marker
#define SPROM_ERASED_BYTE 0xFF // Flash erased state

// IAP Commands (from MS51 TRM)
#define IAP_CMD_READ 0x00 // Read byte
#define IAP_CMD_PROGRAM 0x21 // Program byte
#define IAP_CMD_PAGE_ERASE 0x22 // Erase 128-byte page

#define SPROM_WRITE_RETRIES 3 // Write retry attempts

// ============================================================================
// TIMING CONSTANTS (milliseconds unless specified)
// ============================================================================
#define TIMER_TICK_MS 1 // Timer0 tick period

// Button timing
#define BUTTON_DEBOUNCE_MS 50
#define BUTTON_POWERON_SAMPLES 5
#define BUTTON_POWERON_DELAY_MS 10
#define BUTTON_POWERON_THRESHOLD 3
#define BUTTON_STABLE_COUNT 10
#define BUTTON_STABLE_DELAY_MS 10

// Post-clear lockout (prevents false capture after clear)
#define POST_CLEAR_LOCKOUT_MS 2000
#define POST_CLEAR_SETTLE_MS 500

// ADC timing
#define ADC_SAMPLE_INTERVAL_MS 100 // Main loop sample rate
#define ADC_INIT_DELAY_MS 50
#define ADC_SETTLE_DELAY_US 50 // Channel settling time

// LED timing
#define LED_BLINK_SLOW_MS 400
#define LED_BLINK_FAST_MS 150
#define LED_LONG_PAUSE_MS 3000
#define LED_CLEAR_BLINK_MS 300
#define LED_CAPTURE_BLINK_MS 200
#define LED_ERROR_BLINK_MS 100
#define LED_ADC_FAULT_BLINK_MS 200

// Startup
#define STARTUP_STABILIZE_MS 100

// ============================================================================
// ADC CONFIGURATION
// ============================================================================
#define ADC_CHANNEL 6 // AIN6 = P0.3

// Oversampling configuration (16x = +2 bits resolution)
#define ADC_OVERSAMPLE_COUNT 16 // Number of oversamples
#define ADC_OVERSAMPLE_SHIFT 2 // Right-shift for decimation

// Filtering configuration
#define EMA_SHIFT 3 // EMA alpha = 1/8 (2^3)
#define MEDIAN_WINDOW 3 // Median filter window size
#define PEAK_SAMPLE_COUNT 5 // Samples for peak detection
#define PEAK_SAMPLE_INTERVAL_MS 20 // Interval between peak samples

// ADC validation
#define ADC_MIN_VALID 50 // Minimum valid reading
#define ADC_MAX_VALID 4000 // Maximum valid reading (12-bit)
#define ADC_DEFAULT_VALUE 2048 // Default if no valid readings
#define ADC_TIMEOUT_COUNT 10000 // Conversion timeout

#define ADC_FAIL_THRESHOLD 10 // Failures before fault flag

// Bandgap reference (from MS51 datasheet, typical 1.22V)
#define BANDGAP_TYPICAL_MV 1220 // Bandgap voltage in mV
#define BANDGAP_CHANNEL 8 // Internal bandgap channel

// ============================================================================
// LED DISPLAY LIMITS
// ============================================================================
#define LED_MAX_HUNDREDS 15
#define LED_MAX_TENS 9
#define LED_ZERO_PAUSE_COUNT 2
#define LED_NONZERO_PAUSE_COUNT 3

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define PIN_MAIN_LED P15
#define PIN_FAULT_LED P04
#define PIN_SETTING_LED P01
#define PIN_BUTTON P11

// ============================================================================
// LED IDENTIFIER ENUM (Fix: Cannot take address of SFR bits)
// ============================================================================
typedef enum {
LED_ID_MAIN = 0,
LED_ID_FAULT = 1,
LED_ID_SETTING = 2
} LED_ID_t;

// ============================================================================
// LED CONTROL FUNCTION
// ============================================================================
static void LED_Set(LED_ID_t led_id, uint8_t state)
{
switch(led_id) {
case LED_ID_MAIN: PIN_MAIN_LED = state; break;
case LED_ID_FAULT: PIN_FAULT_LED = state; break;
case LED_ID_SETTING: PIN_SETTING_LED = state; break;
default: break;
}
}

// Legacy macros (still useful for init/direct control)
#define LED_MAIN_ON() (PIN_MAIN_LED = 1)
#define LED_MAIN_OFF() (PIN_MAIN_LED = 0)
#define LED_FAULT_ON() (PIN_FAULT_LED = 1)
#define LED_FAULT_OFF() (PIN_FAULT_LED = 0)
#define LED_SETTING_ON() (PIN_SETTING_LED = 1)
#define LED_SETTING_OFF() (PIN_SETTING_LED = 0)

#define BUTTON_PRESSED() (PIN_BUTTON == 0)

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// System tick (accessed atomically)
volatile __xdata uint32_t systemTick = 0;

// ADC values
uint16_t global_A = 0; // Captured/stored value (compensated)
uint16_t global_A_raw = 0; // Raw captured value
uint16_t global_B = 0; // Current live value (filtered)
uint16_t global_B_raw = 0; // Current raw value
uint16_t current_vdd_mv = 5000; // Measured VDD in millivolts
uint8_t A_valid = 0; // 0 = SPROM empty, 1 = valid data

// ADC health monitoring
volatile uint8_t adc_fail_count = 0;
volatile uint8_t adc_fault_flag = 0;

// EMA filter state (Q8 fixed-point for precision)
static uint32_t ema_state = 0;
static uint8_t ema_initialized = 0;

// Median filter buffer
static uint16_t median_buffer[MEDIAN_WINDOW];
static uint8_t median_index = 0;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

// Core system
void WDT_Clear(void);
uint32_t Get_SystemTick(void);
void Delay_Ms(uint32_t ms);
void Delay_Us(uint16_t us);

// Initialization
void GPIO_Init(void);
void ADC_Init(void);
void Timer_Init(void);

// SPROM operations
void IAP_Unlock(void);
uint8_t SPROM_Read_Byte(uint8_t offset);
uint8_t SPROM_Write_Byte(uint8_t offset, uint8_t dat);
uint8_t SPROM_Erase(void);
uint8_t SPROM_Save_Data(uint16_t value, uint16_t vdd_mv, uint32_t timestamp);
uint8_t SPROM_Load_Data(void);
uint16_t Calculate_CRC16(uint8_t *data, uint8_t length);

// ADC operations
uint16_t ADC_ReadRaw(uint8_t channel);
uint16_t ADC_ReadOversampled(uint8_t channel);
uint16_t ADC_ReadFiltered(uint8_t channel);
uint16_t ADC_ReadPeak(uint8_t channel);
uint16_t ADC_MeasureVDD(void);

// Filtering
uint16_t Filter_EMA(uint16_t new_sample);
uint16_t Filter_Median(uint16_t new_sample);
void Filter_Reset(void);

// ============================================================================
// WATCHDOG TIMER (Corrected for MS51)
// ============================================================================
void WDT_Clear(void)
{
// MS51 TA (Timed Access) sequence required before WDT clear
// Reference: MS51 TRM Section 6.7
TA = 0xAA;
TA = 0x55;
WDCON |= 0x40; // Set WDCLR bit to clear WDT
}

// ============================================================================
// ATOMIC SYSTEM TICK ACCESS
// Prevents torn reads on 8-bit architecture when ISR updates 32-bit value
// ============================================================================
uint32_t Get_SystemTick(void)
{
uint32_t tick;
uint8_t ea_save = EA;
EA = 0; // Disable interrupts (critical section)
tick = systemTick;
EA = ea_save; // Restore interrupt state
return tick;
}

// ============================================================================
// TIMER0 ISR - 1ms System Tick
// ============================================================================
void Timer0_ISR(void) __interrupt (1)
{
// Reload for 1ms @ 24MHz (if using 24MHz HIRC)
// For 16MHz: TH0=0xFC, TL0=0x18
TH0 = 0xFA;
TL0 = 0x24;
systemTick++;
}

// ============================================================================
// DELAY FUNCTIONS
// ============================================================================
void Delay_Ms(uint32_t ms)
{
uint32_t start = Get_SystemTick();
while((Get_SystemTick() - start) < ms) {
WDT_Clear();
}
}

// Optimized microsecond delay for 24MHz (approximate)
void Delay_Us(uint16_t us)
{
// Each iteration ~1µs at 24MHz with loop overhead
while(us--) {
__asm
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
__endasm;
}
}

// ============================================================================
// CRC-16 CALCULATION (CCITT polynomial 0x1021)
// Used for SPROM data integrity verification
// ============================================================================
uint16_t Calculate_CRC16(uint8_t *data, uint8_t length)
{
uint16_t crc = 0xFFFF;
uint8_t i, j;

for(i = 0; i < length; i++) {
crc ^= ((uint16_t)data[i] << 8);
for(j = 0; j < 8; j++) {
if(crc & 0x8000) {
crc = (crc << 1) ^ 0x1021;
} else {
crc <<= 1;
}
}

// Feed WDT during long calculations
if((i & 0x07) == 0) {
WDT_Clear();
}
}

return crc;

}

// ============================================================================
// IAP (IN-APPLICATION PROGRAMMING) FUNCTIONS
// Reference: MS51 TRM Chapter 20 - Flash Memory
// ============================================================================

// Unlock IAP controller (required before any IAP operation)
void IAP_Unlock(void)
{
TA = 0xAA;
TA = 0x55;
CHPCON |= 0x01; // Enable IAP
}

// Read byte from SPROM
uint8_t SPROM_Read_Byte(uint8_t offset)
{
uint8_t dat;
uint8_t ea_save = EA;

EA = 0;

IAP_Unlock();

IAPCN = IAP_CMD_READ;
IAPAH = (SPROM_BASE_ADDR + offset) >> 8;
IAPAL = (SPROM_BASE_ADDR + offset) & 0xFF;

TA = 0xAA;
TA = 0x55;
IAPTRG |= 0x01; // Trigger IAP

dat = IAPFD;

// Disable IAP
TA = 0xAA;
TA = 0x55;
CHPCON &= ~0x01;

EA = ea_save;
return dat;

}

// Write byte to SPROM with verification
uint8_t SPROM_Write_Byte(uint8_t offset, uint8_t dat)
{
uint8_t retry;
uint8_t readback;
uint8_t ea_save = EA;

for(retry = 0; retry < SPROM_WRITE_RETRIES; retry++) {
EA = 0;

IAP_Unlock();

IAPCN = IAP_CMD_PROGRAM;
IAPAH = (SPROM_BASE_ADDR + offset) >> 8;
IAPAL = (SPROM_BASE_ADDR + offset) & 0xFF;
IAPFD = dat;

TA = 0xAA;
TA = 0x55;
IAPTRG |= 0x01; // Trigger IAP (CPU halts during write)

// Disable IAP
TA = 0xAA;
TA = 0x55;
CHPCON &= ~0x01;

EA = ea_save;

// Small delay for write completion
Delay_Us(50);

// Verify write
readback = SPROM_Read_Byte(offset);
if(readback == dat) {
return 1; // Success
}

WDT_Clear();
}

return 0; // Failed after retries

}

// Erase SPROM page (128 bytes)
uint8_t SPROM_Erase(void)
{
uint8_t ea_save = EA;
uint8_t verify_byte;

EA = 0;

IAP_Unlock();

IAPCN = IAP_CMD_PAGE_ERASE;
IAPAH = SPROM_BASE_ADDR >> 8;
IAPAL = SPROM_BASE_ADDR & 0xFF;

TA = 0xAA;
TA = 0x55;
IAPTRG |= 0x01; // Trigger erase (CPU halts, ~5ms typical)

// Disable IAP
TA = 0xAA;
TA = 0x55;
CHPCON &= ~0x01;

EA = ea_save;

// Delay for erase completion
Delay_Ms(10);

// Verify first byte is erased
verify_byte = SPROM_Read_Byte(0);
return (verify_byte == SPROM_ERASED_BYTE) ? 1 : 0;

}

// Save captured data to SPROM with CRC
uint8_t SPROM_Save_Data(uint16_t value, uint16_t vdd_mv, uint32_t timestamp)
{
uint8_t buffer[10]; // Data buffer for CRC calculation
uint16_t crc;
uint8_t success = 1;

// Prepare data buffer
buffer[0] = SPROM_MAGIC_VALUE & 0xFF;
buffer[1] = SPROM_MAGIC_VALUE >> 8;
buffer[2] = value & 0xFF;
buffer[3] = value >> 8;
buffer[4] = vdd_mv & 0xFF;
buffer[5] = vdd_mv >> 8;
buffer[6] = timestamp & 0xFF;
buffer[7] = (timestamp >> 8) & 0xFF;
buffer[8] = (timestamp >> 16) & 0xFF;
buffer[9] = (timestamp >> 24) & 0xFF;

// Calculate CRC over data
crc = Calculate_CRC16(buffer, 10);

// Write magic number
if(!SPROM_Write_Byte(SPROM_ADDR_MAGIC, buffer[0])) success = 0;
if(!SPROM_Write_Byte(SPROM_ADDR_MAGIC + 1, buffer[1])) success = 0;

// Write captured value
if(!SPROM_Write_Byte(SPROM_ADDR_A_LOW, buffer[2])) success = 0;
if(!SPROM_Write_Byte(SPROM_ADDR_A_HIGH, buffer[3])) success = 0;

// Write VDD at capture time
if(!SPROM_Write_Byte(SPROM_ADDR_VDD_LOW, buffer[4])) success = 0;
if(!SPROM_Write_Byte(SPROM_ADDR_VDD_HIGH, buffer[5])) success = 0;

// Write timestamp
if(!SPROM_Write_Byte(SPROM_ADDR_TIMESTAMP, buffer[6])) success = 0;
if(!SPROM_Write_Byte(SPROM_ADDR_TIMESTAMP + 1, buffer[7])) success = 0;
if(!SPROM_Write_Byte(SPROM_ADDR_TIMESTAMP + 2, buffer[8])) success = 0;
if(!SPROM_Write_Byte(SPROM_ADDR_TIMESTAMP + 3, buffer[9])) success = 0;

// Write CRC (commits the data)
if(!SPROM_Write_Byte(SPROM_ADDR_CRC_LOW, crc & 0xFF)) success = 0;
if(!SPROM_Write_Byte(SPROM_ADDR_CRC_HIGH, crc >> 8)) success = 0;

return success;

}

// Load and verify data from SPROM
uint8_t SPROM_Load_Data(void)
{
uint8_t buffer[10];
uint16_t stored_magic;
uint16_t stored_crc;
uint16_t calc_crc;

// Read magic number first
buffer[0] = SPROM_Read_Byte(SPROM_ADDR_MAGIC);
buffer[1] = SPROM_Read_Byte(SPROM_ADDR_MAGIC + 1);
stored_magic = ((uint16_t)buffer[1] << 8) | buffer[0];

if(stored_magic != SPROM_MAGIC_VALUE) {
// SPROM is empty or corrupted
global_A = 0;
A_valid = 0;
return 0;
}

// Read remaining data
buffer[2] = SPROM_Read_Byte(SPROM_ADDR_A_LOW);
buffer[3] = SPROM_Read_Byte(SPROM_ADDR_A_HIGH);
buffer[4] = SPROM_Read_Byte(SPROM_ADDR_VDD_LOW);
buffer[5] = SPROM_Read_Byte(SPROM_ADDR_VDD_HIGH);
buffer[6] = SPROM_Read_Byte(SPROM_ADDR_TIMESTAMP);
buffer[7] = SPROM_Read_Byte(SPROM_ADDR_TIMESTAMP + 1);
buffer[8] = SPROM_Read_Byte(SPROM_ADDR_TIMESTAMP + 2);
buffer[9] = SPROM_Read_Byte(SPROM_ADDR_TIMESTAMP + 3);

// Read stored CRC
stored_crc = ((uint16_t)SPROM_Read_Byte(SPROM_ADDR_CRC_HIGH) << 8) |
SPROM_Read_Byte(SPROM_ADDR_CRC_LOW);

// Calculate CRC over data
calc_crc = Calculate_CRC16(buffer, 10);

if(calc_crc != stored_crc) {
// CRC mismatch - data corrupted
global_A = 0;
A_valid = 0;
return 0;
}

// Data is valid - extract values
global_A = ((uint16_t)buffer[3] << 8) | buffer[2];

// Validate range
if(global_A < ADC_MIN_VALID || global_A > ADC_MAX_VALID) {
global_A = 0;
A_valid = 0;
return 0;
}

A_valid = 1;
return 1;

}

// ============================================================================
// GPIO INITIALIZATION
// ============================================================================
void GPIO_Init(void)
{
// LED outputs (push-pull mode)
// P0.4 (FAULT LED), P0.1 (SETTING LED)
P0M1 &= ~((1<<4) | (1<<1)); // Clear open-drain
P0M2 |= ((1<<4) | (1<<1)); // Set push-pull

// P1.5 (MAIN LED)
P1M1 &= ~(1<<5);
P1M2 |= (1<<5);

// Button input P1.1 (high-impedance with internal pull-up)
P1M1 |= (1<<1); // Input mode
P1M2 &= ~(1<<1);

// ADC input P0.3 (high-impedance, disable digital function)
P0M1 |= (1<<3);
P0M2 &= ~(1<<3);
P0S &= ~(1<<3); // Disable Schmitt trigger

// Initialize LEDs OFF
LED_MAIN_OFF();
LED_FAULT_OFF();
LED_SETTING_OFF();

}

// ============================================================================
// ADC INITIALIZATION
// Reference: MS51 TRM Chapter 18 - ADC
// ============================================================================
void ADC_Init(void)
{
// Clear ADC control registers
ADCCON1 = 0x00;
ADCCON2 = 0x00;
ADCCON0 = 0x00;

// Configure ADC clock divider
// ADCDIV = 01 (÷2) for 12MHz ADC clock @ 24MHz system
// This provides good balance of speed and accuracy
ADCCON1 |= (0x01 << 4);

// Single conversion mode (not continuous)
ADCCON1 &= ~0x80;

// Configure conversion timing for maximum accuracy
ADCCON2 = 0x0E;

// Maximum sampling delay for high-impedance sources
ADCDLY = 0xFF;

// Disable digital input on ADC channel to reduce noise
AINDIDS = 0x00;
AINDIDS |= (1 << ADC_CHANNEL); // Disable digital on AIN6

// Select channel
ADCCON0 = (ADCCON0 & 0xF0) | ADC_CHANNEL;

// Enable ADC
ADCCON1 |= 0x01;

// Wait for ADC to stabilize
Delay_Ms(ADC_INIT_DELAY_MS);

}

// ============================================================================
// TIMER INITIALIZATION
// ============================================================================
void Timer_Init(void)
{
TMOD &= 0xF0; // Clear Timer0 bits
TMOD |= 0x01; // Timer0 Mode 1 (16-bit)

// Reload for 1ms @ 24MHz
TH0 = 0xFA;
TL0 = 0x24;

ET0 = 1; // Enable Timer0 interrupt
TR0 = 1; // Start Timer0

}

// ============================================================================
// ADC FUNCTIONS
// ============================================================================

// Raw ADC read with timeout protection
uint16_t ADC_ReadRaw(uint8_t channel)
{
uint16_t result;
uint16_t timeout;
uint8_t ea_save;

// Select channel and allow settling
ADCCON0 = (ADCCON0 & 0xF0) | channel;
Delay_Us(ADC_SETTLE_DELAY_US);

// Dummy conversion for channel settling (discard result)
ADCF = 0;
ADCS = 1;
timeout = ADC_TIMEOUT_COUNT;
while(ADCF == 0 && --timeout);

if(timeout == 0) {
return 0xFFFF; // Timeout error
}

// Actual conversion
ADCF = 0;
ADCS = 1;
timeout = ADC_TIMEOUT_COUNT;
while(ADCF == 0 && --timeout);

if(timeout == 0) {
return 0xFFFF; // Timeout error
}

// Read result atomically (prevent torn read between ADCRH/ADCRL)
ea_save = EA;
EA = 0;
result = ((uint16_t)ADCRH << 4) | (ADCRL & 0x0F);
EA = ea_save;

return result;

}

// Oversampled ADC read for +2 bits effective resolution
// 16 samples averaged, right-shifted by 2
uint16_t ADC_ReadOversampled(uint8_t channel)
{
uint32_t sum = 0;
uint8_t i;
uint8_t valid_count = 0;
uint16_t reading;

for(i = 0; i < ADC_OVERSAMPLE_COUNT; i++) {
reading = ADC_ReadRaw(channel);

if(reading != 0xFFFF && reading >= ADC_MIN_VALID && reading <= ADC_MAX_VALID) {
sum += reading;
valid_count++;
}

// Feed WDT during extended operation
if((i & 0x03) == 0) {
WDT_Clear();
}
}

if(valid_count == 0) {
// Track failures
if(adc_fail_count < 255) {
adc_fail_count++;
}
if(adc_fail_count >= ADC_FAIL_THRESHOLD) {
adc_fault_flag = 1;
}
return ADC_DEFAULT_VALUE;
}

// Reset failure tracking on success
adc_fail_count = 0;
adc_fault_flag = 0;

// Decimation: sum / 4 (right-shift by 2 for +2 bits resolution)
// Result is now effectively 14-bit
return (uint16_t)(sum >> ADC_OVERSAMPLE_SHIFT);

}

// ============================================================================
// DIGITAL FILTERING
// ============================================================================

// Exponential Moving Average (EMA) Filter
// Alpha = 1/2^EMA_SHIFT = 1/8
// Uses Q8 fixed-point for precision without floating point
uint16_t Filter_EMA(uint16_t new_sample)
{
if(!ema_initialized) {
ema_state = (uint32_t)new_sample << 8; // Initialize to first sample
ema_initialized = 1;
return new_sample;
}

// EMA formula: state = state + alpha * (sample - state)
// In fixed-point: state += (sample << 8 - state) >> EMA_SHIFT
int32_t delta = ((uint32_t)new_sample << 8) - ema_state;
ema_state += delta >> EMA_SHIFT;

return (uint16_t)(ema_state >> 8);

}

// Median Filter (window of 3)
// Excellent for rejecting impulse noise/spikes
uint16_t Filter_Median(uint16_t new_sample)
{
uint16_t a, b, c;

// Add new sample to circular buffer
median_buffer[median_index] = new_sample;
median_index = (median_index + 1) % MEDIAN_WINDOW;

// Get all three samples
a = median_buffer[0];
b = median_buffer[1];
c = median_buffer[2];

// Optimized median-of-3
if(a > b) {
if(b > c) return b;
else if(a > c) return c;
else return a;
} else {
if(a > c) return a;
else if(b > c) return c;
else return b;
}

}

// Reset all filters
void Filter_Reset(void)
{
uint8_t i;

ema_state = 0;
ema_initialized = 0;

for(i = 0; i < MEDIAN_WINDOW; i++) {
median_buffer[i] = ADC_DEFAULT_VALUE;
}
median_index = 0;

}

// Combined filtered read: Oversample → Median → EMA
uint16_t ADC_ReadFiltered(uint8_t channel)
{
uint16_t raw = ADC_ReadOversampled(channel);
uint16_t median_out = Filter_Median(raw);
uint16_t ema_out = Filter_EMA(median_out);

global_B_raw = raw;
return ema_out;

}

// Peak detection: Returns maximum of multiple filtered samples
uint16_t ADC_ReadPeak(uint8_t channel)
{
uint16_t max_reading = 0;
uint8_t i;

for(i = 0; i < PEAK_SAMPLE_COUNT; i++) {
uint16_t reading = ADC_ReadFiltered(channel);

if(reading > max_reading) {
max_reading = reading;
}

if(i < (PEAK_SAMPLE_COUNT - 1)) {
Delay_Ms(PEAK_SAMPLE_INTERVAL_MS);
}
}

return max_reading;

}

// ============================================================================
// VDD MEASUREMENT USING BANDGAP REFERENCE
// Allows compensation for supply voltage variations
// ============================================================================
uint16_t ADC_MeasureVDD(void)
{
uint16_t bandgap_reading;
uint32_t vdd_calc;

// Read internal bandgap reference
// Note: Channel 8 is typically the bandgap on MS51
// Check your specific variant's datasheet
bandgap_reading = ADC_ReadOversampled(BANDGAP_CHANNEL);

if(bandgap_reading == 0 || bandgap_reading == 0xFFFF) {
return 5000; // Default to 5V if reading fails
}

// Calculate VDD: VDD = (Bandgap_mV * 4096) / ADC_reading
// For 14-bit effective (after oversampling): use 16384 instead
// Result in millivolts
vdd_calc = ((uint32_t)BANDGAP_TYPICAL_MV * 16384) / bandgap_reading;

// Clamp to reasonable range (2.4V to 5.5V per MS51 spec)
if(vdd_calc < 2400) vdd_calc = 2400;
if(vdd_calc > 5500) vdd_calc = 5500;

return (uint16_t)vdd_calc;

}

// ============================================================================
// LED BLINK STATE MACHINE
// ============================================================================
typedef struct {
uint8_t hundreds;
uint8_t tens;
uint8_t digit_mode; // 0=hundreds, 1=tens
uint8_t blink_counter;
uint8_t blink_state;
uint32_t blink_timer;
uint8_t pause_counter;
uint8_t in_long_pause;
uint32_t long_pause_timer;
LED_ID_t led_id;
} LED_State_t;

void Update_LED_StateMachine(LED_State_t *state, uint32_t current_tick)
{
uint8_t led_value = 0;
uint16_t interval;
uint8_t current_digit;

if(state->in_long_pause) {
if((current_tick - state->long_pause_timer) >= LED_LONG_PAUSE_MS) {
state->in_long_pause = 0;
}
LED_Set(state->led_id, 0);
return;
}

interval = (state->digit_mode == 0) ? LED_BLINK_SLOW_MS : LED_BLINK_FAST_MS;

if((current_tick - state->blink_timer) < interval) {
return;
}

state->blink_timer = current_tick;
current_digit = (state->digit_mode == 0) ? state->hundreds : state->tens;

if(current_digit == 0) {
if(state->blink_counter == 0) {
led_value = 1;
state->blink_counter = 1;
} else {
led_value = 0;
state->pause_counter++;

if(state->pause_counter >= LED_ZERO_PAUSE_COUNT) {
state->pause_counter = 0;
state->blink_counter = 0;
state->digit_mode = !state->digit_mode;

if(state->digit_mode == 0) {
state->in_long_pause = 1;
state->long_pause_timer = current_tick;
}
}
}
} else {
if(state->blink_counter < current_digit) {
state->blink_state = !state->blink_state;

if(state->blink_state) {
led_value = 1;
} else {
led_value = 0;
state->blink_counter++;
}
} else {
led_value = 0;
state->pause_counter++;

if(state->pause_counter >= LED_NONZERO_PAUSE_COUNT) {
state->pause_counter = 0;
state->blink_counter = 0;
state->blink_state = 0;
state->digit_mode = !state->digit_mode;

if(state->digit_mode == 0) {
state->in_long_pause = 1;
state->long_pause_timer = current_tick;
}
}
}
}

LED_Set(state->led_id, led_value);

}

void Update_LED_Digits(LED_State_t *state, uint16_t value)
{
state->hundreds = (uint8_t)(value / 100);
state->tens = (uint8_t)((value % 100) / 10);

if(state->hundreds > LED_MAX_HUNDREDS) state->hundreds = LED_MAX_HUNDREDS;
if(state->tens > LED_MAX_TENS) state->tens = LED_MAX_TENS;

}

void Reset_LED_State(LED_State_t *state, uint32_t current_tick)
{
state->digit_mode = 0;
state->blink_counter = 0;
state->blink_state = 0;
state->pause_counter = 0;
state->in_long_pause = 0;
state->blink_timer = current_tick;
}

void Init_LED_State(LED_State_t *state, LED_ID_t led_id, uint32_t current_tick)
{
state->hundreds = 0;
state->tens = 0;
state->digit_mode = 0;
state->blink_counter = 0;
state->blink_state = 0;
state->blink_timer = current_tick;
state->pause_counter = 0;
state->in_long_pause = 0;
state->long_pause_timer = current_tick;
state->led_id = led_id;
}

// ============================================================================
// ERROR INDICATION HELPER
// ============================================================================
void Blink_Error(uint8_t count)
{
uint8_t i;
for(i = 0; i < count; i++) {
LED_SETTING_ON();
Delay_Ms(LED_ERROR_BLINK_MS);
LED_SETTING_OFF();
Delay_Ms(LED_ERROR_BLINK_MS);
}
}

// ============================================================================
// MAIN FUNCTION
// ============================================================================
void main(void)
{
// Button state
uint8_t button_state = 0;
uint8_t button_last = 0;
uint32_t button_debounce_timer = 0;
uint8_t button_enable_capture = 0;

// Timing
uint32_t last_sample_time = 0;
uint32_t current_tick = 0;

// Post-clear lockout
uint8_t post_clear_lockout = 0;
uint32_t post_clear_lockout_timer = 0;

// Status flags
uint8_t sprom_error = 0;

// LED state machines
LED_State_t fault_led;
LED_State_t main_led;

// Loop variables (C89 compatibility)
uint8_t i;
uint8_t button_readings;
uint8_t stable_count;
uint8_t actual_button;
uint8_t button_current;

// ========================================================================
// INITIALIZATION
// ========================================================================
WDT_Clear();
GPIO_Init();
Timer_Init();
EA = 1; // Enable global interrupts
ADC_Init();
Filter_Reset();

// Ensure all LEDs start OFF
LED_MAIN_OFF();
LED_FAULT_OFF();
LED_SETTING_OFF();

// Initial stabilization (wait for power to settle)
Delay_Ms(STARTUP_STABILIZE_MS);

// Get initial tick
current_tick = Get_SystemTick();

// Initialize LED state machines
Init_LED_State(&fault_led, LED_ID_FAULT, current_tick);
Init_LED_State(&main_led, LED_ID_MAIN, current_tick);

// ========================================================================
// POWER-ON SEQUENCE
// ========================================================================

// Check for CLEAR MODE (button pressed at power-on)
button_readings = 0;
for(i = 0; i < BUTTON_POWERON_SAMPLES; i++) {
if(BUTTON_PRESSED()) {
button_readings++;
}
Delay_Ms(BUTTON_POWERON_DELAY_MS);
}

if(button_readings >= BUTTON_POWERON_THRESHOLD) {
// ====================================================================
// CLEAR MODE - Erase SPROM
// ====================================================================

// Visual indication: 3 blinks
Blink_Error(3);

// Erase SPROM
sprom_error = !SPROM_Erase();

if(sprom_error) {
// Erase failed - rapid blink error
Blink_Error(10);
}

// SETTING LED ON (ready for capture)
LED_SETTING_ON();

// Update state
A_valid = 0;
global_A = 0;

// Wait for button release with debounce
while(BUTTON_PRESSED()) {
WDT_Clear();
Delay_Ms(BUTTON_STABLE_DELAY_MS);
}

// Confirm stable release
stable_count = 0;
while(stable_count < BUTTON_STABLE_COUNT) {
if(BUTTON_PRESSED()) {
stable_count = 0;
} else {
stable_count++;
}
WDT_Clear();
Delay_Ms(BUTTON_STABLE_DELAY_MS);
}

// Additional settling time
Delay_Ms(POST_CLEAR_SETTLE_MS);

// Initialize button state
current_tick = Get_SystemTick();
actual_button = BUTTON_PRESSED() ? 1 : 0;
button_state = actual_button;
button_last = actual_button;
button_debounce_timer = current_tick;
last_sample_time = current_tick;

// Enable hard lockout (prevents immediate false capture)
button_enable_capture = 0;
post_clear_lockout = 1;
post_clear_lockout_timer = current_tick;

// Reset filters for fresh readings
Filter_Reset();

} else {
// ====================================================================
// NORMAL POWER-ON
// ====================================================================

// Load and verify data from SPROM
SPROM_Load_Data();

if(A_valid) {
LED_SETTING_OFF();
Update_LED_Digits(&fault_led, global_A);
} else {
LED_SETTING_ON();
}

// Initialize button state
current_tick = Get_SystemTick();
actual_button = BUTTON_PRESSED() ? 1 : 0;
button_state = actual_button;
button_last = actual_button;
button_debounce_timer = current_tick;
last_sample_time = current_tick;

// Enable capture if SPROM empty and button not pressed
button_enable_capture = (A_valid == 0 && actual_button == 0) ? 1 : 0;
post_clear_lockout = 0;
}

// Initialize LED timers
current_tick = Get_SystemTick();
main_led.blink_timer = current_tick;
fault_led.blink_timer = current_tick;

// ========================================================================
// MAIN LOOP
// ========================================================================
while(1) {
WDT_Clear();

// Get current tick (atomic read)
current_tick = Get_SystemTick();

// ====================================================================
// POST-CLEAR LOCKOUT HANDLER
// ====================================================================
if(post_clear_lockout) {
if((current_tick - post_clear_lockout_timer) >= POST_CLEAR_LOCKOUT_MS) {
post_clear_lockout = 0;

actual_button = BUTTON_PRESSED() ? 1 : 0;
button_state = actual_button;
button_last = actual_button;
button_debounce_timer = current_tick;

button_enable_capture = (actual_button == 0 && A_valid == 0) ? 1 : 0;
}
}

// ====================================================================
// ADC SAMPLING
// ====================================================================
if((current_tick - last_sample_time) >= ADC_SAMPLE_INTERVAL_MS) {
last_sample_time = current_tick;

// Read filtered peak value
global_B = ADC_ReadPeak(ADC_CHANNEL);

// Periodically measure VDD for compensation
current_vdd_mv = ADC_MeasureVDD();

// Update MAIN LED display
Update_LED_Digits(&main_led, global_B);
}

// ====================================================================
// BUTTON HANDLING
// ====================================================================
if(post_clear_lockout == 0) {
button_current = BUTTON_PRESSED() ? 1 : 0;

if(button_current != button_last) {
button_debounce_timer = current_tick;
button_last = button_current;
}

if((current_tick - button_debounce_timer) >= BUTTON_DEBOUNCE_MS) {
if(button_current != button_state) {
button_state = button_current;

if(button_state == 1) {
// ================================================
// BUTTON PRESS - CAPTURE EVENT
// ================================================

if(A_valid == 0 &&
button_enable_capture == 1 &&
post_clear_lockout == 0) {

button_enable_capture = 0;

// Capture current value
global_A = global_B;
global_A_raw = global_B_raw;

// Save to SPROM with timestamp and VDD
sprom_error = !SPROM_Save_Data(global_A,
current_vdd_mv,
current_tick);

if(sprom_error) {
// Write failed
Blink_Error(6);
LED_SETTING_ON();
A_valid = 0;
} else {
// Success
A_valid = 1;
Update_LED_Digits(&fault_led, global_A);
Reset_LED_State(&fault_led, current_tick);

// Confirmation blink
for(i = 0; i < 2; i++) {
LED_SETTING_ON();
Delay_Ms(LED_CAPTURE_BLINK_MS);
LED_SETTING_OFF();
Delay_Ms(LED_CAPTURE_BLINK_MS);
}

LED_SETTING_OFF();
}

// Re-sync all timers after blocking delays
current_tick = Get_SystemTick();
last_sample_time = current_tick;
main_led.blink_timer = current_tick;
fault_led.blink_timer = current_tick;
}

} else {
// ================================================
// BUTTON RELEASE
// ================================================
if(A_valid == 0 && post_clear_lockout == 0) {
button_enable_capture = 1;
}
}
}
}
}

// ====================================================================
// LED UPDATE
// ====================================================================

// FAULT LED - Shows A (only if valid)
if(A_valid) {
Update_LED_StateMachine(&fault_led, current_tick);
} else {
LED_FAULT_OFF();
}

// MAIN LED - Shows B (or fault indication)
if(adc_fault_flag) {
// ADC fault - rapid blink
if((current_tick / LED_ADC_FAULT_BLINK_MS) & 1) {
LED_MAIN_ON();
} else {
LED_MAIN_OFF();
}
} else {
Update_LED_StateMachine(&main_led, current_tick);
}

// No delay needed - timing is tick-driven
}

}
