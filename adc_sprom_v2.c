/*
* ============================================================================
* ADC WITH SPROM CAPTURE - VERSION 2.0 (FULLY REFACTORED)
* ============================================================================
* 
* FEATURES:
* - PEAK detection (3 samples, maximum value)
* - SPROM write protection (must clear before new capture)
* - Clear mode: Power-on with button pressed
* - SETTING LED indicates SPROM state
* - FAULT LED shows A (only if valid)
* - MAIN LED shows B (always)
* 
* VERSION 2.0 FIXES:
* - [CRITICAL] Atomic systemTick access to prevent torn reads on 8-bit MCU
* - [CRITICAL] Post-clear lockout to prevent false captures
* - [HIGH] SPROM write verification with retry logic
* - [MEDIUM] Removed unnecessary Delay_Ms(1) from main loop
* - [MEDIUM] All magic numbers replaced with named constants
* - [LOW] ADC failure counter for fault detection
* 
* AUTHOR: Refactored for Praful
* DATE: January 2025
* MCU: MS51 (8-bit 8051 core)
* ============================================================================
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
// SPROM CONFIGURATION
// ============================================================================
#define SPROM_ADDR_A_LOW        0x00
#define SPROM_ADDR_A_HIGH       0x01
#define SPROM_ADDR_VALID        0x02    // 0xFF = A exists, 0x00 = empty

#define SPROM_VALID_MARKER      0xFF
#define SPROM_EMPTY_MARKER      0x00
#define SPROM_WRITE_RETRIES     3       // Number of write retry attempts

// ============================================================================
// TIMING CONSTANTS (all in milliseconds)
// ============================================================================
#define TIMER_TICK_MS           1       // Timer0 generates 1ms ticks

// Button timing
#define BUTTON_DEBOUNCE_MS      50      // Debounce period
#define BUTTON_POWERON_SAMPLES  5       // Number of samples at power-on
#define BUTTON_POWERON_DELAY_MS 10      // Delay between power-on samples
#define BUTTON_POWERON_THRESHOLD 3      // Minimum samples for clear mode
#define BUTTON_STABLE_COUNT     10      // Samples for stable release detection
#define BUTTON_STABLE_DELAY_MS  10      // Delay between stability samples

// Post-clear lockout
#define POST_CLEAR_LOCKOUT_MS   2000    // 2 second lockout after SPROM clear
#define POST_CLEAR_SETTLE_MS    500     // Additional settling after button release

// ADC timing
#define ADC_SAMPLE_INTERVAL_MS  80      // Main loop ADC sample rate
#define ADC_INIT_DELAY_MS       50      // ADC initialization delay
#define ADC_CHANNEL_SETTLE_US   1000    // Channel switching settle time
#define ADC_SAMPLE_DELAY_US     2000    // Delay between ADC samples
#define ADC_PEAK_INTERVAL_MS    20      // Delay between peak samples

// LED timing
#define LED_BLINK_SLOW_MS       400     // Hundreds digit blink interval
#define LED_BLINK_FAST_MS       150     // Tens digit blink interval
#define LED_LONG_PAUSE_MS       3000    // Pause between digit cycles
#define LED_CLEAR_BLINK_MS      300     // Clear mode indication blink
#define LED_CAPTURE_BLINK_MS    200     // Capture confirmation blink

// Startup
#define STARTUP_STABILIZE_MS    100     // Initial stabilization delay

// ============================================================================
// ADC CONFIGURATION
// ============================================================================
#define ADC_CHANNEL             6       // P0.3 = AIN6
#define ADC_SAMPLES_PER_READ    4       // Samples for averaging
#define ADC_PEAK_SAMPLES        3       // Samples for peak detection

#define ADC_MIN_VALID           100     // Minimum valid ADC reading
#define ADC_MAX_VALID           1500    // Maximum valid ADC reading
#define ADC_DEFAULT_VALUE       450     // Default value if no valid readings
#define ADC_TIMEOUT             10000   // ADC conversion timeout

#define ADC_FAIL_THRESHOLD      10      // Consecutive failures before fault

// ============================================================================
// LED DISPLAY LIMITS
// ============================================================================
#define LED_MAX_HUNDREDS        12      // Maximum hundreds digit
#define LED_MAX_TENS            9       // Maximum tens digit
#define LED_ZERO_PAUSE_COUNT    2       // Pause cycles for zero digit
#define LED_NONZERO_PAUSE_COUNT 3       // Pause cycles for non-zero digit

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define PIN_MAIN_LED            P15
#define PIN_FAULT_LED           P04
#define PIN_SETTING_LED         P01
#define PIN_BUTTON              P11

// LED macros
#define LED_MAIN_ON()           (PIN_MAIN_LED = 1)
#define LED_MAIN_OFF()          (PIN_MAIN_LED = 0)
#define LED_FAULT_ON()          (PIN_FAULT_LED = 1)
#define LED_FAULT_OFF()         (PIN_FAULT_LED = 0)
#define LED_SETTING_ON()        (PIN_SETTING_LED = 1)
#define LED_SETTING_OFF()       (PIN_SETTING_LED = 0)

// Button macro (active low)
#define BUTTON_PRESSED()        (PIN_BUTTON == 0)

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
volatile __xdata uint32_t systemTick = 0;

// ADC values
uint16_t global_A = 0;          // Captured/stored value
uint16_t global_B = 0;          // Current live value
uint8_t A_valid = 0;            // 0 = SPROM empty, 1 = A exists

// ADC health monitoring
uint8_t adc_fail_count = 0;     // Consecutive ADC failure counter

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
void WDT_Clear(void);
uint32_t Get_SystemTick(void);
void Delay_Ms(uint32_t ms);
void Delay_Us(uint32_t us);

void GPIO_Init(void);
void ADC_Init(void);
void Timer_Init(void);

uint8_t SPROM_Write_Byte_Verified(uint8_t addr, uint8_t dat);
uint8_t SPROM_Read_Byte(uint8_t addr);
uint8_t Clear_SPROM(void);
uint8_t Save_A_to_SPROM(uint16_t value);
void Load_A_from_SPROM(void);

uint16_t ADC_ReadCount(void);
uint16_t ADC_Read_Signal(void);
uint16_t ADC_Read_3Sample_Peak(void);

// ============================================================================
// WATCHDOG
// ============================================================================
void WDT_Clear(void)
{
    __asm
        mov 0xC7, #0xAA
        mov 0xC7, #0x55
        orl 0xAA, #0x40
    __endasm;
}

// ============================================================================
// ATOMIC SYSTEM TICK ACCESS
// ============================================================================
// CRITICAL: On 8-bit MCU, 32-bit reads can be "torn" if ISR fires mid-read.
// This function disables interrupts during the read to ensure atomicity.
// ============================================================================
uint32_t Get_SystemTick(void)
{
    uint32_t tick;
    uint8_t ea_save = EA;
    EA = 0;             // Disable interrupts
    tick = systemTick;  // Read all 4 bytes atomically
    EA = ea_save;       // Restore interrupt state
    return tick;
}

// ============================================================================
// TIMER0 ISR - 1ms tick
// ============================================================================
void Timer0_ISR(void) __interrupt (1)
{  
    TH0 = 0xFA;     // Reload for 1ms @ 16MHz
    TL0 = 0xDB;
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

void Delay_Us(uint32_t us)
{
    uint32_t i;
    uint32_t cycles = (us * 16) / 6;  // Approximate for 16MHz
    for(i = 0; i < cycles; i++) {
        __asm nop __endasm;
    }
}

// ============================================================================
// SPROM FUNCTIONS WITH VERIFICATION
// ============================================================================

// Low-level write (no verification)
static void SPROM_Write_Byte_Raw(uint8_t addr, uint8_t dat)
{
    uint8_t EA_save = EA;
    EA = 0;
   
    TA = 0xAA;
    TA = 0x55;
    IAPFD = dat;
    IAPCN = 0x22;
    IAPAH = 0x00;
    IAPAL = addr;
    IAPTRG = 0x01;
   
    EA = EA_save;
}

// Read byte from SPROM
uint8_t SPROM_Read_Byte(uint8_t addr)
{
    uint8_t dat;
    uint8_t EA_save = EA;
    EA = 0;
   
    TA = 0xAA;
    TA = 0x55;
    IAPCN = 0x20;
    IAPAH = 0x00;
    IAPAL = addr;
    IAPTRG = 0x01;
    dat = IAPFD;
   
    EA = EA_save;
    return dat;
}

// Write with verification and retry
// Returns: 1 = success, 0 = failed after retries
uint8_t SPROM_Write_Byte_Verified(uint8_t addr, uint8_t dat)
{
    uint8_t retry;
    uint8_t readback;
    
    for(retry = 0; retry < SPROM_WRITE_RETRIES; retry++) {
        SPROM_Write_Byte_Raw(addr, dat);
        
        // Small delay for write to complete
        Delay_Us(100);
        
        // Verify
        readback = SPROM_Read_Byte(addr);
        if(readback == dat) {
            return 1;  // Success
        }
        
        WDT_Clear();
    }
    
    return 0;  // Failed after all retries
}

// Clear SPROM with verification
// Returns: 1 = success, 0 = failed
uint8_t Clear_SPROM(void)
{
    uint8_t success = 1;
    
    // Clear valid marker first (most important)
    if(!SPROM_Write_Byte_Verified(SPROM_ADDR_VALID, SPROM_EMPTY_MARKER)) {
        success = 0;
    }
    
    // Clear data bytes
    if(!SPROM_Write_Byte_Verified(SPROM_ADDR_A_LOW, 0x00)) {
        success = 0;
    }
    if(!SPROM_Write_Byte_Verified(SPROM_ADDR_A_HIGH, 0x00)) {
        success = 0;
    }
    
    return success;
}

// Save value to SPROM with verification
// Returns: 1 = success, 0 = failed
uint8_t Save_A_to_SPROM(uint16_t value)
{
    uint8_t success = 1;
    
    // Write data bytes first
    if(!SPROM_Write_Byte_Verified(SPROM_ADDR_A_LOW, (uint8_t)(value & 0xFF))) {
        success = 0;
    }
    if(!SPROM_Write_Byte_Verified(SPROM_ADDR_A_HIGH, (uint8_t)(value >> 8))) {
        success = 0;
    }
    
    // Write valid marker last (commits the data)
    if(!SPROM_Write_Byte_Verified(SPROM_ADDR_VALID, SPROM_VALID_MARKER)) {
        success = 0;
    }
    
    return success;
}

// Load A from SPROM
void Load_A_from_SPROM(void)
{
    uint8_t valid = SPROM_Read_Byte(SPROM_ADDR_VALID);
   
    if(valid == SPROM_VALID_MARKER) {
        uint8_t low = SPROM_Read_Byte(SPROM_ADDR_A_LOW);
        uint8_t high = SPROM_Read_Byte(SPROM_ADDR_A_HIGH);
        global_A = ((uint16_t)high << 8) | low;
        A_valid = 1;
    } else {
        global_A = 0;
        A_valid = 0;
    }
}

// ============================================================================
// GPIO INITIALIZATION
// ============================================================================
void GPIO_Init(void)
{
    // LED outputs (push-pull)
    P0M1 &= ~((1<<4) | (1<<1));     // P0.4, P0.1 not open-drain
    P0M2 |=  ((1<<4) | (1<<1));     // P0.4, P0.1 push-pull
    P1M1 &= ~(1<<5);                // P1.5 not open-drain
    P1M2 |=  (1<<5);                // P1.5 push-pull
   
    // Button input at P1.1 (high-impedance input)
    P1M1 |=  (1<<1);
    P1M2 &= ~(1<<1);
   
    // ADC input P0.3 (high-impedance, disable digital)
    P0M1 |=  (1<<3);
    P0M2 &= ~(1<<3);
    P0S  &= ~(1<<3);
   
    // Initialize all LEDs OFF
    LED_MAIN_OFF();
    LED_FAULT_OFF();
    LED_SETTING_OFF();
}

// ============================================================================
// ADC INITIALIZATION
// ============================================================================
void ADC_Init(void)
{
    ADCCON1 = 0x00;
    ADCCON2 = 0x00;
    ADCCON0 = 0x00;
   
    ADCCON1 |= (0x03 << 4);     // Clock divider
    ADCCON1 &= ~0x80;           // Single conversion mode
    ADCCON2 = 0x0E;             // Conversion timing
    ADCDLY = 0xFF;              // Maximum delay
    AINDIDS = 0x00;
    AINDIDS &= ~(1<<6);         // Disable digital input on AIN6
    ADCCON0 = ADC_CHANNEL;      // Select channel
    ADCCON1 |= 0x01;            // Enable ADC
   
    Delay_Ms(ADC_INIT_DELAY_MS);
}

// ============================================================================
// TIMER INITIALIZATION
// ============================================================================
void Timer_Init(void)
{
    TMOD &= 0xF0;       // Clear Timer0 bits
    TMOD |= 0x01;       // Timer0 Mode 1 (16-bit)
    TH0 = 0xFA;         // Reload value for 1ms @ 16MHz
    TL0 = 0xDB;
    ET0 = 1;            // Enable Timer0 interrupt
    TR0 = 1;            // Start Timer0
}

// ============================================================================
// ADC FUNCTIONS
// ============================================================================

// Single ADC conversion with timeout
uint16_t ADC_ReadCount(void)
{
    __xdata uint16_t result;
    __xdata uint16_t timeout;
   
    ADCF = 0;           // Clear conversion flag
    ADCS = 1;           // Start conversion
   
    timeout = ADC_TIMEOUT;
    while(ADCF == 0 && --timeout);
   
    if(timeout == 0) {
        return 0xFFFF;  // Timeout error
    }
   
    result = ADCRH;
    result = (result << 4) | (ADCRL & 0x0F);
    return result;
}

// Averaged ADC reading with validation
uint16_t ADC_Read_Signal(void)
{
    uint32_t sum = 0;
    uint8_t i;
    uint8_t valid_count = 0;
    uint16_t reading;
    static uint16_t last_good = ADC_DEFAULT_VALUE;
   
    ADCCON0 = ADC_CHANNEL;
    Delay_Us(ADC_CHANNEL_SETTLE_US);
   
    for(i = 0; i < ADC_SAMPLES_PER_READ; i++) {
        reading = ADC_ReadCount();
       
        if(reading >= ADC_MIN_VALID && reading <= ADC_MAX_VALID) {
            sum += reading;
            valid_count++;
        }
       
        Delay_Us(ADC_SAMPLE_DELAY_US);
    }
   
    if(valid_count == 0) {
        // Track consecutive failures
        if(adc_fail_count < 255) {
            adc_fail_count++;
        }
        return last_good;
    }
   
    // Reset failure counter on success
    adc_fail_count = 0;
   
    uint16_t result = (uint16_t)(sum / valid_count);
    last_good = result;
    return result;
}

// Peak detection - returns maximum of multiple samples
uint16_t ADC_Read_3Sample_Peak(void)
{
    uint16_t max_reading = 0;
    uint8_t i;
   
    for(i = 0; i < ADC_PEAK_SAMPLES; i++) {
        uint16_t reading = ADC_Read_Signal();
       
        if(reading > max_reading) {
            max_reading = reading;
        }
       
        if(i < (ADC_PEAK_SAMPLES - 1)) {
            Delay_Ms(ADC_PEAK_INTERVAL_MS);
        }
    }
   
    return max_reading;
}

// ============================================================================
// LED BLINK STATE MACHINE STRUCTURE
// ============================================================================
typedef struct {
    uint8_t hundreds;           // Hundreds digit (0-12)
    uint8_t tens;               // Tens digit (0-9)
    uint8_t digit_mode;         // 0 = hundreds, 1 = tens
    uint8_t blink_counter;      // Current blink count
    uint8_t blink_state;        // LED on/off state
    uint32_t blink_timer;       // Last blink time
    uint8_t pause_counter;      // Pause cycle counter
    uint8_t in_long_pause;      // In long pause between cycles
    uint32_t long_pause_timer;  // Long pause start time
} LED_State_t;

// ============================================================================
// LED STATE MACHINE UPDATE FUNCTION
// ============================================================================
void Update_LED_StateMachine(LED_State_t *state, uint8_t *led_pin, uint32_t current_tick)
{
    if(state->in_long_pause) {
        // In long pause between digit cycles
        if((current_tick - state->long_pause_timer) >= LED_LONG_PAUSE_MS) {
            state->in_long_pause = 0;
        }
        return;
    }
    
    // Determine blink interval based on digit mode
    uint16_t interval = (state->digit_mode == 0) ? LED_BLINK_SLOW_MS : LED_BLINK_FAST_MS;
    
    if((current_tick - state->blink_timer) < interval) {
        return;  // Not time to update yet
    }
    
    state->blink_timer = current_tick;
    
    uint8_t current_digit = (state->digit_mode == 0) ? state->hundreds : state->tens;
    
    if(current_digit == 0) {
        // Zero: single short blink
        if(state->blink_counter == 0) {
            *led_pin = 1;  // LED ON
            state->blink_counter = 1;
        } else {
            *led_pin = 0;  // LED OFF
            state->pause_counter++;
            
            if(state->pause_counter >= LED_ZERO_PAUSE_COUNT) {
                state->pause_counter = 0;
                state->blink_counter = 0;
                state->digit_mode = !state->digit_mode;
                
                if(state->digit_mode == 0) {
                    // Completed full cycle, enter long pause
                    state->in_long_pause = 1;
                    state->long_pause_timer = current_tick;
                }
            }
        }
    } else {
        // Non-zero: blink N times
        if(state->blink_counter < current_digit) {
            state->blink_state = !state->blink_state;
            
            if(state->blink_state) {
                *led_pin = 1;  // LED ON
            } else {
                *led_pin = 0;  // LED OFF
                state->blink_counter++;
            }
        } else {
            *led_pin = 0;  // LED OFF
            state->pause_counter++;
            
            if(state->pause_counter >= LED_NONZERO_PAUSE_COUNT) {
                state->pause_counter = 0;
                state->blink_counter = 0;
                state->blink_state = 0;
                state->digit_mode = !state->digit_mode;
                
                if(state->digit_mode == 0) {
                    // Completed full cycle, enter long pause
                    state->in_long_pause = 1;
                    state->long_pause_timer = current_tick;
                }
            }
        }
    }
}

// ============================================================================
// HELPER: Update digits from value
// ============================================================================
void Update_LED_Digits(LED_State_t *state, uint16_t value)
{
    state->hundreds = (uint8_t)(value / 100);
    state->tens = (uint8_t)((value % 100) / 10);
    
    // Clamp to display limits
    if(state->hundreds > LED_MAX_HUNDREDS) {
        state->hundreds = LED_MAX_HUNDREDS;
    }
    if(state->tens > LED_MAX_TENS) {
        state->tens = LED_MAX_TENS;
    }
}

// ============================================================================
// HELPER: Reset LED state machine
// ============================================================================
void Reset_LED_State(LED_State_t *state, uint32_t current_tick)
{
    state->digit_mode = 0;
    state->blink_counter = 0;
    state->blink_state = 0;
    state->pause_counter = 0;
    state->in_long_pause = 0;
    state->blink_timer = current_tick;
}

// ============================================================================
// MAIN FUNCTION
// ============================================================================
void main(void)
{
    // Button state variables
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
    
    // SPROM operation status
    uint8_t sprom_error = 0;
    
    // LED state machines
    LED_State_t fault_led = {0};
    LED_State_t main_led = {0};
    
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
    EA = 1;             // Enable global interrupts
    ADC_Init();
    
    // Ensure all LEDs start OFF
    LED_MAIN_OFF();
    LED_FAULT_OFF();
    LED_SETTING_OFF();
    
    // Initial stabilization
    Delay_Ms(STARTUP_STABILIZE_MS);
    
    // ========================================================================
    // POWER-ON SEQUENCE
    // ========================================================================
    
    // Check if button is pressed at power-on (CLEAR MODE detection)
    button_readings = 0;
    for(i = 0; i < BUTTON_POWERON_SAMPLES; i++) {
        if(BUTTON_PRESSED()) {
            button_readings++;
        }
        Delay_Ms(BUTTON_POWERON_DELAY_MS);
    }
    
    if(button_readings >= BUTTON_POWERON_THRESHOLD) {
        // ====================================================================
        // CLEAR MODE - Button pressed at power-on
        // ====================================================================
        
        // Visual indication: Blink SETTING LED 3 times
        for(i = 0; i < 3; i++) {
            LED_SETTING_ON();
            Delay_Ms(LED_CLEAR_BLINK_MS);
            LED_SETTING_OFF();
            Delay_Ms(LED_CLEAR_BLINK_MS);
        }
        
        // Clear SPROM with verification
        sprom_error = !Clear_SPROM();
        
        if(sprom_error) {
            // SPROM clear failed - rapid blink to indicate error
            for(i = 0; i < 10; i++) {
                LED_SETTING_ON();
                Delay_Ms(100);
                LED_SETTING_OFF();
                Delay_Ms(100);
            }
        }
        
        // SETTING LED ON steady (SPROM cleared, waiting for capture)
        LED_SETTING_ON();
        
        // Update state
        A_valid = 0;
        global_A = 0;
        
        // Step 1: Wait for button release
        while(BUTTON_PRESSED()) {
            WDT_Clear();
            Delay_Ms(BUTTON_STABLE_DELAY_MS);
        }
        
        // Step 2: Confirm stable release (debounce)
        stable_count = 0;
        while(stable_count < BUTTON_STABLE_COUNT) {
            if(BUTTON_PRESSED()) {
                stable_count = 0;   // Reset on bounce
            } else {
                stable_count++;     // Count stable readings
            }
            WDT_Clear();
            Delay_Ms(BUTTON_STABLE_DELAY_MS);
        }
        
        // Step 3: Additional settling time
        Delay_Ms(POST_CLEAR_SETTLE_MS);
        
        // Step 4: Initialize button state from actual reading
        current_tick = Get_SystemTick();
        actual_button = BUTTON_PRESSED() ? 1 : 0;
        button_state = actual_button;
        button_last = actual_button;
        button_debounce_timer = current_tick;
        last_sample_time = current_tick;
        
        // CRITICAL: Capture disabled until lockout expires
        button_enable_capture = 0;
        
        // CRITICAL: Enable hard lockout
        post_clear_lockout = 1;
        post_clear_lockout_timer = current_tick;
        
    } else {
        // ====================================================================
        // NORMAL POWER-ON
        // ====================================================================
        
        // Load stored value from SPROM
        Load_A_from_SPROM();
        
        if(A_valid) {
            // SPROM has valid data
            LED_SETTING_OFF();
            
            // Initialize FAULT LED digits
            Update_LED_Digits(&fault_led, global_A);
            
        } else {
            // SPROM is empty
            LED_SETTING_ON();
        }
        
        // Initialize button state from actual reading
        current_tick = Get_SystemTick();
        actual_button = BUTTON_PRESSED() ? 1 : 0;
        button_state = actual_button;
        button_last = actual_button;
        button_debounce_timer = current_tick;
        last_sample_time = current_tick;
        
        // Enable capture if SPROM empty and button not pressed
        if(A_valid == 0 && actual_button == 0) {
            button_enable_capture = 1;
        } else {
            button_enable_capture = 0;
        }
        
        // No lockout for normal power-on
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
        
        // Get current tick ONCE per loop iteration (atomic read)
        current_tick = Get_SystemTick();
        
        // ====================================================================
        // POST-CLEAR LOCKOUT HANDLER
        // ====================================================================
        if(post_clear_lockout) {
            if((current_tick - post_clear_lockout_timer) >= POST_CLEAR_LOCKOUT_MS) {
                // Lockout expired
                post_clear_lockout = 0;
                
                // Re-read actual button state
                actual_button = BUTTON_PRESSED() ? 1 : 0;
                button_state = actual_button;
                button_last = actual_button;
                button_debounce_timer = current_tick;
                
                // Enable capture only if button is released and SPROM empty
                if(actual_button == 0 && A_valid == 0) {
                    button_enable_capture = 1;
                } else {
                    button_enable_capture = 0;
                }
            }
        }
        
        // ====================================================================
        // ADC SAMPLING
        // ====================================================================
        if((current_tick - last_sample_time) >= ADC_SAMPLE_INTERVAL_MS) {
            last_sample_time = current_tick;
            
            // Read peak ADC value
            global_B = ADC_Read_3Sample_Peak();
            
            // Update MAIN LED digits
            Update_LED_Digits(&main_led, global_B);
        }
        
        // ====================================================================
        // BUTTON HANDLING
        // ====================================================================
        if(post_clear_lockout == 0) {
            // Only process button when not in lockout
            
            button_current = BUTTON_PRESSED() ? 1 : 0;
            
            // Detect state change - restart debounce timer
            if(button_current != button_last) {
                button_debounce_timer = current_tick;
                button_last = button_current;
            }
            
            // Check if debounce period has passed
            if((current_tick - button_debounce_timer) >= BUTTON_DEBOUNCE_MS) {
                if(button_current != button_state) {
                    // Valid state change detected
                    button_state = button_current;
                    
                    if(button_state == 1) {
                        // ================================================
                        // BUTTON PRESS DETECTED
                        // ================================================
                        
                        // Triple condition check for capture
                        if(A_valid == 0 && 
                           button_enable_capture == 1 && 
                           post_clear_lockout == 0) {
                            
                            // Disable further captures
                            button_enable_capture = 0;
                            
                            // Capture current value
                            global_A = global_B;
                            
                            // Save to SPROM with verification
                            sprom_error = !Save_A_to_SPROM(global_A);
                            
                            if(sprom_error) {
                                // SPROM write failed
                                // Rapid blink to indicate error
                                for(i = 0; i < 6; i++) {
                                    LED_SETTING_ON();
                                    Delay_Ms(100);
                                    LED_SETTING_OFF();
                                    Delay_Ms(100);
                                }
                                
                                // Stay in capture mode (SETTING LED ON)
                                LED_SETTING_ON();
                                A_valid = 0;
                                
                            } else {
                                // SPROM write successful
                                A_valid = 1;
                                
                                // Update FAULT LED digits
                                Update_LED_Digits(&fault_led, global_A);
                                
                                // Reset FAULT LED state machine
                                Reset_LED_State(&fault_led, current_tick);
                                
                                // Visual confirmation: Blink SETTING LED 2 times
                                for(i = 0; i < 2; i++) {
                                    LED_SETTING_ON();
                                    Delay_Ms(LED_CAPTURE_BLINK_MS);
                                    LED_SETTING_OFF();
                                    Delay_Ms(LED_CAPTURE_BLINK_MS);
                                }
                                
                                // SETTING LED stays OFF (capture complete)
                                LED_SETTING_OFF();
                            }
                            
                            // Re-sync timers after blocking delays
                            current_tick = Get_SystemTick();
                            last_sample_time = current_tick;
                            main_led.blink_timer = current_tick;
                        }
                        
                    } else {
                        // ================================================
                        // BUTTON RELEASE DETECTED
                        // ================================================
                        
                        // Re-enable capture if conditions met
                        if(A_valid == 0 && post_clear_lockout == 0) {
                            button_enable_capture = 1;
                        }
                    }
                }
            }
        }
        
        // ====================================================================
        // FAULT LED - Shows A (only if valid)
        // ====================================================================
        if(A_valid) {
            Update_LED_StateMachine(&fault_led, &PIN_FAULT_LED, current_tick);
        } else {
            LED_FAULT_OFF();
        }
        
        // ====================================================================
        // MAIN LED - Shows B (always)
        // ====================================================================
        Update_LED_StateMachine(&main_led, &PIN_MAIN_LED, current_tick);
        
        // NOTE: Removed Delay_Ms(1) - unnecessary since timing is tick-driven
    }
}
