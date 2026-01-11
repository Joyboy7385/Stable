# ADC SPROM Capture Firmware v2.0

Embedded firmware for the MS51 8-bit microcontroller that captures analog-to-digital converter (ADC) readings and stores them in non-volatile SPROM memory.

## Overview

This firmware provides a complete solution for:
- **ADC Signal Capture**: Reads analog signals with peak detection and validation
- **Persistent Storage**: Saves captured values to SPROM (non-volatile memory)
- **LED Status Display**: Uses blink patterns to display numeric values
- **User Interaction**: Button-controlled capture and clear operations

## Hardware Requirements

| Component | Specification |
|-----------|---------------|
| MCU | MS51 (8-bit 8051 core) |
| Clock | 16 MHz |
| ADC Input | P0.3 (AIN6) |
| Main LED | P1.5 |
| Fault LED | P0.4 |
| Setting LED | P0.1 |
| Button | P1.1 (active low) |

## Pin Configuration

```
P0.1 - SETTING LED (output, push-pull)
P0.3 - ADC Input (high-impedance, AIN6)
P0.4 - FAULT LED (output, push-pull)
P1.1 - Button (input, high-impedance, active low)
P1.5 - MAIN LED (output, push-pull)
```

## Features

### ADC Reading
- **Peak Detection**: Takes 3 samples and returns maximum value
- **Averaging**: Each sample is an average of 4 ADC conversions
- **Validation**: Only accepts readings between 100-1500 counts
- **Fault Tracking**: Monitors consecutive ADC failures

### SPROM Storage
- **Write Protection**: Values cannot be overwritten without explicit clear
- **Verification**: All writes are verified with retry logic (3 attempts)
- **Atomic Updates**: Valid marker written last to ensure data integrity

### LED Display System
The firmware uses blink patterns to display numeric values:

| LED | Purpose | Behavior |
|-----|---------|----------|
| MAIN LED | Shows current value (B) | Always active |
| FAULT LED | Shows stored value (A) | Only when SPROM has valid data |
| SETTING LED | SPROM state indicator | ON = empty/ready, OFF = value stored |

**Blink Pattern Format:**
- Hundreds digit: Slow blinks (400ms interval)
- Tens digit: Fast blinks (150ms interval)
- Zero value: Single short blink
- 3-second pause between display cycles

## Operation Modes

### Normal Power-On
1. Firmware loads stored value from SPROM
2. If valid data exists: SETTING LED OFF, FAULT LED displays stored value
3. If SPROM empty: SETTING LED ON, waiting for capture

### Clear Mode (Power-on with button pressed)
1. Hold button during power-on
2. SETTING LED blinks 3 times to confirm
3. SPROM is erased with verification
4. 2-second lockout prevents accidental captures
5. System ready for new capture

### Value Capture
1. Ensure SPROM is empty (SETTING LED ON)
2. Press button to capture current ADC reading
3. SETTING LED blinks 2 times to confirm
4. Value is permanently stored until next clear

## Timing Constants

| Parameter | Value | Description |
|-----------|-------|-------------|
| ADC Sample Interval | 80ms | Main loop sampling rate |
| Button Debounce | 50ms | Debounce period |
| Post-Clear Lockout | 2000ms | Lockout after SPROM clear |
| LED Slow Blink | 400ms | Hundreds digit interval |
| LED Fast Blink | 150ms | Tens digit interval |
| Long Pause | 3000ms | Between display cycles |

## SPROM Memory Map

| Address | Content | Values |
|---------|---------|--------|
| 0x00 | Value A (Low Byte) | 0x00-0xFF |
| 0x01 | Value A (High Byte) | 0x00-0xFF |
| 0x02 | Valid Marker | 0xFF=valid, 0x00=empty |

## Version 2.0 Improvements

### Critical Fixes
- **Atomic Tick Access**: Prevents torn reads of 32-bit system tick on 8-bit MCU
- **Post-Clear Lockout**: 2-second delay prevents false captures after clear

### High Priority
- **SPROM Write Verification**: Retry logic ensures reliable storage

### Medium Priority
- **Optimized Main Loop**: Removed unnecessary delays
- **Named Constants**: All magic numbers replaced with descriptive defines

### Low Priority
- **ADC Health Monitoring**: Tracks consecutive failures for fault detection

## Code Structure

```
adc_sprom_v2.c
├── Hardware Definitions
│   ├── SFR Registers
│   ├── Pin Definitions
│   └── LED/Button Macros
├── Configuration Constants
│   ├── SPROM Addresses
│   ├── Timing Values
│   └── ADC Parameters
├── Core Functions
│   ├── WDT_Clear()          - Watchdog refresh
│   ├── Get_SystemTick()     - Atomic tick read
│   ├── Delay_Ms/Us()        - Timing delays
│   ├── GPIO_Init()          - Pin configuration
│   ├── ADC_Init()           - ADC setup
│   └── Timer_Init()         - Timer0 for system tick
├── SPROM Functions
│   ├── SPROM_Read_Byte()    - Read from SPROM
│   ├── SPROM_Write_Byte_Verified() - Write with retry
│   ├── Clear_SPROM()        - Erase all data
│   ├── Save_A_to_SPROM()    - Store captured value
│   └── Load_A_from_SPROM()  - Load on power-up
├── ADC Functions
│   ├── ADC_ReadCount()      - Single conversion
│   ├── ADC_Read_Signal()    - Averaged reading
│   └── ADC_Read_3Sample_Peak() - Peak detection
├── LED State Machine
│   ├── LED_State_t          - State structure
│   ├── Update_LED_StateMachine() - Blink controller
│   ├── Update_LED_Digits()  - Value to digits
│   └── Reset_LED_State()    - Reset state
└── main()
    ├── Initialization
    ├── Power-on Sequence
    │   ├── Clear Mode Detection
    │   └── Normal Startup
    └── Main Loop
        ├── Lockout Handler
        ├── ADC Sampling
        ├── Button Handler
        └── LED Updates
```

## Building

This firmware is designed for the Keil C51 compiler with MS51 support:

```bash
# Requires Keil C51 and MS51 device support package
# Include path must contain "ms51_16k_keil.h"
```

## Global Variables

| Variable | Type | Description |
|----------|------|-------------|
| `systemTick` | `volatile uint32_t` | 1ms tick counter (ISR updated) |
| `global_A` | `uint16_t` | Stored/captured ADC value |
| `global_B` | `uint16_t` | Current live ADC value |
| `A_valid` | `uint8_t` | SPROM validity flag |
| `adc_fail_count` | `uint8_t` | Consecutive ADC failures |

## Error Handling

### SPROM Write Failure
- Rapid LED blink (10 times) during clear
- Rapid LED blink (6 times) during capture
- System remains in capture-ready state

### ADC Failure
- Returns last known good value
- Increments failure counter (max 255)
- Counter resets on successful read

## Author

Refactored for Praful - January 2025

## License

Proprietary embedded firmware.
