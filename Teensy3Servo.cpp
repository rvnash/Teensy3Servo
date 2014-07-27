//
//  Teensy3Servo.cpp
//
//  Created by Richard Nash on 7/13/14.
//  Copyright (c) 2014 Richard Nash.
/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "Teensy3Servo.h"
#include <mk20dx128.h>
#include <core_pins.h>

// Prescale the Bus Clock (system clock) so the TIMER_FREQENCY gives
// count values close to but not over 2^16th. For 20ms servo period this
// value is 65,536 / 0.002 or maximum speed of 3,276,800 MHz.
#if F_CPU == 96000000 || F_CPU == 48000000
    // NOTE:
    // Even though the FlexTimer documentation refers to FTM_SC_CLKS(0x01)
    // as the "System Clock", it is actually the "Bus Clock" from the rest of
    // the documentation.
    // The way this is set up is in mk20dx128.c, using OUTDIV2==1 in both the
    // 96Mhz and 48Mhz processor speed case. So, the actual bus speed is 48Mhz
    // in both cases.
    #define CLK_PRESCALE    4        // Divide by 16
    #define TIMER_FREQENCY  (48000000/16)   // 3MHz
#elif F_CPU == 72000000 || F_CPU == 36000000
    // Assuming this will be done the same way as described above,
    // will use "Bus Clock" value of 36MHz
    #define CLK_PRESCALE    4        // Divide by 16
    #define TIMER_FREQENCY  (36000000/16)    // 2.25MHz
#elif F_CPU == 24000000
    #define CLK_PRESCALE    3        // Divide by 8
    #define TIMER_FREQENCY  (24000000/8)    // 3MHz
#elif F_CPU == 18000000
    #define CLK_PRESCALE    3        // Divide by 8
    #define TIMER_FREQENCY  (18000000/8)    // 2.25MHz
#else
    #error Unsupported clock speed F_CPU
#endif

// Assuming the full period is 20ms. So divide by 50 to get how many counts.
#define TIMER_COUNTS                (uint16_t)(TIMER_FREQENCY / 50 )

// Assuming the minus 180 position is 0.5ms pulse. So divide by 2000 to get how many counts.
#define MINUS_180_COUNT             (uint16_t)(TIMER_FREQENCY / 2000)

// Assuming the plus 180 position is 2.5ms pulse. So divide by 400 to get how many counts.
#define PLUS_180_COUNT              (uint16_t)(TIMER_FREQENCY / 400)

// Counts per degree
#define COUNTS_PER_DEGREE           ((float)(PLUS_180_COUNT - MINUS_180_COUNT) / 360.0f)

// Convert degrees to counts
#define degreesToCount(deg)         (uint16_t)((float)((deg)+180.0f) * COUNTS_PER_DEGREE + MINUS_180_COUNT + 0.5f)

// Convert counts to degrees
#define countToDegrees(cnt)         ((float)((cnt) - MINUS_180_COUNT) / COUNTS_PER_DEGREE - 180.0f)

// Some things not defined in mk20dx128.h
#define FTM_QDCTRL_QUADEN       (uint32_t)0x00000001    // First bit of FTMx_QDCTRL
#define FTM_CnSC_ELSB           (uint32_t)0x00000008    // Some channel control register bits
#define FTM_CnSC_ELSA           (uint32_t)0x00000004
#define FTM_CnSC_CHF            (uint32_t)0x00000080
#define FTM_CnSC_CHIE           (uint32_t)0x00000040
#define FTM_CnSC_MSB            (uint32_t)0x00000020
#define FTM_CnSC_MSA            (uint32_t)0x00000010

// Unfortunately, some static storage. 48 bytes
uint16_t Teensy3Servo::unmatchedRiseCounts[12];
uint16_t Teensy3Servo::pulseWidth[12];


// Initialize the pin to be a Servo output pin
// Returns: true on success, false on failure
bool Teensy3Servo::InitOut(int8_t teensyPin)
{
    volatile uint32_t *channelControlRegister;
    volatile uint32_t *portControlRegister;
    uint32_t altNumber;

    if (!InitByPin(teensyPin,&channelControlRegister,&portControlRegister,&altNumber)) return false;
    
    // Pin control Register (MUX to route the desired signal to the pin)
    *portControlRegister  = PORT_PCR_MUX(altNumber)  | PORT_PCR_DSE;
    
    // FTMx_CnSC - contains the channel-interrupt status flag control bits
    // CnSC set to edge aligned PWM
    // MSnB:MSnA   (5:4) = 11
    // ELSnB:ELSnA (3:2) = 01
    *channelControlRegister = FTM_CnSC_MSB | FTM_CnSC_MSA | FTM_CnSC_ELSB;
    
    // Initialize the output to 0 servo angle
    Teensy3Servo::Set(teensyPin, 0);
    
    return true;
}

// Sets the servo pin to the given value
// The pin should have been previously initialized with InitOut
// val is nominally in the range -180 <= val <= 180
// In actuality, your servo may not support this range.
// Returns: true on success, false on failure
bool Teensy3Servo::Set(int8_t teensyPin, int16_t val)
{
    uint32_t value = degreesToCount(val);
    //FTMx_CnV contains the comparison value for the pulse width
    switch (teensyPin) {
        case 3:
            FTM1_C0V = value;
            break;
        case 4:
            FTM1_C1V = value;
            break;
        case 5:
            FTM0_C7V = value;
            break;
        case 6:
            FTM0_C4V = value;
            break;
        case 9:
            FTM0_C2V = value;
            break;
        case 10:
            FTM0_C3V = value;
            break;
        case 20:
            FTM0_C5V = value;
            break;
        case 21:
            FTM0_C6V = value;
            break;
        case 22:
            FTM0_C0V = value;
            break;
        case 23:
            FTM0_C1V = value;
            break;
        case 25:
            FTM2_C1V = value;
            break;
        case 32:
            FTM2_C0V = value;
            break;
        default:
            return false;
    }
    return true;
}


// Initialize the pin to be a Servo input pin
// Note, this also enables interrupts on all of the timers
// Returns: true on success, false on failure
bool Teensy3Servo::InitIn(int8_t teensyPin)
{
    volatile uint32_t *channelControlRegister;
    volatile uint32_t *portControlRegister;
    uint32_t altNumber;
    
    
    if (!InitByPin(teensyPin,&channelControlRegister,&portControlRegister,&altNumber)) return false;
    
    // Pin control Register (MUX to route the pin to the desired signal.)
    *portControlRegister  = PORT_PCR_MUX(altNumber);
    
    // FTMx_CnSC - contains the channel-interrupt status flag control bits
    // Continuious capture mode, rising edge AND falling edges, enable channel interrupt
    *channelControlRegister = FTM_CnSC_ELSB | FTM_CnSC_ELSA | FTM_CnSC_CHIE ;

    // Might as well enable all of the interrupts for the FTMs
    NVIC_ENABLE_IRQ(IRQ_FTM0);
    NVIC_ENABLE_IRQ(IRQ_FTM1);
    NVIC_ENABLE_IRQ(IRQ_FTM2);
    
    return true;
}

#ifdef DEBUG
#include <WProgram.h>
static int nRisesInARow = 0;
static int nFallsInARow = 0;
static int nRisesVersion = 0;
static int nFallsVersion = 0;
#endif

// Gets the value of the servo signal on the given pin.
// The pin should have been previous initialized using InitIn
// Enough time should have transpired (20ms) to get one cycle
// of rise and fall.
// Note: This assumes that the input Servo timing is the same
// as the timing of the output this library produces.
// Returns -180 to 180
int16_t Teensy3Servo::Get(int8_t teensyPin)
{
    uint16_t width;
    
   switch (teensyPin) {
        case 3:
            width = pulseWidth[0];
            break;
        case 4:
            width = pulseWidth[1];
            break;
        case 5:
            width = pulseWidth[2];
            break;
        case 6:
            width = pulseWidth[3];
           break;
        case 9:
            width = pulseWidth[4];
            break;
        case 10:
            width = pulseWidth[5];
            break;
        case 20:
            width = pulseWidth[6];
            break;
        case 21:
            width = pulseWidth[7];
            break;
        case 22:
            width = pulseWidth[8];
            break;
        case 23:
            width = pulseWidth[9];
            break;
        case 25:
            width = pulseWidth[10];
            break;
        case 32:
            width = pulseWidth[11];
            break;
        default:
            return 0;
    }
   
#ifdef DEBUG
    Serial.printf("Rise in a row: %d\n\r", nRisesInARow);
    Serial.printf("Fall in a row: %d\n\r", nFallsInARow);
    Serial.printf("Rises version: %d\n\r", nRisesVersion);
    Serial.printf("Falls version: %d\n\r", nFallsVersion);
    Serial.printf("width: %d\n\r", width);
#endif
    
    return (int16_t)(countToDegrees(width)+0.5f);
    
}

// Private Static Methods

// Initialize output for FlexTimer0
void Teensy3Servo::InitTimer(volatile uint32_t *FTMx_MODE,
                             volatile uint32_t *FTMx_COMBINE,
                             volatile uint32_t *FTMx_QDCTRL,
                             volatile uint32_t *FTMx_SC,
                             volatile uint32_t *FTMx_CNT,
                             volatile uint32_t *FTMx_MOD,
                             volatile uint32_t *FTMx_CNTIN)
{
    //Enable the Clock to the FTMx Module
    
    //FTM0_MODE[WPDIS] = 1; //Disable Write Protection - enables changes to QUADEN, DECAPEN, etc.
    *FTMx_MODE |= FTM_MODE_WPDIS;
    
    //FTMEN is bit 0, need to set to 0
    *FTMx_MODE &= ~FTM_MODE_FTMEN;
    
    // You can't do DECAP mode on the same timer that you are doing edge aligned PWM
    // So, instead, just use ajacent pins in input mode. It means you need to use two pins.
    *FTMx_COMBINE = 0;
    
    //Set Edge Aligned PWM
    //QUADEN is Bit 1, Set Quadrature Decoder Mode (QUADEN) Enable to 0,   (disabled)
    *FTMx_QDCTRL &= ~FTM_QDCTRL_QUADEN;
    
    // Selects Clock source to be system clock
    // Sets pre-scale value
    *FTMx_SC = FTM_SC_CLKS(0x01) | FTM_SC_PS(CLK_PRESCALE);
    
    *FTMx_CNT = 0x0; //FTM Counter Value - reset counter to zero
    *FTMx_MOD = TIMER_COUNTS;  // Count value of full duty cycle
    *FTMx_CNTIN = 0; //Set the Counter Initial Value to 0
}

bool Teensy3Servo::InitByPin(uint8_t teensyPin, volatile uint32_t **channelControlRegister,
                             volatile uint32_t **pportControlRegister, uint32_t *paltNumber)
{
    switch (teensyPin) {
        case 3:
            SIM_SCGC6 |= SIM_SCGC6_FTM1;
            Teensy3Servo::InitTimer(&FTM1_MODE, &FTM1_COMBINE, &FTM1_QDCTRL, &FTM1_SC, &FTM1_CNT, &FTM1_MOD, &FTM1_CNTIN);
            *channelControlRegister = &FTM1_C0SC;
            *pportControlRegister = &PORTA_PCR12;
            *paltNumber = 3;
            break;
        case 4:
            SIM_SCGC6 |= SIM_SCGC6_FTM1;
            Teensy3Servo::InitTimer(&FTM1_MODE, &FTM1_COMBINE, &FTM1_QDCTRL, &FTM1_SC, &FTM1_CNT, &FTM1_MOD, &FTM1_CNTIN);
            *channelControlRegister = &FTM1_C1SC;
            *pportControlRegister = &PORTA_PCR13;
            *paltNumber = 3;
            break;
        case 5:
            SIM_SCGC6 |= SIM_SCGC6_FTM0;
            Teensy3Servo::InitTimer(&FTM0_MODE, &FTM0_COMBINE, &FTM0_QDCTRL, &FTM0_SC, &FTM0_CNT, &FTM0_MOD, &FTM0_CNTIN);
            *channelControlRegister = &FTM0_C7SC;
            *pportControlRegister = &PORTD_PCR7;
            *paltNumber = 4;
            break;
        case 6:
            SIM_SCGC6 |= SIM_SCGC6_FTM0;
            Teensy3Servo::InitTimer(&FTM0_MODE, &FTM0_COMBINE, &FTM0_QDCTRL, &FTM0_SC, &FTM0_CNT, &FTM0_MOD, &FTM0_CNTIN);
            *channelControlRegister = &FTM0_C4SC;
            *pportControlRegister = &PORTD_PCR4;
            *paltNumber = 4;
            break;
        case 9:
            SIM_SCGC6 |= SIM_SCGC6_FTM0;
            Teensy3Servo::InitTimer(&FTM0_MODE, &FTM0_COMBINE, &FTM0_QDCTRL, &FTM0_SC, &FTM0_CNT, &FTM0_MOD, &FTM0_CNTIN);
            *channelControlRegister = &FTM0_C2SC;
            *pportControlRegister = &PORTC_PCR3;
            *paltNumber = 4;
            break;
        case 10:
            SIM_SCGC6 |= SIM_SCGC6_FTM0;
            Teensy3Servo::InitTimer(&FTM0_MODE, &FTM0_COMBINE, &FTM0_QDCTRL, &FTM0_SC, &FTM0_CNT, &FTM0_MOD, &FTM0_CNTIN);
            *channelControlRegister = &FTM0_C3SC;
            *pportControlRegister = &PORTC_PCR4;
            *paltNumber = 4;
            break;
        case 20:
            SIM_SCGC6 |= SIM_SCGC6_FTM0;
            Teensy3Servo::InitTimer(&FTM0_MODE, &FTM0_COMBINE, &FTM0_QDCTRL, &FTM0_SC, &FTM0_CNT, &FTM0_MOD, &FTM0_CNTIN);
            *channelControlRegister = &FTM0_C5SC;
            *pportControlRegister = &PORTD_PCR5;
            *paltNumber = 4;
            break;
        case 21:
            SIM_SCGC6 |= SIM_SCGC6_FTM0;
            Teensy3Servo::InitTimer(&FTM0_MODE, &FTM0_COMBINE, &FTM0_QDCTRL, &FTM0_SC, &FTM0_CNT, &FTM0_MOD, &FTM0_CNTIN);
            *channelControlRegister = &FTM0_C6SC;
            *pportControlRegister = &PORTD_PCR6;
            *paltNumber = 4;
            break;
        case 22:
            SIM_SCGC6 |= SIM_SCGC6_FTM0;
            Teensy3Servo::InitTimer(&FTM0_MODE, &FTM0_COMBINE, &FTM0_QDCTRL, &FTM0_SC, &FTM0_CNT, &FTM0_MOD, &FTM0_CNTIN);
            *channelControlRegister = &FTM0_C0SC;
            *pportControlRegister = &PORTC_PCR1;
            *paltNumber = 4;
            break;
        case 23:
            SIM_SCGC6 |= SIM_SCGC6_FTM0;
            Teensy3Servo::InitTimer(&FTM0_MODE, &FTM0_COMBINE, &FTM0_QDCTRL, &FTM0_SC, &FTM0_CNT, &FTM0_MOD, &FTM0_CNTIN);
            *channelControlRegister = &FTM0_C1SC;
            *pportControlRegister = &PORTC_PCR2;
            *paltNumber = 4;
            break;
        case 25:
            SIM_SCGC3 |= SIM_SCGC3_FTM2;
            Teensy3Servo::InitTimer(&FTM2_MODE, &FTM2_COMBINE, &FTM2_QDCTRL, &FTM2_SC, &FTM2_CNT, &FTM2_MOD, &FTM2_CNTIN);
            *channelControlRegister = &FTM2_C1SC;
            *pportControlRegister = &PORTB_PCR19;
            *paltNumber = 3;
            break;
        case 32:
            SIM_SCGC3 |= SIM_SCGC3_FTM2;
            Teensy3Servo::InitTimer(&FTM2_MODE, &FTM2_COMBINE, &FTM2_QDCTRL, &FTM2_SC, &FTM2_CNT, &FTM2_MOD, &FTM2_CNTIN);
            *channelControlRegister = &FTM2_C0SC;
            *pportControlRegister = &PORTB_PCR18;
            *paltNumber = 4;
            break;
        default:
            return false;
    }
    return true;
}

// FTMx_ISR are interrupt service routines called from Input servo activity
// Both rising and falling edges generate input. Go through the list of
// channels to see which one(s) caused the interrupt, and capture the counts.
// Fill in both rise/fall on fall
void fillInPulseWidth(uint16_t *pulseWidth, uint16_t riseTime, uint16_t fallTime)
{
    if (fallTime > riseTime) {
        *pulseWidth = fallTime-riseTime;
    } else {
        *pulseWidth = fallTime + (TIMER_COUNTS - riseTime);
    }
}

void ftm0_isr()
{
    if (FTM0_C7SC & FTM_CnSC_CHF) { // Pin 5
        FTM0_C7SC &= ~FTM_CnSC_CHF;
        if (CORE_PIN5_PINREG & CORE_PIN5_BITMASK) {
            Teensy3Servo::unmatchedRiseCounts[2] = FTM0_C7V;
        } else {
            fillInPulseWidth(&(Teensy3Servo::pulseWidth[2]), Teensy3Servo::unmatchedRiseCounts[2], FTM0_C7V);
        }
    }
    if (FTM0_C4SC & FTM_CnSC_CHF) { // Pin 6
        FTM0_C4SC &= ~FTM_CnSC_CHF;
        if (CORE_PIN6_PINREG & CORE_PIN6_BITMASK) {
            Teensy3Servo::unmatchedRiseCounts[3] = FTM0_C4V;
        } else {
            fillInPulseWidth(&(Teensy3Servo::pulseWidth[3]), Teensy3Servo::unmatchedRiseCounts[3], FTM0_C4V);
        }
#ifdef DEBUG
        if (CORE_PIN6_PINREG & CORE_PIN6_BITMASK) {
            nRisesVersion++;
            if (nFallsInARow) {
                nRisesInARow = 1;
                nFallsInARow = 0;
            } else {
                nRisesInARow++;
            }
        } else {
            nFallsVersion++;
            if (nRisesInARow) {
                nFallsInARow = 1;
                nRisesInARow = 0;
            } else {
                nFallsInARow++;
            }
        }
#endif
    }
    if (FTM0_C2SC & FTM_CnSC_CHF) { // Pin 9
        FTM0_C2SC &= ~FTM_CnSC_CHF;
        if (CORE_PIN9_PINREG & CORE_PIN9_BITMASK) {
            Teensy3Servo::unmatchedRiseCounts[4] = FTM0_C2V;
        } else {
            fillInPulseWidth(&(Teensy3Servo::pulseWidth[4]), Teensy3Servo::unmatchedRiseCounts[4], FTM0_C2V);
        }
    }
    if (FTM0_C3SC & FTM_CnSC_CHF) { // Pin 10
        FTM0_C3SC &= ~FTM_CnSC_CHF;
        if (CORE_PIN10_PINREG & CORE_PIN10_BITMASK) {
            Teensy3Servo::unmatchedRiseCounts[5] = FTM0_C3V;
        } else {
            fillInPulseWidth(&(Teensy3Servo::pulseWidth[5]), Teensy3Servo::unmatchedRiseCounts[5], FTM0_C3V);
        }
    }
    if (FTM0_C5SC & FTM_CnSC_CHF) { // Pin 20
        FTM0_C5SC &= ~FTM_CnSC_CHF;
        if (CORE_PIN20_PINREG & CORE_PIN20_BITMASK) {
            Teensy3Servo::unmatchedRiseCounts[6] = FTM0_C5V;
        } else {
            fillInPulseWidth(&(Teensy3Servo::pulseWidth[6]), Teensy3Servo::unmatchedRiseCounts[6], FTM0_C5V);
        }
    }
    if (FTM0_C6SC & FTM_CnSC_CHF) { // Pin 21
        FTM0_C6SC &= ~FTM_CnSC_CHF;
        if (CORE_PIN21_PINREG & CORE_PIN21_BITMASK) {
            Teensy3Servo::unmatchedRiseCounts[7] = FTM0_C6V;
        } else {
            fillInPulseWidth(&(Teensy3Servo::pulseWidth[7]), Teensy3Servo::unmatchedRiseCounts[7], FTM0_C6V);
        }
    }
    if (FTM0_C0SC & FTM_CnSC_CHF) { // Pin 22
        FTM0_C0SC &= ~FTM_CnSC_CHF;
        if (CORE_PIN22_PINREG & CORE_PIN22_BITMASK) {
            Teensy3Servo::unmatchedRiseCounts[8] = FTM0_C0V;
        } else {
            fillInPulseWidth(&(Teensy3Servo::pulseWidth[8]), Teensy3Servo::unmatchedRiseCounts[8], FTM0_C0V);
        }
    }
    if (FTM0_C1SC & FTM_CnSC_CHF) { // Pin 23
        FTM0_C1SC &= ~FTM_CnSC_CHF;
        if (CORE_PIN23_PINREG & CORE_PIN23_BITMASK) {
            Teensy3Servo::unmatchedRiseCounts[9] = FTM0_C1V;
        } else {
            fillInPulseWidth(&(Teensy3Servo::pulseWidth[9]), Teensy3Servo::unmatchedRiseCounts[9], FTM0_C1V);
        }
    }
}

void ftm1_isr()
{
    if (FTM1_C0SC & FTM_CnSC_CHF) { // Pin 3
        FTM1_C0SC &= ~FTM_CnSC_CHF;
        if (CORE_PIN3_PINREG & CORE_PIN3_BITMASK) {
            Teensy3Servo::unmatchedRiseCounts[0] = FTM1_C0V;
        } else {
            fillInPulseWidth(&(Teensy3Servo::pulseWidth[0]), Teensy3Servo::unmatchedRiseCounts[0], FTM1_C0V);
        }
    }
    if (FTM1_C1SC & FTM_CnSC_CHF) { // Pin 4
        FTM1_C1SC &= ~FTM_CnSC_CHF;
        if (CORE_PIN4_PINREG & CORE_PIN4_BITMASK) {
            Teensy3Servo::unmatchedRiseCounts[1] = FTM1_C1V;
        } else {
            fillInPulseWidth(&(Teensy3Servo::pulseWidth[1]), Teensy3Servo::unmatchedRiseCounts[1], FTM1_C1V);
        }
    }
}

void ftm2_isr()
{
    if (FTM2_C0SC & FTM_CnSC_CHF) { // Pin 32
        FTM2_C0SC &= ~FTM_CnSC_CHF;
        if (CORE_PIN32_PINREG & CORE_PIN32_BITMASK) {
            Teensy3Servo::unmatchedRiseCounts[11] = FTM2_C0V;
        } else {
            fillInPulseWidth(&(Teensy3Servo::pulseWidth[11]), Teensy3Servo::unmatchedRiseCounts[11], FTM2_C0V);
        }
    }
    if (FTM2_C1SC & FTM_CnSC_CHF) { // Pin 25
        FTM2_C1SC &= ~FTM_CnSC_CHF;
        if (CORE_PIN25_PINREG & CORE_PIN25_BITMASK) {
            Teensy3Servo::unmatchedRiseCounts[10] = FTM2_C1V;
        } else {
            fillInPulseWidth(&(Teensy3Servo::pulseWidth[10]), Teensy3Servo::unmatchedRiseCounts[10], FTM2_C1V);
        }
    }
}


