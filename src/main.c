/**
 * Copyright (c) 2019 Tara Keeling
 * 
 * This software is released under the MIT License.
 * https://opensource.org/licenses/MIT
 */

#include <stdio.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define WaitTimeout( op, us ) { \
    Timeout = 0; \
    while ( ( op ) && ( Timeout <= us ) ) { \
        _delay_us( 1 ); \
        Timeout++; \
    }; \
}

// PIN Configuration
#define Pin_Sensor PIN7
#define Pin_Latch PIN4
#define Pin_CLK PIN3
#define Pin_Data PIN1

// PORT Configuration
#define PORTA_PINCTL( pin ) ( _SFR_MEM8( 0x410 + pin ) )

#define SENSOR_PINCTL *( &( PORTA_PIN0CTRL ) + Pin_Sensor )
#define SENSOR_DIR PORTA_DIR
#define SENSOR_OUT PORTA_OUT
#define SENSOR_IN PORTA_IN

#define SHIFT_DIR PORTA_DIR
#define SHIFT_OUT PORTA_OUT

void ShiftOut( uint8_t Data );
void UpdateDisplay( void );
void SetTemperature( uint8_t Whole, uint8_t Frac );
bool DHT_Read( void );

volatile bool ShouldUpdateSensor = false;

const uint8_t DigitTable[ ] = {
    //abcdefg.
    0b11111100, // 0
    0b01100000, // 1
    0b11011010, // 2
    0b11110010, // 3
    0b01100110, // 4
    0b10110110, // 5
    0b10111110, // 6
    0b11100000, // 7
    0b11111110, // 8
    0b11100110, // 9
    0b11101110, // A
    0b00111110, // B
    0b00011010, // C
    0b01111010, // D
    0b10011110, // E
    0b10001110, // F
    0b00000010, // 0x10 Negative
    0b10000110, // 0x11 Celcius
    0b00000000, // 0x12 Off
};

uint8_t DisplayData[ 4 ] = {
    0xFF, 0xFF, 0xFF, 0xFF
};

int8_t DisplayIndex = 0;

uint8_t TempWhole = 0;
uint8_t TempFrac = 0;

// Called every 1ms
ISR( TCA0_OVF_vect ) {
    static volatile uint16_t i = 0;

    if ( ++i >= 5000 ) {
        ShouldUpdateSensor = true;
        i = 0;
    }
}
/*
ISR( TIM0_COMPA_vect ) {
    static volatile uint16_t i = 0;

    if ( ++i >= 5000 ) {
        i = 0;

        ShouldUpdateSensor = true;
    }
}
*/

void ShiftOut( uint8_t Data ) {
    uint8_t i = 0;

    for ( i = 0; i < 8; i++ ) {
        // Set the data pin according to the MSB of the digit data
        SHIFT_OUT = ( Data & 0x80 ) ? SHIFT_OUT | _BV( Pin_Data ) : SHIFT_OUT & ~_BV( Pin_Data );

        // Pulse the clock line
        SHIFT_OUT |= _BV( Pin_CLK );
        SHIFT_OUT &= ~_BV( Pin_CLK );       

        // Shift the next bit into the MSB
        Data<<= 1; 
    }
}

void UpdateDisplay( void ) {
    DisplayIndex = ( DisplayIndex >= sizeof( DisplayData ) ) ? 0 : DisplayIndex;

    ShiftOut( _BV( DisplayIndex ) );
    ShiftOut( DisplayData[ DisplayIndex ] );

    // Latch the data into the shift registers
    SHIFT_OUT |= _BV( Pin_Latch );
    SHIFT_OUT &= ~_BV( Pin_Latch );

    // Check if sensor pin is shared
#if Pin_Sensor == Pin_CLK || Pin_Sensor == Pin_Data || Pin_Sensor == Pin_Latch
    // Bring sensor/data line back high
    SHIFT_DIR |= Pin_Sensor;
    SHIFT_OUT |= Pin_Sensor;

    DisplayIndex++;
#endif
}

void SetTemperature( uint8_t Whole, uint8_t Frac ) {
    bool Negative = ( Whole & 0x80 ) ? true : false;
    uint8_t* Digits = DisplayData;
    uint8_t Ones = 0;
    uint8_t Tens = 0;
    uint8_t Hundreds = 0;

    Digits[ 0 ] = DigitTable[ 0x12 ];
    Digits[ 1 ] = DigitTable[ 0x12 ];
    Digits[ 2 ] = DigitTable[ 0x12 ];
    Digits[ 3 ] = DigitTable[ 0x11 ];

    if ( Negative == true ) {
        Whole = ~Whole;
        Whole++;

        Digits[ 0 ] = DigitTable[ 0x10 ];
    }

    Ones = Whole % 10;
    Tens = ( Whole / 10 ) % 10;
    Hundreds = ( Whole / 100 ) % 10;

    if ( Whole >= 0 && Whole <= 9 ) {
        Digits[ 2 ] = DigitTable[ Frac ];
        Digits[ 1 ] = DigitTable[ Ones ] | _BV( 0x0 );
    } else if ( Whole >= 10 && Whole <= 99 ) {
        if ( Negative == true ) {
            Digits[ 2 ] = DigitTable[ Ones ];
            Digits[ 1 ] = DigitTable[ Tens ];
        } else {
            Digits[ 2 ] = DigitTable[ Frac ];
            Digits[ 1 ] = DigitTable[ Ones ] | _BV( 0x0 );
            Digits[ 0 ] = DigitTable[ Tens ];
        }
    } else {
        if ( Negative == true ) {
            Digits[ 3 ] = DigitTable[ Ones ];
            Digits[ 2 ] = DigitTable[ Tens ];
            Digits[ 1 ] = DigitTable[ Hundreds ];
        } else {
            Digits[ 2 ] = DigitTable[ Ones ];
            Digits[ 1 ] = DigitTable[ Tens ];
            Digits[ 0 ] = DigitTable[ Hundreds ];
        }
    }
}

bool DHT_Read( void ) {
    uint8_t Data[ 5 ] = { 0, 0, 0, 0, 0 };
    uint8_t Timeout = 0;
    uint16_t CSum = 0;
    int8_t Bit = 0;
    int8_t i = 0;

    cli( );
        // Set sensor as output and bring it low for 20ms
        SENSOR_DIR |= _BV( Pin_Sensor );
        SENSOR_OUT &= ~_BV( Pin_Sensor );

        _delay_ms( 20 );

        // Set sensor pin to input with pull up and wait for 40us
        SENSOR_DIR &= ~_BV( Pin_Sensor );
        SENSOR_PINCTL = PORT_PULLUPEN_bm;

        _delay_us( 40 );

        // Wait for sensor to pull the data line low
        WaitTimeout( bit_is_set( SENSOR_IN, Pin_Sensor ), 100 );

        // Wait for sensor to bring the data line high
        WaitTimeout( bit_is_clear( SENSOR_IN, Pin_Sensor ), 100 );

        // DHT11 returns 5 bytes of data
        for ( i = 0; i < sizeof( Data ); i++ ) {
            // Read each bit (MSB First)
            for ( Bit = 7; Bit >= 0; Bit-- ) {
                // Wait for last bit to finish
                WaitTimeout( bit_is_set( SENSOR_IN, Pin_Sensor ), 100 );

                // Wait for start bit
                WaitTimeout( bit_is_clear( SENSOR_IN, Pin_Sensor ), 100 );

                // Sample in the middle of a 1 bit (35us out of 70)
                // If the sensor line is still high then it's a 1
                _delay_us( 35 );

                // Shift last bit over toward the MSBit
                Data[ i ] <<= 1;

                // If the data line is still high then this bit is a 1
                // set the appropriate bit and continue on
                if ( bit_is_set( SENSOR_IN, Pin_Sensor ) ) {
                    Data[ i ] |= 1;
                }
            }

            // Calculate checksum
            // Ignore the last byte (the actual checksum)
            if ( i < sizeof( Data ) - 1 ) {
                CSum+= Data[ i ];
            }
        }
    sei( );

    // Bring the sensor back to idle
    SENSOR_PINCTL &= ~PORT_PULLUPEN_bm;
    SENSOR_DIR |= _BV( Pin_Sensor );
    SENSOR_OUT |= _BV( Pin_Sensor );

    TempWhole = Data[ 2 ];
    TempFrac = Data[ 3 ];

    return ( CSum == Data[ 4 ] ) ? true : false;
}

int main( void ) {
    cli( );
        SENSOR_DIR = 0;
        SHIFT_DIR = 0;

        SENSOR_DIR |= _BV( Pin_Sensor );
        SHIFT_DIR |= _BV( Pin_CLK ) | _BV( Pin_Data ) | _BV( Pin_Latch );

        // Sensor line needs to start high
        SENSOR_OUT = Pin_Sensor;
    
        // ~1ms at 1MHz
        //OCR0A = ( uint16_t ) 999;
        //TIMSK0 = _BV( OCIE0A );
        //TCCR0B = _BV( CS00 ) | _BV( WGM02 );
        TCA0_SINGLE_PER = 416;
        TCA0_SINGLE_CTRLA = TCA_SINGLE_CLKSEL_DIV8_gc | TCA_SINGLE_ENABLE_bm;
        TCA0_SINGLE_INTCTRL = TCA_SINGLE_OVF_bm;
    sei( );

    while ( 1 ) {
        if ( ShouldUpdateSensor == true ) {
            ShouldUpdateSensor = false;

            if ( DHT_Read( ) == false ) {
                TempWhole = ( uint8_t ) -127;
                TempFrac = 0;
            }

            SetTemperature( TempWhole, TempFrac );
        }

        UpdateDisplay( );
    }

    return 0;
}
