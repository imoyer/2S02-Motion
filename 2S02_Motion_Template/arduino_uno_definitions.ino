// Arduino Uno Definitions
// 
// We're going to be controlling the stepper motors using a background "interrupt routine," which
// means that we need write FAST to the digital I/O so that we don't slow down the main program code.
//
// As we saw in class, the arduino digitalWrite function is 20x slower than direct writes.
// The below code helps with mapping between arduino pin numbers (i.e. 1-12) and the internal registers
// that allow fast IO access.

// Each pin has two important memory locations: PORT and DDR. Writing a 1 to the correct "bit position" within
// PORT will make the corresponding pin go high. Similarly, writing a 0 will make it go low. This is the
// equivalent of the arduino digitalWrite command. DDR controls the direction of the pin, where 1 makes it an output
// and 0 makes it an input. This is the equivalent of pinMode().

// The arduino pins are organized into banks of 8 pins, each with a corresponding 8-bit PORT and DDR register. Each
// pin has a location within the register. For example, Arduino Uno's D1 is mapped to bit 0 in PORTD and DDRD, and has the
// name PD0.

// The arrays below help to look up the bit location as well as PORT registers for each of the Uno's pins D1-D13.

uint8_t null_register; //we use this in cases where there isn't a valid register to write to, like for pin 0 (which doesn't exist).
const uint8_t UNO_PINS[14] = {PD0, PD1, PD2, PD3, PD4, PD5, PD6, PD7, PB0, PB1, PB2, PB3, PB4, PB5};
const uint8_t *UNO_PORTS[14] = {&PORTD, &PORTD, &PORTD, &PORTD, &PORTD, &PORTD, &PORTD, &PORTD, &PORTB, &PORTB, &PORTB, &PORTB, &PORTB};