// Motion Control Pin Definitions
//
// This is where you should define all the arduino uno pins that you're using.

// -- STEPPER DRIVER PINS --
// note: the definitions below will map to the arduino stepper shield that we'll be providing in lab, so for convenience it might make
//       sense to use these for breadboarding in class as well.

const uint8_t PIN_X_STEP  = 2; //digital pin 2
const uint8_t PIN_X_DIR   = 5; //digital pin 5

const uint8_t PIN_Y_STEP  = 3;
const uint8_t PIN_Y_DIR   = 6;

const uint8_t PIN_Z_STEP  = 4;
const uint8_t PIN_Z_DIR   = 7;

// -- CONTROL BUTTONS --
const uint8_t PIN_BUTTON_FORWARD = A0;
const uint8_t PIN_BUTTON_REVERSE = A1;
const uint8_t PIN_SPEED_KNOB = A2;