// -- PIN DEFINITIONS --
const uint8_t PIN_BUTTON_FORWARD = 9;
const uint8_t PIN_BUTTON_REVERSE = 8;
const uint8_t PIN_SPEED_KNOB = A0;

// -- PARAMETERS --
float axis_max_speed = 10; //default, but we update in setup


void your_setup(){ // put your setup code here, to run once:

  pinMode(PIN_BUTTON_FORWARD, INPUT_PULLUP); //here we use the internal pullup resistor to avoid wiring one on our own
  pinMode(PIN_BUTTON_REVERSE, INPUT_PULLUP);
  pinMode(PIN_SPEED_KNOB, INPUT); //This is an analog input, so no need for a pullup
  axis_max_speed = axis_calculate_max_speed(&axis_single); //this calculates the maximum axis speed, based on microstepping, pulley diameter, and max step rate, etc...
}

void your_loop(){ // put your code here, to run in a loop forever

  //1. calculate axis velocity based on knob
  int knob_value = analogRead(PIN_SPEED_KNOB);
  float knob_fractional_value = ((float)knob_value) / 1023.0; // we're scaling the knob value between 0 -> 1. The ADC is 10 bit, so max value is 1023.
  float axis_velocity = knob_fractional_value * axis_max_speed;

  if(digitalRead(PIN_BUTTON_FORWARD) == 0){ //forward button is pressed. With one side of the switch wired to GND, a pressed button corresponds to LOW.
    axis_move_at_velocity(&axis_single, axis_velocity); //spin axis at calculated velocity

  }else if(digitalRead(PIN_BUTTON_REVERSE) == 0){ //reverse button is pressed
    axis_move_at_velocity(&axis_single, -axis_velocity); //spin axis in reverse at calculated velocity

  }else{
    axis_move_at_velocity(&axis_single, 0);
  }
}