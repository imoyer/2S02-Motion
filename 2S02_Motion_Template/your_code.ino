// -- PIN DEFINITIONS --
const uint8_t PIN_MOTOR_ENABLE = 8;
const uint8_t PIN_BUTTON = 9;

// -- PARAMETERS --
float motor_speed_steps_per_sec = 50; //for a motor with 200 steps/rev, this is 0.25 rev/sec


void your_setup(){ // put your setup code here, to run once:

  pinMode(PIN_BUTTON, INPUT_PULLUP); //here we use the internal pullup resistor to avoid wiring one on our own
  pinMode(PIN_MOTOR_ENABLE, OUTPUT); //this pin controls whether the motor drivers are enabled
  digitalWrite(PIN_MOTOR_ENABLE, LOW); //Enables the motor drivers (they are active low)
}

void your_loop(){ // put your code here, to run in a loop forever

  if(digitalRead(PIN_BUTTON) == 0){ //forward button is pressed. With one side of the switch wired to GND, a pressed button corresponds to LOW.
    motor_spin(&motor_a, motor_speed_steps_per_sec); //spin motor at calculated velocity
  }else{
    motor_spin(&motor_a, 0);
  }
}

motor_move_relative_synchronous(struct motor &motor_a, 399, 580, 100);
motor_move_relative_synchronous(struct motor &motor_b, 400, 580, 100);
motor_wait_for_all_idle();

