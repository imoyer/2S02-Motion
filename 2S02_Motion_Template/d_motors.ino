// Generates output steps to each motor, based on target positions and velocity limits.

// --- GLOBAL STEP GENERATION PARAMETERS ---
const int STEP_GEN_TICK_PERIOD_US = 100; //microseconds. This yields a max output step rate of 10k steps/sec.
const long STEP_GEN_MAX_RATE_STEPS_PER_SEC = 1000000 / STEP_GEN_TICK_PERIOD_US; //steps per second
const long MOTOR_ACCUMULATOR_THRESHOLD = 1000000;

// --- MOTORS ---
struct motor{
  // step and direction pin numbers using the Arduino Uno numbering convention. 
  const uint8_t MOTOR_STEP_PIN;
  const uint8_t MOTOR_DIR_PIN;

  // direction invert flag. If 1, the motor direction will be inverted.
  const uint8_t MOTOR_DIR_INVERT;

  // step+direction pin numbers and ports using the Atmega numbering convention.
  //  These all get auto-populated when the motor is initialized.
  uint8_t motor_step_pin_atmega;
  uint8_t *motor_step_PORT;
  uint8_t motor_dir_pin_atmega;
  uint8_t *motor_dir_PORT;

  // step generation state variables. These are all declared "volatile" because they are used by the interrupt routine, and 
  // the compiler may otherwise remove them from the program if it doesn't see them written to in the main code.
  volatile long motor_target_position;
  volatile long motor_current_position;
  volatile long motor_accumulator;
  volatile long motor_accumulator_velocity;
  volatile uint8_t motor_last_direction;
};

// -- DEFINE MOTORS --
// Here is where we list all motors.

// Step 1 – create a new motor struct here.
struct motor motor_a = {.MOTOR_STEP_PIN = PIN_X_STEP, .MOTOR_DIR_PIN = PIN_X_DIR, .MOTOR_DIR_INVERT = 0};
struct motor motor_b = {.MOTOR_STEP_PIN = PIN_Y_STEP, .MOTOR_DIR_PIN = PIN_Y_DIR, .MOTOR_DIR_INVERT = 0};
struct motor motor_c = {.MOTOR_STEP_PIN = PIN_Z_STEP, .MOTOR_DIR_PIN = PIN_Z_DIR, .MOTOR_DIR_INVERT = 0};

// Step 2 -  adjust the motor count
const int MOTOR_COUNT = 3;

// Step 3 - add a pointer to the all_motors array following the pattern below.
struct motor *all_motors[MOTOR_COUNT] = {&motor_a, &motor_b, &motor_c};


// -- MOTOR INITIALIZATION FUNCTIONS --
void motor_initialize(struct motor *motor_target){
  // initialize a single motor.

  // 1. Populate the atmega pin and port definitions based on the arduino uno step and direction pins provided during initialization.
  // This is done by looking them up in arduino_uno_definitions.ino
  motor_target->motor_step_pin_atmega = UNO_PINS[motor_target->MOTOR_STEP_PIN];
  motor_target->motor_step_PORT = UNO_PORTS[motor_target->MOTOR_STEP_PIN];
  motor_target->motor_dir_pin_atmega = UNO_PINS[motor_target->MOTOR_DIR_PIN];
  motor_target->motor_dir_PORT = UNO_PORTS[motor_target->MOTOR_DIR_PIN];

  // 2. Initialize all the state variables
  // These control the motor step generation algorithm
  motor_target->motor_target_position = 0;
  motor_target->motor_current_position = 0;
  motor_target->motor_accumulator = 0;
  motor_target->motor_accumulator_velocity = 0;
  motor_target->motor_last_direction = 0;

  //3. Initialize step and direction pins as outputs
  pinMode(motor_target->MOTOR_STEP_PIN, OUTPUT);
  pinMode(motor_target->MOTOR_DIR_PIN, OUTPUT);

  //4. Set the max velocity to the max step rate.
  motor_velocity_set_max(STEP_GEN_MAX_RATE_STEPS_PER_SEC/2, motor_target);
}

void motors_initialize(){
  // Initializes all motors
  int motor_index;
  for(motor_index = 0; motor_index < MOTOR_COUNT; motor_index ++){
    motor_initialize(all_motors[motor_index]);
  }
  step_generator_start();
  
  motor_a.motor_target_position = 10000000;
  motor_b.motor_target_position = 10000000;
  motor_c.motor_target_position = 10000000;
}

void motor_velocity_set_max(float velocity_max_steps_per_sec, struct motor *motor_target){
  // sets the maximum velocity permissible on a motor.
  const float tick_time_seconds = (float) STEP_GEN_TICK_PERIOD_US / 1000000.0; //seconds per tick
  float steps_per_tick = velocity_max_steps_per_sec * tick_time_seconds; //steps per tick
  if(steps_per_tick>1.0){
    //cap the velocity at 1 step per tick
    steps_per_tick = 1.0;
  }
  motor_target->motor_accumulator_velocity = (long)((float)MOTOR_ACCUMULATOR_THRESHOLD * steps_per_tick);
}

// --- STEP GENERATOR --

void step_generator_start(){
  // Starts the step timer interrupt routine
  // We're using the built-in Timer/Counter1 to generate an interrupt each time the counter reaches a threshold value.
  // We'll configure the timer in "Clear Timer on Compare Match" (CTC) mode, which means that when the timer reaches its maximum
  // value, it will trigger an interrupt and then reset and continue counting. In this way, we'll get interrupt calls every X microseconds
  // for a maximum step rate of Y khz (these are set in the global step generation parameters at the top of the file).

  uint16_t timer_top_value = (F_CPU / 1000000) * STEP_GEN_TICK_PERIOD_US; //the number of CPU cycles between step generation interrupts

  TCCR1A = (0<<COM1A1)|(0<<COM1A0)|(0<<COM1B1)|(0<<COM1B0)|(0<<WGM11)|(0<<WGM10); //all zeros, see page 134 of the ATMega328PA datasheet for reference
  TCCR1B = (0<<ICNC1)|(0<<ICES1)|(0<<WGM13)|(1<<WGM12)|(0<<CS12)|(0<<CS11)|(1<<CS10); //CTC on OCR1A, CLK/1 (run directly off the system clock, no prescalar)
  TCCR1C = (0<<FOC1A)|(0<<FOC1B);

  OCR1A = timer_top_value; //set the output compare register to the top value
  TIMSK1 = (1<<OCIE1A); //enable interrupts. The interrupt routine for Output Compare 1A will be called whenever the counter reaches timer_top_value
}

ISR(TIMER1_COMPA_vect){
  PORTB|= (1<<PB0);
  // This is where the magic happens. The ISR (interrupt service routine) gets called each time TIMER1 reaches the top value set set in the step_generator_start() function.
  int motor_index;
  for(motor_index = 0; motor_index < MOTOR_COUNT; motor_index ++){
    // reference the target motor
    struct motor *motor_target = all_motors[motor_index];
    
    // calculate distance to target position, in steps.
    long motor_delta_position = motor_target->motor_target_position - motor_target->motor_current_position;
    
    //determine direction of motion
    int motor_delta_direction;
    if(motor_delta_position > 0){
      motor_delta_direction = 1;
    }else{
      motor_delta_direction = 0;
    }

    //IF A STEP IS AVALIABLE...
    if(motor_delta_position != 0){
      //calculate the active accumulator threshold
      long motor_accumulator_active_threshold;
      if(motor_delta_direction ^ motor_target->motor_last_direction){
        //if direction has changed, we use an accumulator that is double the normal value.
        motor_accumulator_active_threshold = MOTOR_ACCUMULATOR_THRESHOLD * 2;
      }else{
        motor_accumulator_active_threshold = MOTOR_ACCUMULATOR_THRESHOLD;
      }

      if(motor_target->motor_accumulator >= motor_accumulator_active_threshold){
        // The accumulator is already maxed out. This happens when the accumulator has time to fill before a step is taken.
        // In this case, we simply take a step and empty the accumulator.
        motor_step(motor_target, motor_delta_direction); //take a step
        motor_target->motor_accumulator = 0;

      }else{
        motor_target->motor_accumulator += motor_target->motor_accumulator_velocity; //increment the accumulator by the velocity

        if(motor_target->motor_accumulator >= motor_accumulator_active_threshold){
          //we've _now_ exceeded the accumulator threshold, so take a step
          motor_step(motor_target, motor_delta_direction);
          motor_target->motor_accumulator -= motor_accumulator_active_threshold;
        }
      }
    } else { //we are not taking a step, but still want to increment the accumulator if it isn't already capped out.
      if(motor_target->motor_accumulator < 2*MOTOR_ACCUMULATOR_THRESHOLD){ //2x threshold is the top of what is meaningful for us.
        motor_target->motor_accumulator += motor_target->motor_accumulator_velocity;
      }
    }
  }
  PORTB&= ~(1<<PB0);
}

void motor_step(struct motor *motor_target, int step_direction){
  //actually takes the step
  if(step_direction){ //forwards
    motor_target->motor_current_position ++;
    motor_target->motor_last_direction = 1;
    if(motor_target->MOTOR_DIR_INVERT){ //direction is inverted, go in reverse
      *motor_target->motor_dir_PORT &= ~(1<<motor_target->motor_dir_pin_atmega);
    }else{ //go forwards
      *motor_target->motor_dir_PORT |= (1<<motor_target->motor_dir_pin_atmega);
    }
  }else{ //reverse
    //check that limits are off, or that within negative limits
    motor_target->motor_current_position --;
    motor_target->motor_last_direction = 0;
    if(motor_target->MOTOR_DIR_INVERT){ //direction is inverted, go forwards
      *motor_target->motor_dir_PORT |= (1<<motor_target->motor_dir_pin_atmega);
    }else{ //go in reverse
      *motor_target->motor_dir_PORT &= ~(1<<motor_target->motor_dir_pin_atmega);
    }
  }
  //Now, to actually take a step
  *motor_target->motor_step_PORT |= (1<<motor_target->motor_step_pin_atmega);
  *motor_target->motor_step_PORT &= ~(1<<motor_target->motor_step_pin_atmega);
}