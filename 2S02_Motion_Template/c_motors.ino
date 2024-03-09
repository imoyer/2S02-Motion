// Generates output steps to each motor, based on target positions and velocity limits.

// --- GLOBAL STEP GENERATION PARAMETERS ---
const int STEP_GEN_TICK_PERIOD_US = 100; //microseconds. This yields a max output step rate of 10k steps/sec.
const long STEP_GEN_MAX_RATE_STEPS_PER_SEC = 1000000 / STEP_GEN_TICK_PERIOD_US; //steps per second
const long MOTOR_ACCUMULATOR_THRESHOLD_DEFAULT = 1000000; //we'll use this when not using the bresenham algorithm for synchronized motion.
const long BIG_NUMBER = 1000000000; //just another big number. We use this in the motor_spin function, by setting the target position very far away


// --- MOTOR STRUCT ---
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
  volatile long motor_accumulator_threshold;
};

// -- DEFINE MOTORS --
// Here is where we list all motors used in our project.

// Step 1 – create a new motor struct here.
struct motor motor_a = {.MOTOR_STEP_PIN = PIN_X_STEP, .MOTOR_DIR_PIN = PIN_X_DIR, .MOTOR_DIR_INVERT = 0};
struct motor motor_b = {.MOTOR_STEP_PIN = PIN_Y_STEP, .MOTOR_DIR_PIN = PIN_Y_DIR, .MOTOR_DIR_INVERT = 0};
struct motor motor_c = {.MOTOR_STEP_PIN = PIN_Z_STEP, .MOTOR_DIR_PIN = PIN_Z_DIR, .MOTOR_DIR_INVERT = 0};

// Step 2 -  adjust the motor count
const int MOTOR_COUNT = 3;

// Step 3 - add a pointer to the all_motors array following the pattern below.
struct motor *all_motors[MOTOR_COUNT] = {&motor_a, &motor_b, &motor_c};

// --- MOTOR FUNCTIONS ---
void motor_spin(struct motor *motor_target, float velocity_steps_per_sec){
  // Causes an individual motor to spin at a specified speed.
  //    motor_target -- a pointer to the target motor. You should call this with &motor_name.
  //    velocity_steps_per_sec -- a floating-point number specifying the velocity in steps per second. Can be positive or negative.
  // For example, calling motor_spin(&motor_a, -100.5) will make motor_a spin at approximately 100.5 steps per second.

  motor_set_velocity(motor_target, velocity_steps_per_sec);

  //set the motor target position to something far away
  if(velocity_steps_per_sec > 0){
    motor_target->motor_target_position = BIG_NUMBER;
  }else{
    motor_target->motor_target_position = -BIG_NUMBER;
  }
}

void motor_move_absolute(struct motor *motor_target, long motor_position_steps, float speed_steps_per_sec){
  // Moves an individual motor to an absolute target position at a specified speed.
  //    motor_target -- a pointer to the target motor. You should call this with &motor_name.
  //    motor_position_steps -- the target motor position, in steps. Either positive or negative.
  //    speed_steps_per_sec -- a floating-point number specifying the speed in positive steps per second.
  // For example, calling motor_move_absolute(&motor_a, 1000, 100) will make motor_a move to position 1000, at a velocity of 100 steps per second.
  motor_set_velocity(motor_target, speed_steps_per_sec);
  motor_target->motor_target_position = motor_position_steps;
}

void motor_move_relative(struct motor *motor_target, long motor_delta_steps, float speed_steps_per_sec){
  // Moves an individual motor by a relative number of steps at a specified speed.
  //    motor_target -- a pointer to the target motor. You should call this with &motor_name.
  //    motor_delta_steps -- the number of steps to move, either positive or negative
  //    speed_steps_per_sec -- a floating-point number specifying the speed in positive steps per second.
  // For example, calling motor_move_relative(&motor_a, -1000, 100) will make motor_a move -1000 steps, at a velocity of 100 steps per second.
  motor_set_velocity(motor_target, speed_steps_per_sec);
  motor_target->motor_target_position += motor_delta_steps;
}

void motor_move_relative_synchronous(struct motor *motor_target, long motor_delta_steps, long virtual_delta_steps, float virtual_speed_steps_per_sec){
  // Moves a motor by a relative number of steps, in sync with other motors.
  //    motor_target -- a pointer ot the target motor. You should call this with &motor_name.
  //    motor_delta_steps -- the number of steps to move, either positive or negative
  //    virtual_delta_steps -- the number of steps that the virtual motor moves. positive value only.
  //    virtual_speed_steps_per_sec_abs -- The speed at which the virtual motor will move.

  // Synchronization between motors is achieved thru the use of a _virtual motor_. We synchronize each real motor to the same virtual motor, and then all the real
  // motors are synchronized.
  //
  // For example, let's pretend we have motor_a and motor_b that we want to move 100 and 140 steps, respectively. We could create a virtual motor that moves 172 steps, which is
  // the diagonal distance between 100 and 140 steps (sqrt(100^2 + 140^2)). And perhaps we want to traverse that diagonal distance at a rate of 200 steps/sec.
  // We would make the following function calls:
  //  motor_move_relative_synchronous(&motor_a, 100, 172, 200);
  //  motor_move_relative_synchronous(&motor_b, 140, 172, 200);
  // 
  // This would cause each motor to move in sync with the virtual motor, and thus move in sync with each other. The velocity of each motor would be the virtual motor's
  // velocity times (motor_delta_steps/virtual_delta_steps).
  //
  // The algorithm behind this is called the "Bresenham Line Drawing Algorithm" with a "virtual major axis".
  //
  // NOTE: This function takes a while to run, meaning that calling it repeatedly for multiple motors does affect their synchronization. We've measured that for
  // three motors, the synchronization error is about 26ms. Future work includes writing an improved version that accepts multiple motors at once for tighter synchronization.
  
  // 1. we figure out how fast the virtual motor is going, as a fraction of the step generator interrupt rate
  const float interrupt_time_seconds = (float) STEP_GEN_TICK_PERIOD_US / 1000000.0;
  float steps_per_interrupt = fabs(virtual_speed_steps_per_sec) * interrupt_time_seconds;
  if(steps_per_interrupt>1.0){ //we can only do one step per interrupt
    //cap the velocity at 1 step per step generator interrupt
    steps_per_interrupt = 1.0;
  }

  //2. temporarily set the target motor velocity to zero, so we can safely make changes
  motor_target->motor_accumulator_velocity = 0;

  //3. Set the target motor position, while velocity is zero
  motor_target->motor_target_position += motor_delta_steps;

  //4.  Set the accumulator maximum value, based on the virtual motor number of steps and speed.
  motor_target->motor_accumulator_threshold = (long)(virtual_delta_steps/steps_per_interrupt);

  //5. we set the target motor velocity to its number of steps.
  motor_target->motor_accumulator_velocity = motor_delta_steps;
}

uint8_t motor_is_moving(struct motor *motor_target){
  // Returns 1 if the target motor is moving, otherwise 0 if idle.
  if(motor_target->motor_target_position != motor_target->motor_current_position){
    return 1;
  }else{
    return 0;
  }
}

uint8_t motor_any_moving(){
  // Returns 1 if any motor is moving, otherwise 0 if idle.
  int motor_index;
  for(motor_index = 0; motor_index < MOTOR_COUNT; motor_index ++){
    if(motor_is_moving(all_motors[motor_index])){
      return 1;
    };
  }
  return 0;
}

// --- INTERNAL USE MOTOR FUNCTIONS ---
// These functions help make the code above cleaner. You shouldn't need to call them directly, but take a look to see how they work!

void motor_set_velocity(struct motor *motor_target, float velocity_steps_per_sec){
  // internal function that sets the velocity of the motor, but doesn't change the target position.
  //    motor_target -- a pointer to the target motor. You should call this with &motor_name.
  //    velocity_steps_per_sec -- a floating-point number specifying the velocity in steps per second. Can be positive or negative.

  const float interrupt_time_seconds = (float) STEP_GEN_TICK_PERIOD_US / 1000000.0; //seconds between step generator interrupt
  float steps_per_interrupt = fabs(velocity_steps_per_sec) * interrupt_time_seconds; //steps per interrupt
  if(steps_per_interrupt>1.0){ //we can only do one step per interrupt
    //cap the velocity at 1 step per step generator interrupt
    steps_per_interrupt = 1.0;
  }
  
  // set the motor velocity
  motor_target->motor_accumulator_threshold = MOTOR_ACCUMULATOR_THRESHOLD_DEFAULT;
  motor_target->motor_accumulator_velocity = (long)((float)MOTOR_ACCUMULATOR_THRESHOLD_DEFAULT * steps_per_interrupt);
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
  // This is where the magic happens. The ISR (interrupt service routine) gets called each time TIMER1 reaches the top value set set in the step_generator_start() function.

  int motor_index;
  for(motor_index = 0; motor_index < MOTOR_COUNT; motor_index ++){ //Loop over all motors

    struct motor *motor_target = all_motors[motor_index]; // create a reference to this target motor
    
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
      motor_target->motor_accumulator += motor_target->motor_accumulator_velocity; //increment the accumulator by the velocity
      if(motor_target->motor_accumulator >= motor_target->motor_accumulator_threshold){
        //we've _now_ exceeded the accumulator threshold, so take a step
        motor_step(motor_target, motor_delta_direction);
        motor_target->motor_accumulator -= motor_target->motor_accumulator_threshold;
      }
    }
  }

}

void motor_step(struct motor *motor_target, int step_direction){
  
  // Set the direction pin to the correct _electrical_ direction. This will respect the MOTOR_DIR_INVERT flag
  if(step_direction ^ motor_target->MOTOR_DIR_INVERT){ //electrically forwards
    *motor_target->motor_dir_PORT |= (1<<motor_target->motor_dir_pin_atmega);      
  }else{ //electrically reverse
    *motor_target->motor_dir_PORT &= ~(1<<motor_target->motor_dir_pin_atmega);
  }

  // Now, increment or decrement the motor position based on the _logical_ direction
  if(step_direction){
    motor_target->motor_current_position ++;
  }else{
    motor_target->motor_current_position --;
  }

  //Finally, actually take a step
  *motor_target->motor_step_PORT |= (1<<motor_target->motor_step_pin_atmega);
  *motor_target->motor_step_PORT &= ~(1<<motor_target->motor_step_pin_atmega);
}

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
  motor_target->motor_accumulator_threshold = MOTOR_ACCUMULATOR_THRESHOLD_DEFAULT;

  //3. Initialize step and direction pins as outputs
  pinMode(motor_target->MOTOR_STEP_PIN, OUTPUT);
  pinMode(motor_target->MOTOR_DIR_PIN, OUTPUT);
}

void motors_initialize(){
  // Initializes all motors
  int motor_index;
  for(motor_index = 0; motor_index < MOTOR_COUNT; motor_index ++){
    motor_initialize(all_motors[motor_index]);
  }
  step_generator_start();
}