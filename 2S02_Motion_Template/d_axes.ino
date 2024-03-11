// Interfacing directly with motors in the language of steps is great, but often we want to work in real units.
// This module introduces the concept of "axes", which may be single axes like your linear stage assignment, or
// multiple synchronized axes like corexy or a polar printer.

struct axis{
  const float AXIS_STEPS_PER_REV; //number of steps per revolution of the axis motor.
  const float AXIS_UNITS_PER_REV; //axis units (e.g. mm, radians) per revolution of the axis motor.
  struct motor* axis_motor; // a pointer to the motor driving this axis
};

struct axis axis_single = {.AXIS_STEPS_PER_REV = 3200, .AXIS_UNITS_PER_REV = 32, // 200 step/rev motor @ 1/16 microstepping, 32 mm / rev
                            .axis_motor = &motor_a}; //axis units is mm

// -- AXIS FUNCTIONS

void axis_move_at_velocity(struct axis *axis_target, float axis_velocity){
  //Moves the axis at a fixed velocity, provided in axis units per second
  float motor_velocity_steps_per_sec = axis_convert_units_to_motor_steps(axis_target, axis_velocity);
  motor_spin(axis_target->axis_motor, motor_velocity_steps_per_sec);
}

void axis_move_absolute(struct axis *axis_target, float axis_position, float axis_velocity){
  // moves the axis to a specified position
  long motor_steps = (long)axis_convert_units_to_motor_steps(axis_target, axis_position);
  float motor_velocity_steps_per_sec = axis_convert_units_to_motor_steps(axis_target, axis_velocity);
  motor_move_absolute(axis_target->axis_motor, motor_steps, motor_velocity_steps_per_sec);
}

void axis_move_relative(struct axis *axis_target, float axis_delta, float axis_velocity){
  // moves the axis by a specified amount
  long motor_steps = (long)axis_convert_units_to_motor_steps(axis_target, axis_delta);
  float motor_velocity_steps_per_sec = axis_convert_units_to_motor_steps(axis_target, axis_velocity);
  motor_move_relative(axis_target->axis_motor, motor_steps, motor_velocity_steps_per_sec);
}

// void two_axis_linear_move_absolute(struct axis *axis_one, struct axis *axis_two, float axis_one_position, float axis_two_position,, float linear_velocity){
//   // synchronously moves two axes to specified positions.
//   // velocity is interpreted as along the straight-line path from the start point to the end point.
// }

float axis_calculate_max_speed(struct axis *axis_target){
  //calculates the maximum speed of the axis, based on its parameters and the motor max step rate
  float max_revs_per_sec = ((float)STEP_GEN_MAX_RATE_STEPS_PER_SEC)/((float)axis_target->AXIS_STEPS_PER_REV);
  float max_axis_speed_units_per_sec = max_revs_per_sec * axis_target->AXIS_UNITS_PER_REV;
  return max_axis_speed_units_per_sec;
}
float axis_convert_units_to_motor_steps(struct axis *axis_target, float value_in_axis_units){
  // converts a value in axis units into motor steps.
  float motor_revs = value_in_axis_units / axis_target->AXIS_UNITS_PER_REV;
  long motor_steps = motor_revs * axis_target->AXIS_STEPS_PER_REV;
  return motor_steps;

}