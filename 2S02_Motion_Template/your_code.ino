
void your_setup(){
  // put your setup code here, to run once:
  // motor_spin(&motor_a, -100);

}
void your_loop(){
  // put your code here, to run in a loop forever
    while(motor_any_moving()){
      
    }
    delay(1000);
    motor_move_relative_synchronous(&motor_a, 1000, 1000, 100);
    motor_move_relative_synchronous(&motor_b, 500, 1000, 100);
    motor_move_relative_synchronous(&motor_c, 273, 1000, 100);
}