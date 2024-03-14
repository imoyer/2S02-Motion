// 2.S02 MOTION CONTROL TEMPLATE
//
// ** PUT YOUR CODE IN THE YOUR_CODE TAB**
//
// Any code in the below setup and loop functions will have access to functions in the other tabs, but NOT variables.
// This is why we suggest you put your code in the last tab, named "your_code". We've put secondary setup and loop functions there for you to use.
//
// There is a way around this using "header files," but to spare you this additional complexity for now, we're using this approach. The reason it works
// is because the arduino compiler works on the tabs in alphabetical order, so tabs can only "see" variables and functions in tabs that preceed them. The
// exception to this is the first tab named after the project, which as we said before, can access functions in all the tabs but not variables.

void setup() {
  motors_initialize();
  your_setup();
}

void loop() {
  your_loop();
}
