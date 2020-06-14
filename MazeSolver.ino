#include "Robot.h"

//Setup Arduino
void setup() {Robot::getInstance(WIDTH, HEIGHT);}

//Code to loop
void loop() {if (Robot::getInstance()->run()) while(true);}

