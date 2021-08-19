//=========================================================================
// Canon operated via joystick
//=========================================================================

// ------------------------------------------------------------------------
// Includes
// ------------------------------------------------------------------------
#include <AccelStepper.h>  // Steppers

 
// ------------------------------------------------------------------------
// Defines
// ------------------------------------------------------------------------

// Pins
#define STEPPER_HORIZONTAL_DIR_PIN  2
#define STEPPER_HORIZONTAL_STEP_PIN 3
#define STEPPER_VERTICAL_DIR_PIN    4
#define STEPPER_VERTICAL_STEP_PIN   5   

// Steppers
#define STEPPERS_STEPS_PER_REVOLUTION  200  // Steps per revolution
#define STEPPERS_MICRO_STEPPING          8  // 1, 2, 4, 8 or 16
#define STEPPERS_MAX_SPEED            1000  // Steps per second

// ------------------------------------------------------------------------
// Constants and variables
// ------------------------------------------------------------------------
const int stepsPerRevolution   = STEPPERS_STEPS_PER_REVOLUTION 
                                                 * STEPPERS_MICRO_STEPPING;
AccelStepper stepperHorizontal = AccelStepper(AccelStepper::DRIVER, 
                                              STEPPER_HORIZONTAL_STEP_PIN, 
                                              STEPPER_HORIZONTAL_DIR_PIN);
AccelStepper stepperVertical   = AccelStepper(AccelStepper::DRIVER, 
                                              STEPPER_VERTICAL_STEP_PIN, 
                                              STEPPER_VERTICAL_DIR_PIN);


// ------------------------------------------------------------------------
// Setup
// ------------------------------------------------------------------------
void setup() {
  stepperHorizontal.setMaxSpeed(STEPPERS_MAX_SPEED);
  stepperVertical.setMaxSpeed(STEPPERS_MAX_SPEED);
  
  stepperHorizontal.setSpeed(1000);
}

// ------------------------------------------------------------------------
// Loop
// ------------------------------------------------------------------------
void loop() {
  stepperHorizontal.runSpeed();
}
