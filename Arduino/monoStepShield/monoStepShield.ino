// ---------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------
#define STEPPER_ALTITUDE_DIR_PIN   7
#define STEPPER_ALTITUDE_STEP_PIN  6

#define STEPPERS_ENABLE_PIN        2
#define JOYSTICK_SWITCH_PIN        A2
#define JOYSTICK_X_PIN             A0
#define JOYSTICK_Y_PIN             A1
#define SERVO_PIN                  9
#define POLOLU_MS1  3
#define POLOLU_MS2  4
#define POLOLU_MS3  5

#define DEADZONE 85
 
// Steppers
#define STEPPERS_FULL_STEPS_PER_REVOLUTION  200  // Full steps
#define STEPPERS_MICRO_STEPPING               8  // 1, 2, 4, 8 or 16
#define STEPPERS_MAX_SPEED                 1000  // Steps per second
#define STEPPERS_MAX_ACCELERATION          1000  // Steps per second^2

// Match steppers and drivers with AccelStepper
#define STEPPER_ALTITUDE_DIRECTION_PIN_INVERTED true
#define STEPPER_ALTITUDE_STEP_PIN_INVERTED      false
#define STEPPER_ALTITUDE_ENABLE_PIN_INVERTED    true

// Calibration angles in degrees (where do the steppers start).
//#define AZIMUTH_CALIBRATION_ANGLE     0.0
#define ALTITUDE_CALIBRATION_ANGLE  -47.4 
#define ALT_MIN_ANGLE -47.4

// Min and max angles in degrees
#define ALTITUDE_MIN_ANGLE   0.0
#define ALTITUDE_MAX_ANGLE  90.0

// Positions for the servo
#define SERVO_IDLE_ANGLE    90
#define SERVO_TRIGGER_ANGLE  5
// ---------------------------------------------------------------------
// Libraries        
// ---------------------------------------------------------------------
#include <AccelStepper.h>   // Steppers
#include <Servo.h>          // Servo
// ---------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------

// Number of actual steps per revolution when using microstepping
#define STEPPERS_STEPS_PER_REVOLUTION \
  (STEPPERS_FULL_STEPS_PER_REVOLUTION * STEPPERS_MICRO_STEPPING)

// Min and max angles in steps  
#define ALTITUDE_MIN_POSITION (Deg2Steps(ALTITUDE_MIN_ANGLE))
#define ALTITUDE_MAX_POSITION (Deg2Steps(ALTITUDE_MAX_ANGLE))

// Convert from degrees to steps and vice versa
#define Deg2Steps(deg) \
  ((deg / 360.0) * STEPPERS_STEPS_PER_REVOLUTION)
 
// ---------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------

// Steppers
AccelStepper stepperAltitude = AccelStepper(
                                 AccelStepper::DRIVER,
                                 STEPPER_ALTITUDE_STEP_PIN,
                                 STEPPER_ALTITUDE_DIR_PIN
                               );
// Joystick
long joystickX, newJoystickX;
long joystickY, newJoystickY;

// Servo
Servo servo;

float angle = 0;

// ---------------------------------------------------------------------
// The real things
// ---------------------------------------------------------------------
void setup() {

  stepperAltitude.setEnablePin(STEPPERS_ENABLE_PIN);
  stepperAltitude.setPinsInverted(
    STEPPER_ALTITUDE_DIRECTION_PIN_INVERTED,
    STEPPER_ALTITUDE_STEP_PIN_INVERTED,
    STEPPER_ALTITUDE_ENABLE_PIN_INVERTED
  );
  stepperAltitude.disableOutputs();

  stepperAltitude.setMaxSpeed(STEPPERS_MAX_SPEED);
  stepperAltitude.setAcceleration(STEPPERS_MAX_ACCELERATION);

  stepperAltitude.setCurrentPosition(Deg2Steps(ALTITUDE_CALIBRATION_ANGLE));
  //angle = ALTITUDE_CALIBRATION_ANGLE;

  stepperAltitude.enableOutputs();
  stepperAltitude.moveTo(0);

  /* SETTING POLOLU STEP SIZE
  MS1 MS2 MS3
  0   0   0  - Full step
  1   0   0  - Half step
  0   1   0  - Quarter step
  1   1   0  - Eighth step
  1   1   1  - Sixteenth step
  */

  // Setting Fullstep (also default, if not connected)
  pinMode(POLOLU_MS1, OUTPUT);
  pinMode(POLOLU_MS2, OUTPUT);
  pinMode(POLOLU_MS3, OUTPUT);
  digitalWrite(POLOLU_MS1, HIGH);
  digitalWrite(POLOLU_MS2, HIGH);
  digitalWrite(POLOLU_MS3, LOW);
  
  pinMode(JOYSTICK_X_PIN, INPUT);
  pinMode(JOYSTICK_Y_PIN, INPUT);
  pinMode(JOYSTICK_SWITCH_PIN, INPUT_PULLUP);
  joystickX = analogRead(JOYSTICK_X_PIN);
  joystickY = analogRead(JOYSTICK_Y_PIN);

  servo.attach(SERVO_PIN);
  servo.write(SERVO_IDLE_ANGLE);
}

void loop() {
  if (digitalRead(JOYSTICK_SWITCH_PIN) == LOW) {
    servo.write(SERVO_TRIGGER_ANGLE);
    delay(500);
    servo.write(SERVO_IDLE_ANGLE);
    delay(2500);  
  }
  joystickX = analogRead(JOYSTICK_X_PIN);

  if ( joystickX > (512+DEADZONE) ) {
    if ( angle > ALTITUDE_MIN_ANGLE ){
      angle -= 0.005;
    }
  } else if ( joystickX < (512-DEADZONE)) {
    if ( angle < ALTITUDE_MAX_ANGLE ){
      angle += 0.005;
    }
    
  }
  
  
  stepperAltitude.moveTo(  Deg2Steps( angle )    );
  stepperAltitude.run();
}
