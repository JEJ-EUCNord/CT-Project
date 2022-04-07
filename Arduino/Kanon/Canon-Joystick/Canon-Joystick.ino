//======================================================================
// Canon operated via joystick
//   - Steppers are controlled by
//       Hardware: A4988/DRV8825 (or similar)
//       Software: AccelStepper library
//                 https://www.airspayce.com/mikem/arduino/AccelStepper
//   - The "Azimuth Coordinate System"
//     https://en.wikipedia.org/wiki/Azimuth_coordinate_system
//     is used, where
//     - North is perpendicular to the baseline. (In practice: the angle
//       bisector of the steppers outer positions)
//
//======================================================================


// ---------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------

// Pins
#define STEPPER_AZIMUTH_DIR_PIN    2
#define STEPPER_AZIMUTH_STEP_PIN   3
#define STEPPER_ALTITUDE_DIR_PIN   4
#define STEPPER_ALTITUDE_STEP_PIN  5
#define STEPPERS_ENABLE_PIN        6
#define JOYSTICK_SWITCH_PIN        A0
#define JOYSTICK_X_PIN             A1
#define JOYSTICK_Y_PIN             A2
#define SERVO_PIN                  7

// Steppers
#define STEPPERS_FULL_STEPS_PER_REVOLUTION  200  // Full steps
#define STEPPERS_MICRO_STEPPING               4  // 1, 2, 4, 8 or 16
#define STEPPERS_MAX_SPEED                 1000  // Steps per second
#define STEPPERS_MAX_ACCELERATION          1000  // Steps per second^2

// Match steppers and drivers with AccelStepper
#define STEPPER_AZIMUTH_DIRECTION_PIN_INVERTED  false
#define STEPPER_AZIMUTH_STEP_PIN_INVERTED       false
#define STEPPER_AZIMUTH_ENABLE_PIN_INVERTED     true
#define STEPPER_ALTITUDE_DIRECTION_PIN_INVERTED true
#define STEPPER_ALTITUDE_STEP_PIN_INVERTED      false
#define STEPPER_ALTITUDE_ENABLE_PIN_INVERTED    true

// Calibration angles in degrees (where do the steppers start).
#define AZIMUTH_CALIBRATION_ANGLE     0.0
#define ALTITUDE_CALIBRATION_ANGLE  -47.4 

// Min and max angles in degrees
#define AZIMUTH_MIN_ANGLE  -90.0
#define AZIMUTH_MAX_ANGLE   90.0
#define ALTITUDE_MIN_ANGLE   0.0
#define ALTITUDE_MAX_ANGLE  90.0

// Positions for the servo
#define SERVO_IDLE_ANGLE    90
#define SERVO_TRIGGER_ANGLE 5


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
#define AZIMUTH_MIN_POSITION  (Deg2Steps(AZIMUTH_MIN_ANGLE))
#define AZIMUTH_MAX_POSITION  (Deg2Steps(AZIMUTH_MAX_ANGLE))
#define ALTITUDE_MIN_POSITION (Deg2Steps(ALTITUDE_MIN_ANGLE))
#define ALTITUDE_MAX_POSITION (Deg2Steps(ALTITUDE_MAX_ANGLE))

// Convert from degrees to steps and vice versa
#define Deg2Steps(deg) \
  ((deg / 360.0) * STEPPERS_STEPS_PER_REVOLUTION)
 

// ---------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------

// Steppers
AccelStepper stepperAzimuth  = AccelStepper(
                                 AccelStepper::DRIVER,
                                 STEPPER_AZIMUTH_STEP_PIN,
                                 STEPPER_AZIMUTH_DIR_PIN
                               );
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


// ---------------------------------------------------------------------
// The real things
// ---------------------------------------------------------------------
void setup() {
  stepperAzimuth.setEnablePin(STEPPERS_ENABLE_PIN);
  stepperAzimuth.setPinsInverted(
    STEPPER_AZIMUTH_DIRECTION_PIN_INVERTED,
    STEPPER_AZIMUTH_STEP_PIN_INVERTED,
    STEPPER_AZIMUTH_ENABLE_PIN_INVERTED
  );
  stepperAzimuth.disableOutputs();

  stepperAltitude.setEnablePin(STEPPERS_ENABLE_PIN);
  stepperAltitude.setPinsInverted(
    STEPPER_ALTITUDE_DIRECTION_PIN_INVERTED,
    STEPPER_ALTITUDE_STEP_PIN_INVERTED,
    STEPPER_ALTITUDE_ENABLE_PIN_INVERTED
  );
  stepperAltitude.disableOutputs();

  stepperAzimuth.setMaxSpeed(STEPPERS_MAX_SPEED);
  stepperAzimuth.setAcceleration(STEPPERS_MAX_ACCELERATION);
  stepperAltitude.setMaxSpeed(STEPPERS_MAX_SPEED);
  stepperAltitude.setAcceleration(STEPPERS_MAX_ACCELERATION);

  stepperAzimuth.setCurrentPosition(Deg2Steps(AZIMUTH_CALIBRATION_ANGLE));
  stepperAltitude.setCurrentPosition(Deg2Steps(ALTITUDE_CALIBRATION_ANGLE));

  stepperAzimuth.enableOutputs();
  stepperAltitude.enableOutputs();

  stepperAzimuth.moveTo(0);
  stepperAltitude.moveTo(0);

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
    delay(400);
    servo.write(SERVO_IDLE_ANGLE);
//;  
  }
  newJoystickX = analogRead(JOYSTICK_X_PIN);
  newJoystickY = analogRead(JOYSTICK_Y_PIN);
  joystickX = abs(newJoystickX - joystickX) > 65 ? newJoystickX : joystickX;
  joystickY = abs(newJoystickY - joystickY) > 65 ? newJoystickY : joystickY;  
  stepperAzimuth.moveTo(map(joystickX, 0, 1023, AZIMUTH_MIN_POSITION, AZIMUTH_MAX_POSITION));
  stepperAltitude.moveTo(map(joystickY, 0, 1023, ALTITUDE_MIN_POSITION, ALTITUDE_MAX_POSITION));
  stepperAzimuth.run();
  stepperAltitude.run();
}
