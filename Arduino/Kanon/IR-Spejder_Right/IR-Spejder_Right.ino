//======================================================================
// Scout controlled via I2C
//   - Steppers are controlled by
//       Hardware: A4988/DRV8825 (or similar)
//       Software: AccelStepper library
//                 https://www.airspayce.com/mikem/arduino/AccelStepper
//
//======================================================================


// ---------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------

// Pins
#define STEPPER_AZIMUTH_DIR_PIN    A0
#define STEPPER_AZIMUTH_STEP_PIN   A1
#define STEPPERS_ENABLE_PIN        A2
#define SCAN_PIN                   A3

// Steppers
#define STEPPERS_FULL_STEPS_PER_REVOLUTION  200  // Full steps
#define STEPPERS_MICRO_STEPPING               4  // 1, 2, 4, 8 or 16
#define STEPPERS_MAX_SPEED                 1000  // Steps per second
#define STEPPERS_MAX_ACCELERATION          1000  // Steps per second^2

// Match steppers and drivers with AccelStepper
#define STEPPER_AZIMUTH_DIRECTION_PIN_INVERTED  true
#define STEPPER_AZIMUTH_STEP_PIN_INVERTED       false
#define STEPPER_AZIMUTH_ENABLE_PIN_INVERTED     true

// Calibration angles in degrees (where do the steppers start).
#define AZIMUTH_CALIBRATION_ANGLE     0.0

// Min and max angles in degrees
#define AZIMUTH_MIN_ANGLE   0.0
#define AZIMUTH_MAX_ANGLE  90.0

enum states {
  WAITING,
  SCAN,
  SCANNING
};


// ---------------------------------------------------------------------
// Libraries
// ---------------------------------------------------------------------
#include <AccelStepper.h>   // Steppers
#include <Wire.h>



// ---------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------

// Number of actual steps per revolution when using microstepping
#define STEPPERS_STEPS_PER_REVOLUTION \
  (STEPPERS_FULL_STEPS_PER_REVOLUTION * STEPPERS_MICRO_STEPPING)

// Min and max angles in steps
#define AZIMUTH_MIN_POSITION  (Deg2Steps(AZIMUTH_MIN_ANGLE))
#define AZIMUTH_MAX_POSITION  (Deg2Steps(AZIMUTH_MAX_ANGLE))

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

int maxHits;
int maxStepsFirst;
int maxStepsLast;

int state;


void setup() {
  stepperAzimuth.setEnablePin(STEPPERS_ENABLE_PIN);
  stepperAzimuth.setPinsInverted(
    STEPPER_AZIMUTH_DIRECTION_PIN_INVERTED,
    STEPPER_AZIMUTH_STEP_PIN_INVERTED,
    STEPPER_AZIMUTH_ENABLE_PIN_INVERTED
  );
  stepperAzimuth.disableOutputs();

  stepperAzimuth.setMaxSpeed(STEPPERS_MAX_SPEED);
  stepperAzimuth.setAcceleration(STEPPERS_MAX_ACCELERATION);

  stepperAzimuth.setCurrentPosition(Deg2Steps(AZIMUTH_CALIBRATION_ANGLE));

  stepperAzimuth.enableOutputs();

  stepperAzimuth.moveTo(0);

  Serial.begin(9600);
  Wire.begin(5);                // join i2c bus with address
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event

  state = WAITING;
}


void loop() {
  delay(100);
  if (state == SCAN) {
    state = SCANNING;
    Scan();
    state = WAITING;
  }
}

void receiveEvent(int howMany)
{
  while (1 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read();    // receive byte as a character
    Serial.print(c);         // print the character
  }
  char c = Wire.read();
  if (c == 's') {
    Serial.println("Yeps");         // print the character
    state = SCAN;
  }
}

void requestEvent()
{
  Wire.write((maxStepsFirst + maxStepsLast) / 2);
}




byte Scan() {
  maxHits = 0;
  maxStepsFirst = 0;
  maxStepsLast = 0;
  stepperAzimuth.runToNewPosition(0);
  for (int step = 0; step < Deg2Steps(90); step++) {
    stepperAzimuth.runToNewPosition(step);
    ReadIR(step);
  }
  stepperAzimuth.runToNewPosition((maxStepsFirst + maxStepsLast) / 2);
  return ((maxStepsFirst + maxStepsLast) / 2);
}

void ReadIR(int s) {
  // Take 10 readings
  int hits = 0;
  for (int n = 0; n < 10; n++) {
    hits += digitalRead(SCAN_PIN) == LOW ? 1 : 0;
  }
  if (hits > maxHits) {
    maxHits = hits;
    maxStepsFirst = s;
  }
  if (hits == maxHits) {
    maxStepsLast = s;
  }
}
