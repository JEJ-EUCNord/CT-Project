/*
   -------------------------------------------------------------------------------------
   HX711_ADC
   Arduino library for HX711 24-Bit Analog-to-Digital Converter for Weight Scales
   Olav Kallhovd sept2017
   -------------------------------------------------------------------------------------
*/

/*
   This example file shows how to calibrate the load cell and optionally store the calibration
   value in EEPROM, and also how to change the value manually.
   The result value can then later be included in your project sketch or fetched from EEPROM.

   To implement calibration in your project sketch the simplified procedure is as follow:
       LoadCell.tare();
       //place known mass
       LoadCell.refreshDataSet();
       float newCalibrationValue = LoadCell.getNewCalibration(known_mass);
*/

#include <HX711_ADC.h>

//Jump Threshold
#define JUMP_THRESHOLD 110 // Vægt for detektering af hop

//pins:
#define HX711_dout 4 //mcu > HX711 dout pin
#define HX711_sck  5 //mcu > HX711 sck pin

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

// unsigned long myTime;
// int antal = 0;
float val, valMax;
#define LAGER_SIZE 100
float lager[LAGER_SIZE];
int   li;
bool  jumpDetected;
int   jumps;
int   samplesleft;

void setup() {
  Serial.begin(9600); delay(10);
  Serial.println();
  Serial.println("Starting...");

  LoadCell.begin();
  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(1.0); // user set calibration value (float), initial value 1.0 may be used for this sketch
    Serial.println("Startup is complete");
  }
  while (!LoadCell.update());
  calibrate(); //start calibration procedure

  li = 0;
  valMax = 0;
  jumpDetected = false;
  samplesleft = 0;
  jumps = 0;
}

void loop() {
  // Wait for new data from HX711
  while (!LoadCell.update())
    ;

  // Get the data
  val = LoadCell.getData();

  // Save the read value
  if (!jumpDetected || samplesleft > 0) {
    lager[li++] = val;
    if (li == LAGER_SIZE)
      li = 0;
    if (val > valMax && val > JUMP_THRESHOLD) {
      jumpDetected = true;
      samplesleft = LAGER_SIZE / 3;
    }
    if (samplesleft > 0)
      samplesleft--;
  }

  // set new max value
  valMax = max(val, valMax);

  // Check if ready for output and start over
  if (jumpDetected && samplesleft == 0) {
    Serial.println();
    Serial.print("Jump ");
    Serial.println(++jumps);
    for (int i = 0; i < LAGER_SIZE; i++) {
      Serial.println(lager[li++]);
      if (li == LAGER_SIZE)
        li = 0;
    }
    valMax = 0;
    jumpDetected = false;
  }
}

void calibrate() {
  Serial.println("***");
  Serial.println("Start calibration:");
  Serial.println("Place the load cell an a level stable surface.");
  Serial.println("Remove any load applied to the load cell.");
  Serial.println("Send 't' from serial monitor to set the tare offset.");

  boolean _resume = false;
  while (_resume == false) {
    LoadCell.update();
    if (Serial.available() > 0) {
      if (Serial.available() > 0) {
        char inByte = Serial.read();
        if (inByte == 't') LoadCell.tareNoDelay();
      }
    }
    if (LoadCell.getTareStatus() == true) {
      Serial.println("Tare complete");
      _resume = true;
    }
  }

  Serial.println("Now, place your known mass on the loadcell.");
  Serial.println("Then send the weight of this mass (i.e. 100.0) from serial monitor.");

  float known_mass = 0;
  _resume = false;
  while (_resume == false) {
    LoadCell.update();
    if (Serial.available() > 0) {
      known_mass = Serial.parseFloat();
      if (known_mass != 0) {
        Serial.print("Known mass is: ");
        Serial.println(known_mass);
        _resume = true;
      }
    }
  }

  LoadCell.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
  float newCalibrationValue = LoadCell.getNewCalibration(known_mass); //get the new calibration value

  Serial.print("New calibration value has been set to: ");
  Serial.print(newCalibrationValue);
  Serial.println(", use this as calibration value (calFactor) in your project sketch.");

  Serial.println("End calibration");
  Serial.println("***");
}
