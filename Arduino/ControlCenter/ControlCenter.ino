#include <Wire.h>

float vinkelA;

void setup()
{
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(9600);
}

void loop()
{
  // Wait for keypress
  Serial.println("\nSend kommando for at starte");
  WaitForKeypress();

  // Find vinkel A
  Serial.print("Finder vinkel A: ");
  Wire.beginTransmission(4); // Send 's' to #4
  Wire.write('s');
  Wire.endTransmission();

  delay(8000);
  
  Wire.requestFrom(4, 1);
  vinkelA = (Wire.read() / 200.0) * 90.0;
  
  Serial.println(vinkelA);  
}

void WaitForKeypress() {
  while (!Serial.available()) {
    // wait and do nothing
  }
  delay(500);
  while (Serial.available() > 0) {
    Serial.read();  // read and accept anything
  }
}
