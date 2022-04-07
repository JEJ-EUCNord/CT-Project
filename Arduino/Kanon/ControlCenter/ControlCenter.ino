#include <Wire.h>

#define AB 104.5
#define AK  50.0

float vinkelA;
float vinkelB;
float vinkelK;
float afstand;

float vinkelC;


#define Deg2Rad(d) (d / 180.0 * PI)
#define Rad2Deg(r) (r / PI * 180.0)

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
  vinkelA = ScanA();
  Serial.println(vinkelA);  

  // Find vinkel B
  Serial.print("Finder vinkel B: ");
  vinkelB = ScanB();
  Serial.println(vinkelB);  

  // Beregn vinkel for kanon og afstand
//  Beregn();

//  Serial.print("Vinkel K er beregnet til:  ");
//  Serial.println(vinkelK);
//  Serial.print("Afstanden er beregnet til: ");
//  Serial.println(afstand);
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

float ScanA() {
  Wire.beginTransmission(4); // Send 's' to #4
  Wire.write('s');
  Wire.endTransmission();

  delay(8000);
  
  Wire.requestFrom(4, 1);
  vinkelA = (Wire.read() / 200.0) * 90.0;
  return vinkelA;
}

float ScanB() {
  Wire.beginTransmission(5); // Send 's' to #5
  Wire.write('s');
  Wire.endTransmission();

  delay(8000);
  
  Wire.requestFrom(5, 1);
  vinkelB = (Wire.read() / 200.0) * 90.0;
  return vinkelB;
}

void Beregn() {
  float vinkelC = 180-vinkelA-vinkelB;
  float AC = AB*sin(Deg2Rad(vinkelB))/sin(Deg2Rad(vinkelC));
}
