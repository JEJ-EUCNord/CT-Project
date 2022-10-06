#include <Wire.h>
#include <hd44780.h>                        // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h>  // i2c expander i/o class header

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define BUZZER A3
#define LASER_ADC A1
#define PHOTODIODE A2
#define LASER_PWM 3
#define LED_GREEN 8
#define LED_RED 7

hd44780_I2Cexp lcd;  // declare lcd object: auto locate & auto config expander chip

const int LCD_COLS = 16;
const int LCD_ROWS = 2;

RF24 radio(6, 4);  // CE, CSN
const byte address[6] = "00001";
char sender;

#define STATE_READY 0
#define STATE_RUNNING 1
int state;
unsigned long startTime;      // milliseconds
unsigned long stopTime;       // milliseconds
unsigned long nextLcdUpdate;  // milliseconds
unsigned long currentTime;    // milliseconds
float runTime;                // seconds
float elapsedTime;            // seconds

void setup() {
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(PHOTODIODE, INPUT);

  Serial.begin(9600);

  int lcdStatus = lcd.begin(LCD_COLS, LCD_ROWS);
  if (lcdStatus)  // non zero status means it was unsuccesful
  {
    // hd44780 has a fatalError() routine that blinks an led if possible
    // begin() failed so blink error code using the onboard LED if possible
    hd44780::fatalError(lcdStatus);  // does not return
  }

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  // Selftest !?!?!?!
  lcd.print("CT LaserGate HTX");
  lcd.setCursor(0, 1);
  lcd.print("Version 0.1b");
  for (int i = 0; i < 20; i++) {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_RED, HIGH);
    delay(100);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, LOW);
    delay(100);
  }

  lcd.clear();
  state = STATE_READY;
  lcd.print("Ready");
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  startTime = 0;
  stopTime = 0;
  elapsedTime = 0;
  nextLcdUpdate = 0;
}

void beep() {
  digitalWrite(BUZZER, HIGH);
  delay(20);
  digitalWrite(BUZZER, LOW);
}

void loop() {
  if (radio.available())  //Looking for the data.
  {
    radio.read(&sender, sizeof(sender));  //Reading the data
    if (sender == 'A') {
      startTime = millis();
      stopTime = 0;
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_GREEN, LOW);
      beep();
      lcd.setCursor(0, 0);
      lcd.print("Running :       ");
      elapsedTime = 0;
      state = STATE_RUNNING;
      //int dummy = analogRead(PHOTODIODE); // This will fuck up futher radio com, dont know why???
    }
    Serial.println(sender);
  }

  if (state == STATE_RUNNING) {
    if (digitalRead(PHOTODIODE) == LOW) {
      stopTime = millis();
      runTime = (stopTime - startTime) / 1000.0;
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_GREEN, HIGH);
      beep();
      lcd.setCursor(0, 1);
      lcd.print("Time    :       ");
      lcd.setCursor(10, 1);
      if (runTime < 10.0) {
        lcd.print(" ");
      }
      lcd.print(runTime);
      lcd.setCursor(0, 0);
      lcd.print("Ready           ");
      state = STATE_READY;
    } else {
      currentTime = millis();
      if (nextLcdUpdate < currentTime) {
        elapsedTime = (currentTime - startTime) / 1000.0;
        if (elapsedTime < 90.0) {
          lcd.setCursor(10, 0);
          if (elapsedTime < 10.0) {
            lcd.print(" ");
          }
          lcd.print(elapsedTime);
          nextLcdUpdate = currentTime + 383;  // This is af magic number
        } else {
          digitalWrite(LED_RED, LOW);
          digitalWrite(LED_GREEN, HIGH);
          beep();
          lcd.setCursor(0, 0);
          lcd.print("Ready           ");
          state = STATE_READY;
        }
      }
    }
  }

  delay(5);
}