#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define RADIO_CE 6
#define RADIO_CSN 4
#define BUZZER A3
#define LASER_ADC A1
#define PHOTODIODE A2
#define LASER_PWM 3
#define LED_GREEN 8
#define LED_RED 7

RF24 radio(RADIO_CE, RADIO_CSN);
const byte address[6] = "00001";
const char sender = 'A';

void setup() {
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(PHOTODIODE, INPUT);

  Serial.begin(9600);

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  // Selftest !?!?!?!
  for (int i = 0; i < 20; i++) {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_RED, HIGH);
    delay(100);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, LOW);
    delay(100);
  }

  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
}

void beep() {
  digitalWrite(BUZZER, HIGH);
  delay(20);
  digitalWrite(BUZZER, LOW);
}

void loop() {
  if (analogRead(PHOTODIODE) < 100) {
    radio.write(&sender, sizeof(sender));
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
    beep();
    delay(2000);
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
  }
}
