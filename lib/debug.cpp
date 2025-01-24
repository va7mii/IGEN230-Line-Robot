




//Robot code to run sensor array


#include <Arduino.h>

const int pins[] = {26, 25, 35, 34,A0, 33};
int numPins = sizeof(pins) / sizeof(int);

void setup () {
  Serial.begin(115200);

  for (int pin = 0; pin < numPins; pin++) {
    pinMode(pins[pin], INPUT);
  }
}

void loop () {
  for (int pin = 0; pin < numPins; pin++) {
      Serial.print("pin");
      Serial.print(pins[pin]);
      Serial.print(" = ");
      Serial.print(analogRead(pins[pin]));
      Serial.print(", ");
  }

  Serial.println("");

  delay(500);

}