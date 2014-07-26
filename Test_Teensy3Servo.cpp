#include "WProgram.h"
#include <Teensy3Servo.h>

// Called from the powerup interrupt servicing routine.
int main(void)
{
    Serial.begin(38400); // For debugging output over the USB port
    delay(5000);
    Serial.printf("Starting Teensy3Servo Test\n\r");

    Teensy3Servo::InitOut(9);
    Teensy3Servo::InitIn(5);

    while (true) {
	for (int i = -180; i < 180; i+=10) {
          delay(500);
	  Teensy3Servo::Set(9,i);
	  delay(50);
	  Serial.printf("Set %04d\n\r", i);
	  Serial.printf("Get %04d\n\r", Teensy3Servo::Get(5));
	}
	for (int i = 180; i > -180; i-=10) {
          delay(500);
	  Teensy3Servo::Set(9,i);
          delay(50);
	  Serial.printf("Set %04d\n\r", i);
	  Serial.printf("Get %04d\n\r", Teensy3Servo::Get(5));
	}
    }
    return 0; // Never reached.
}
