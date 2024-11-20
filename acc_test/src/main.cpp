#include <Adafruit_CircuitPlayground.h>

void setup(){
  CircuitPlayground.begin();
  Serial.begin(115200);
}

void loop(){
  float x = CircuitPlayground.motionX();
  float y = CircuitPlayground.motionY();
  float z = CircuitPlayground.motionZ();

  float pos = sqrt((x*x+y*y+z*z));

  Serial.print(">pos:");
  Serial.println(pos);
  delay(20);
}