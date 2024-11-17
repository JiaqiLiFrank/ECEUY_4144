#include <Adafruit_CircuitPlayground.h>

const int BUFFER_SIZE = 50;   // Number of data points to record
const int TOLERANCE = 2;      // Allowed tolerance for data comparison

float xStored[BUFFER_SIZE], 
      yStored[BUFFER_SIZE], 
      zStored[BUFFER_SIZE];
      
float xTest[BUFFER_SIZE], 
      yTest[BUFFER_SIZE], 
      zTest[BUFFER_SIZE];

int storeIndex = 0, 
    testIndex = 0;

bool storeComplete = false;

void setup() {
  Serial.begin(115200);
  CircuitPlayground.begin();
  CircuitPlayground.playTone(440,500);    // output a 440 Hz sound for a tenth of a second
}

void successLight(){
  CircuitPlayground.setPixelColor(4,0,255,0);
  delay(1000);
  CircuitPlayground.setPixelColor(4, 0x000000);
}

void unsuccessfulLight(){
  for(int i=0; i<4; i++){
    CircuitPlayground.setPixelColor(4,255,0,0);
    delay(100);
    CircuitPlayground.setPixelColor(4, 0x000000);
    delay(100);
  }
}

void recordingLightON(){
  CircuitPlayground.setPixelColor(5,0,0,255);
}

void compareData() {
  int matchCount = 0;

  for (int i = 0; i < BUFFER_SIZE; i++) {
    float diffX = abs(xTest[i] - xStored[i]);
    float diffY = abs(yTest[i] - yStored[i]);
    float diffZ = abs(zTest[i] - zStored[i]);

    if (diffX <= TOLERANCE && 
        diffY <= TOLERANCE && 
        diffZ <= TOLERANCE){
          matchCount++;
          }
  }

  if (matchCount >= 40){
    successLight();
  }else{
    unsuccessfulLight();
  }
  
}

void recordingKey(){
  recordingLightON();
  
  for (int i = 0; i < BUFFER_SIZE; i++) {
    xStored[i] = CircuitPlayground.motionX();
    yStored[i] = CircuitPlayground.motionY();
    zStored[i] = CircuitPlayground.motionZ();

    delay(50);

  }

  CircuitPlayground.setPixelColor(5, 0x000000);
  successLight();
  storeComplete = true;
}

void recordingGesture(){
  recordingLightON();

  for (int i = 0; i < BUFFER_SIZE; i++) {

    xTest[i] = CircuitPlayground.motionX();
    yTest[i] = CircuitPlayground.motionY();
    zTest[i] = CircuitPlayground.motionZ();

    delay(50);

    CircuitPlayground.setPixelColor(5, 0x000000);
    compareData();
  }
}

void loop() {
  if (CircuitPlayground.leftButton()) {
    recordingKey();
  }

  if (CircuitPlayground.rightButton() && storeComplete) {
    recordingGesture();
  }
}
