
// Makes use of the Adafruit NeoPixel library and the Teensy Audio Library

#include <Adafruit_NeoPixel.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <HSBColor.h>


// Define pin and pixel number for LEDs
#define PIN            2
#define NUMPIXELS      20

//#define LIMIT 0.5       //Sound levels over this will change the colour
#define HOLDTIME 500    //How long to hold the sound level
#define SPEEDLIMIT 1  //Max blinking speed
#define SPEEDLOW 0.02 //Min blinking speed
#define MICSENS 1.0     //Mic sensitivity


//setup the NeoPixel library, tell it how many pixels, and which pin to use.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Audio input
const int myInput = AUDIO_INPUT_MIC;

// Create the Audio components.  These should be created in the
// order data flows, inputs/sources -> processing -> outputs
AudioInputI2S          audioInput;
AudioAnalyzeFFT1024    myFFT;
AudioAnalyzePeak       peak;


// Connect audio connections
AudioConnection patchCord1(audioInput, 0, myFFT, 0);
AudioConnection patchCord2(audioInput, 0, peak, 0);


// Create an object to control the audio shield.
AudioControlSGTL5000 audioShield;

float red, green, blue;   // holds the brightness for each colour

float LIMIT = 0.5;
float peakVol;
float newVol, prevVol;
const float Pi = 3.14159;     // Steak and cheese
bool up;

float hue, saturation, brightness;
float targetHue, targetSat, targetBright;
int colour[3];

unsigned long holdStart;
float blinkSpeed;
float blinkVal;


int slidePin = A6;
int slideVal = 0;
int in[] = {0,40,340,900,1023};
int out[] = {0,1,2,3,4};
int potPin = A7;
int potVal = 0;
bool potHigh = true;
//unsigned int waitTime = 60;        // seconds to wait until change state
int state = 0;
int lastState = 4;

void setup() {

  Serial.begin(115200);
  Serial.println("Serial");

  pixels.begin(); // This initializes the NeoPixel library.
  Serial.println("pixels");

  AudioMemory(20);
  Serial.println("Audio Memory");

  // Enable the audio shield and set the output volume.
  audioShield.enable();
  audioShield.inputSelect(myInput);
  Serial.println("Shield");

  // Configure the window algorithm to use
  myFFT.windowFunction(AudioWindowHanning1024);
  Serial.println("FFT");

  saturation = 1.0;
  brightness = 0.3;

  for (int i = 0; i < 3; i++) {
    H2R_HSBtoRGBfloat(i * 0.3, saturation, brightness, colour);
    for (int j = 0; j < NUMPIXELS; j++) {
      pixels.setPixelColor(j, pixels.Color(colour[0], colour[1], colour[2]));
      pixels.show();
      delay(5);
    }
  }
  randomSeed(analogRead(0));
  hue = mapFloat(random(0, 100), 0, 100, 0.0, 1.0);
  
//  Serial.println(hue);


}
//-------------------------- end of setup  ----------------------------

//-------------------------- start of loop -----------------------------
void loop() {


  if (state == 0) {
    potVal = analogRead(potPin);
    brightness = mapFloat(potVal, 0, 1023, 0, 0.8);
    saturation = 0;
    //  Serial.println(brightness);

    H2R_HSBtoRGBfloat(hue, saturation, brightness, colour);
  }

  if (state == 1) {
    potVal = analogRead(potPin);
    LIMIT = mapFloat(potVal, 0, 1023, 0.95, 0.1);
//    Serial.println(potVal);
    if (peak.available()) {

      newVol = peak.read() * MICSENS;



      if (newVol > LIMIT) { // if volume is over the limit
        blinkSpeed += 0.005;
        if (newVol < prevVol) { // if volume is decreasing
          if (holdVal()) {        //if volume is being held
            newVol = prevVol;       //set the new volume to the previous volume
          }
          else {                //if volume is not being held
            newVol = lerp(prevVol, newVol, 0.04); // fade down
          }
        }
        else {              //if the volume is increasing
          holdStart = millis();  // set new holdStart time
        }
        hue += mapFloat(random(0, 1000), 0, 1000, 0.1, 0.3);
        if (hue > 1){
          hue = 0;
        }
        
      }
      else {   // if volume is under the limit
        blinkSpeed -= blinkSpeed * .001;

      }
    }

    breathe();


    //  Serial.print("holdVal:");
    //  Serial.print(holdVal());
    //  Serial.print(" blinkSpeed:");
    //  Serial.print(blinkSpeed);
    //  Serial.print(" newVol:");
    //  Serial.print(newVol);
    //  for(int i=0; i<newVol*20; i++){
    //    Serial.print("-");
    //  }
    //  Serial.println("|");



    prevVol = newVol;

    saturation = 1.0;
    H2R_HSBtoRGBfloat(hue, saturation, brightness, colour);


  }
  //--end of peak state

  if (state == 2) {
    potVal = analogRead(potPin);
    hue = mapFloat(potVal, 0, 1023, 0, 1.0);
    H2R_HSBtoRGBfloat(hue, saturation, brightness, colour);
  }

  if (state == 3) {
    potVal = analogRead(potPin);
    saturation = mapFloat(potVal, 0, 1023, 0, 1.0);
    Serial.println(saturation);
    H2R_HSBtoRGBfloat(hue, saturation, brightness, colour);
  }

  if (state == 4) {
    potVal = analogRead(potPin);
    brightness = mapFloat(potVal, 0, 1023, 0, 1.0);
    H2R_HSBtoRGBfloat(hue, saturation, brightness, colour);
  }

  stateChanger();
  // apply colour to the pixels
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(colour[0], colour[1], colour[2]));
  }
  pixels.show();


}

void stateChanger() {
  slideVal = analogRead(slidePin);
  state = multiMap(slideVal, in, out, 5);
//  Serial.println(state);
  Serial.println(slideVal);
}


void breathe() {

  if (blinkSpeed < SPEEDLOW) blinkSpeed = SPEEDLOW;
  else if (blinkSpeed > SPEEDLIMIT) blinkSpeed = SPEEDLIMIT;


  if (brightness < 0.2) { //When dim, change  fade direction to up
    brightness = 0.2;
    up = true;
  }
  else if (brightness > 1) { //When bright, change fade direction to down
    brightness = 1;
    up = false;
  }

  if (up) { //When fading up, add fade amount
    brightness += blinkSpeed * 0.05;
  }
  else {
    brightness -= blinkSpeed * 0.01; //When fading down, minus fade amount
  }

}

//fade function for titania state
float fade(float current, float target, float amount) {
  float result;
  if (current < target) {
    result = current + amount;
    if (result > target) {
      result = target;
    }
  }
  else if (current > target) {
    result = current - amount;
    if (result < target) {
      result = target;
    }
  }
  else if (current == target) {
    result = target;
  }
  return result;
}


//hold function
bool holdVal() {
  bool result;
  if (millis() < holdStart + HOLDTIME) result = true;
  else result = false;
  return result;
}

//map function
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//lerp function
float lerp(float v0, float v1, float t) {
  return (1 - t) * v0 + t * v1;
}

// note: the _in array should have increasing values
int multiMap(int val, int* _in, int* _out, uint8_t size)
{
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];
  if (val >= _in[size-1]) return _out[size-1];

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > _in[pos]) pos++;

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}


