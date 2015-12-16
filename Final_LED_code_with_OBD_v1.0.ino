//#include <Arduino.h>
#include <Wire.h>
//#include <OBD.h>
#include <FastLED.h>
#include <microsmooth.h>
#include <MicroLCD.h>
uint16_t *ptr;
LCD_SH1106 lcd; // for SH1106 OLED module
//COBD obd;

// How many leds in your strips?
#define NUM_LEDS_SIDES 30
#define NUM_LEDS_CLUSTER 2

// Define the arrays of LO, MED, HI, FLASH pixels
#define NUM_LEDS_LOW 9
#define NUM_LEDS_MED 8
#define NUM_LEDS_HI 7
#define NUM_LEDS_FLASH 6

// Define data pins for LED strips
#define DATA_PIN_LH 11
#define DATA_PIN_RH 12
#define DATA_PIN_C 8
#define CLOCK_PIN_C 9

// Define button, switch, buzzer pins
#define MODE_PIN 5 //digital
#define BUTTON_PIN 4 //digital
//#define BUZZER_PIN 3 //digital

//Accelerometer Pins (analog)
const int xpin = 1;                  // x-axis of the accelerometer
const int ypin = 2;                  // y-axis
//const int zpin = 3;                  // z-axis (only on 3-axis models)
//#define AUDIO_IN_LH 0 //analog
#define AUDIO_IN_RH 7 //analog

// Variables will change:
int buttonPushCounter = 4;   // counter for the number of button presses in DYNAMIC mode
int buttonPushCounter2 = 5;   // counter for the number of button presses in STATIC mode
int buttonState = 0;         // current state of the button in DYNAMIC mode
int buttonState2 = 0;         // current state of the button in STATIC mode
int lastButtonState = 0;     // previous state of the button in DYNAMIC mode
int lastButtonState2 = 0;     // previous state of the button in STATIC mode
int modeState = 0;           // current state of the Mode Switch
int lastModeState = 0;        // previous state of Mode Switch
int arrayIndex = 0;          // index for dyanamic rainbow
int funcHue = 0;              // starting Hue for pulsing rainbow
int funcSat = 255;           // saturation for pulsing rainbow
int thisstep = 1;             // Hue increment per delay cycle for pulsing rainbow
int funcDelay = 90;          // delay cycle time for pulsing rainbow
int volume2 = 0;              // volume intensity placeholder

// Define Vehicle Redline (RPM)
#define REDLINE 7000
#define MAX_RPM 7500

// Define the array of leds
CRGB leds_LH[NUM_LEDS_SIDES];
CRGB leds_RH[NUM_LEDS_SIDES];
CRGB leds_C[NUM_LEDS_CLUSTER];

//Accelerometer 1G calibrations
int minValx = 280;
int maxValx = 420;
int minValy = 280;
int maxValy = 420;

const unsigned char PROGMEM bowtie_small [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80,
0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
0x80, 0x80, 0x80, 0x80, 0x80, 0xC0, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x03, 0x03, 0x03, 0x03, 0x03,
0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x80, 0x80, 0x80, 0x80,
0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xE0, 0xFC, 0xFF, 0xFF, 0x7F,
0x1F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F,
0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F,
0x1F, 0x0F, 0x0F, 0x0F, 0x1F, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0x0F, 0x01, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xF0, 0xFC, 0xFF, 0xFF, 0x3F, 0x0F, 0x01, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x80, 0xF0, 0xFE, 0xFF, 0xFF, 0x3F, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x80, 0xF0, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xF1, 0xE0, 0xE0, 0xE0, 0xE0,
0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0,
0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0,
0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0,
0xE0, 0xF0, 0xFE, 0xFF, 0x7F, 0x1F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
0x07, 0x07, 0x07, 0x07, 0x07, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x80, 0x80, 0x80, 0x80, 0x80,
0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x07, 0x07, 0x07,
0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
0x07, 0x07, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F,
0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F,
0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void setup() {
  // begin lcd
  lcd.begin();
  // setup buttons, switches, buzzers
  pinMode(BUTTON_PIN, INPUT);
  pinMode(MODE_PIN, INPUT);
  //pinMode(BUZZER_PIN, OUTPUT);
  // setup LEDs
  FastLED.addLeds<WS2812B, DATA_PIN_LH, GRB>(leds_LH, NUM_LEDS_SIDES);
  FastLED.addLeds<WS2812B, DATA_PIN_RH, GRB>(leds_RH, NUM_LEDS_SIDES);
  FastLED.addLeds<LPD8806, DATA_PIN_C, CLOCK_PIN_C, GRB>(leds_C, NUM_LEDS_CLUSTER);
  ptr = ms_init(EMA);
  Serial.begin(9600);
  //bootscreen
  lcd.clear();
  lcd.setCursor(8, 0);
  lcd.draw(bowtie_small, 112, 56);
  delay(2000);
  lcd.setCursor(95,60);
  lcd.setFontSize(FONT_SIZE_SMALL);
  lcd.println("Init");
  delay(2000);
  lcd.clear();
  // start communication with OBD-II UART adapter
//obd.begin();
  // initiate OBD-II connection until success
//while (!obd.init());
}

void singleColor(int singleHue, int singleSat, int singleBrightness)
{
  // First, clear the existing led values
  FastLED.clear();

  //set different colors for the different RPM ranges
  for (int led = 0; led < NUM_LEDS_SIDES; led++){
    leds_LH[led] = CHSV( singleHue, singleSat, singleBrightness);
    leds_RH[led] = CHSV( singleHue, singleSat, singleBrightness);}

  FastLED.show();
}

/*void rainbowLoop(int funcHue, int funcSat, int thisstep, int funcDelay)         //-m3-LOOP HSV RAINBOW
{
arrayIndex++;
funcHue = funcHue + thisstep;
if (arrayIndex >= NUM_LEDS_SIDES-1) {
  arrayIndex = 0;
}
if (funcHue > 255) {
  funcHue = 0;
}
leds_LH[arrayIndex] = CHSV(funcHue, funcSat, 255);
leds_RH[arrayIndex] = CHSV(funcHue, funcSat, 255);
LEDS.show();
delay(funcDelay);
}*/

/*void rainbowPulse(int funcHue, int funcSat, int thisstep, int funcDelay)         //-m3-PULSE HSV RAINBOW
{
funcHue = funcHue + thisstep;
if (funcHue > 255) funcHue = 0;

for (int led = 0; led < NUM_LEDS_SIDES; led++){
leds_LH[led] = CHSV(funcHue, funcSat, 255);
leds_RH[led] = CHSV(funcHue, funcSat, 255);}
LEDS.show();
delay(funcDelay);
}*/

void loop() {

//LEDs*************************************************************************************
// read MODE_PIN
modeState = digitalRead(MODE_PIN);
if (modeState != lastModeState) lcd.clear();
lastModeState = modeState;

//DYNAMIC MODE
if (modeState == LOW){
// read the pushbutton input pin:
  buttonState = digitalRead(BUTTON_PIN);

// compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      // if the current state is HIGH then the button
      // wend from off to on:
      buttonPushCounter++;
      lcd.clear();
    }
  }
// save the current state as the last state,
//for next time through the loop
  lastButtonState = buttonState;

  if (buttonPushCounter == 7) buttonPushCounter = 2;

//RPM and Engine load performance mode
/*if (buttonPushCounter == 1)
{
  //Set array maps for RPM vs LED# and Engine Load vs Brightness
  int RPM;
  obd.read(PID_RPM, RPM);
  lcd.setFontSize(FONT_SIZE_MEDIUM);
  lcd.setCursor(0,10);
  lcd.print("RPM: ");
  lcd.print(RPM);

  int EngLoad;
  obd.read(PID_ENGINE_LOAD, EngLoad);
  lcd.setFontSize(FONT_SIZE_MEDIUM);
  lcd.setCursor(64,10);
  lcd.print("Load: ");
  lcd.print(EngLoad);

  int numLedsToLight = map(RPM, 0, MAX_RPM, 0, NUM_LEDS_SIDES);
  int Brightness = map(EngLoad, 50, 100, 0, 100);

Serial.print("RPM: ");
Serial.print(RPM);
Serial.print("\t");
Serial.print("| Load: ");
Serial.print(EngLoad);
Serial.print("\t");
Serial.print("| LEDs: ");
Serial.print(numLedsToLight);
Serial.print("\t");
Serial.print("| Brightness: ");
Serial.print(Brightness);
Serial.print("\n");

  FastLED.setBrightness(Brightness);

  // First, clear the existing led values
  FastLED.clear();

  //set different colors for the different RPM ranges
  for (int led = 0; led < numLedsToLight; led++) {
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED + NUM_LEDS_HI + NUM_LEDS_FLASH -1){
      leds_LH[led] = CRGB::White;}
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED + NUM_LEDS_HI + NUM_LEDS_FLASH -1){
      leds_RH[led] = CRGB::White;}
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED + NUM_LEDS_HI-1) leds_LH[led] = CRGB::Red;
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED + NUM_LEDS_HI-1) leds_RH[led] = CRGB::Red;
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED-1) leds_LH[led] = CRGB::Yellow;
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED-1) leds_RH[led] = CRGB::Yellow;
    if (led <= NUM_LEDS_LOW-1) leds_LH[led] = CRGB::Lime;
    if (led <= NUM_LEDS_LOW-1) leds_RH[led] = CRGB::Lime;}

  FastLED.show();

    //Shift Mode
      if (RPM > REDLINE){
      //Flash Cluster Lights
      //for (int cLed = 0; cLed < NUM_LEDS_CLUSTER; cLed++) {
      //  leds_C[cLed] = CRGB::Red;}
          // Turn the LED on, then pause
          FastLED.setBrightness(100);
          FastLED.show();
          delay(20);
          // Now turn the LED off, then pause
          FastLED.setBrightness(0);
          FastLED.show();
          delay(30);
      }
}*/

//fore-aft acceleration mode
if (buttonPushCounter == 2)
{
//Get fore-aft acceleration from accelerometer
  int xRead = analogRead(xpin);  //read from xpin
  float xGs = map(xRead, minValx, maxValx, -1000, 1000); //assign analog reading to 10^-3 Gs
  float xGsDisplay = xGs/1000;

  lcd.setFontSize(FONT_SIZE_MEDIUM);
  lcd.setCursor(25,10);
  lcd.println(xGsDisplay);
  lcd.setCursor(80,10);
  lcd.println(" Gs");
  lcd.setFontSize(FONT_SIZE_SMALL);
  lcd.println();
  lcd.println();
  lcd.println();
  lcd.setCursor(50,40);
  lcd.print("D2: FA Accel");
  delay(50);

  FastLED.setBrightness(80);

//Set array for acceleration magnitude
  if (xGs >= 0){
    FastLED.clear();
    int NUM_LEDS_UNFLTD = map(abs(xGs), 0, 800, 0, NUM_LEDS_SIDES);
    int numLedsToLight = ema_filter(NUM_LEDS_UNFLTD, ptr);
    if (numLedsToLight > 30) numLedsToLight = 30;

/*Serial.print(xRead);
Serial.print("\t");
Serial.print(xGs);
Serial.print("\t");
Serial.print(NUM_LEDS_UNFLTD);
Serial.print("\t");
Serial.print(numLedsToLight);
Serial.print("\n");
delay(200);*/

//Set green for positive acceleration
  for (int led = 0; led < numLedsToLight; led++) {
    leds_LH[led] = CRGB::Lime;
    leds_RH[led] = CRGB::Lime;}
  }

//Set array for acceleration magnitude - RH LEDs
  if (xGs < 0){
    FastLED.clear();
    int NUM_LEDS_UNFLTD = map(abs(xGs), 0, 800, 0, NUM_LEDS_SIDES);
    int numLedsToLight = ema_filter(NUM_LEDS_UNFLTD, ptr);
    if (numLedsToLight > 30) numLedsToLight = 30;

/*Serial.print(xRead);
Serial.print("\t");
Serial.print(xGs);
Serial.print("\t");
Serial.print(NUM_LEDS_UNFLTD);
Serial.print("\t");
Serial.print(numLedsToLight);
Serial.print("\n");
delay(200);*/

//Set red for negative acceleration
  for (int led = 0; led < numLedsToLight; led++) {
    leds_LH[led] = CRGB::Red;
    leds_RH[led] = CRGB::Red;}
}
  FastLED.show();
}

//lateral acceleration mode
if (buttonPushCounter == 3)
{
//Get lateral acceleration from accelerometer
  int yRead = analogRead(ypin);  //read from ypin
  float yGs = map(yRead, minValy, maxValy, -1000, 1000); //assign analog reading to 10^-3 Gs
  float yGsDisplay = yGs/1000;

  lcd.setFontSize(FONT_SIZE_MEDIUM);
  lcd.setCursor(25,10);
  lcd.println(yGsDisplay);
  lcd.setCursor(80,10);
  lcd.println(" Gs");
  lcd.setFontSize(FONT_SIZE_SMALL);
  lcd.println();
  lcd.println();
  lcd.println();
  lcd.setCursor(50,40);
  lcd.print("D3: Lt Accel");
  delay(50);

  FastLED.setBrightness(80);

//Set array for acceleration magnitude - LH LEDs
  if (yGs >= 0){
    FastLED.clear();
    int NUM_LEDS_UNFLTD_LH = map(abs(yGs), 0, 800, 0, NUM_LEDS_SIDES);
    int numLedsToLight_LH = ema_filter(NUM_LEDS_UNFLTD_LH, ptr);
    if (numLedsToLight_LH > 30) numLedsToLight_LH = 30;
    int numLedsToLight_RH = 0;

/*Serial.print(yRead);
Serial.print("\t");
Serial.print(yGs);
Serial.print("\t");
Serial.print(NUM_LEDS_UNFLTD_LH);
Serial.print("\t");
Serial.print(numLedsToLight_LH);
Serial.print("\n");
delay(200);*/

//set different colors for the different RPM ranges - LH
  for (int led = 0; led < numLedsToLight_LH; led++) {
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED + NUM_LEDS_HI + NUM_LEDS_FLASH -1){
      leds_LH[led] = CRGB::Lime;}
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED + NUM_LEDS_HI-1) leds_LH[led] = CRGB::Orange;
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED-1) leds_LH[led] = CRGB::Purple;
    if (led <= NUM_LEDS_LOW-1) leds_LH[led] = CRGB::Blue;}

//set different colors for the different RPM ranges - RH
  for (int led = 0; led < numLedsToLight_RH; led++) {
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED + NUM_LEDS_HI + NUM_LEDS_FLASH -1){
      leds_RH[led] = CRGB::Lime;}
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED + NUM_LEDS_HI-1) leds_RH[led] = CRGB::Orange;
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED-1) leds_RH[led] = CRGB::Purple;
    if (led <= NUM_LEDS_LOW-1) leds_RH[led] = CRGB::Blue;}}

//Set array for acceleration magnitude - RH LEDs
  if (yGs < 0){
    FastLED.clear();
    int NUM_LEDS_UNFLTD_RH = map(abs(yGs), 0, 800, 0, NUM_LEDS_SIDES);
    int numLedsToLight_RH = ema_filter(NUM_LEDS_UNFLTD_RH, ptr);
    int numLedsToLight_LH = 0;
    if (numLedsToLight_RH > 30) numLedsToLight_RH = 30;

/*Serial.print(yRead);
Serial.print("\t");
Serial.print(yGs);
Serial.print("\t");
Serial.print(NUM_LEDS_UNFLTD_RH);
Serial.print("\t");
Serial.print(numLedsToLight_RH);
Serial.print("\n");
delay(200);*/

//set different colors for the different RPM ranges - LH
  for (int led = 0; led < numLedsToLight_LH; led++) {
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED + NUM_LEDS_HI + NUM_LEDS_FLASH -1){
      leds_LH[led] = CRGB::Lime;}
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED + NUM_LEDS_HI-1) leds_LH[led] = CRGB::Orange;
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED-1) leds_LH[led] = CRGB::Purple;
    if (led <= NUM_LEDS_LOW-1) leds_LH[led] = CRGB::Blue;}

//set different colors for the different RPM ranges - RH
  for (int led = 0; led < numLedsToLight_RH; led++) {
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED + NUM_LEDS_HI + NUM_LEDS_FLASH -1){
      leds_RH[led] = CRGB::Lime;}
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED + NUM_LEDS_HI-1) leds_RH[led] = CRGB::Orange;
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED-1) leds_RH[led] = CRGB::Purple;
    if (led <= NUM_LEDS_LOW-1) leds_RH[led] = CRGB::Blue;}
}
  FastLED.show();
}

//volume response mode
if (buttonPushCounter == 4)
{

  lcd.setFontSize(FONT_SIZE_MEDIUM);
  lcd.setCursor(44,10);
  lcd.println("Audio");
  lcd.setFontSize(FONT_SIZE_SMALL);
  lcd.println();
  lcd.println();
  lcd.println();
  lcd.setCursor(40,40);
  lcd.print("D4: Visualizer");

//Set array for media volume
  int volume1 = analogRead(AUDIO_IN_RH);
if (volume1 < volume2){
  volume2 = volume2*.975;}
else{
  volume2 = volume1*1.4;}
  int volumefilt2 = ema_filter(volume2, ptr);  
  int numLedsToLight = map(volumefilt2, 0, 70, 0, NUM_LEDS_SIDES); //testing using potentiometer

/*Serial.print(volume1);
Serial.print("\t");
Serial.print(volume2);
Serial.print("\t");
Serial.print(volumefilt1);
Serial.print("\t");
Serial.print(volumefilt2);
Serial.print("\n");*/

  FastLED.setBrightness(80);

  // First, clear the existing led values
  FastLED.clear();

  //set different colors for the different RPM ranges
  for (int led = 0; led < numLedsToLight; led++) {
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED + NUM_LEDS_HI + NUM_LEDS_FLASH -1){
      leds_LH[led] = CRGB::Orange;}
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED + NUM_LEDS_HI + NUM_LEDS_FLASH -1){
      leds_RH[led] = CRGB::Orange;}
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED + NUM_LEDS_HI-1) leds_LH[led] = CRGB::DarkGreen;
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED + NUM_LEDS_HI-1) leds_RH[led] = CRGB::DarkGreen;
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED-1) leds_LH[led].setHSV( 192, 255, 255);
    if (led <= NUM_LEDS_LOW + NUM_LEDS_MED-1) leds_RH[led].setHSV( 192, 255, 255);
    if (led <= NUM_LEDS_LOW-1) leds_LH[led] = CRGB::Blue;
    if (led <= NUM_LEDS_LOW-1) leds_RH[led] = CRGB::Blue;}

  FastLED.show();
}

//pulsing rainbow
if (buttonPushCounter == 5)
{

  lcd.setFontSize(FONT_SIZE_MEDIUM);
  lcd.setCursor(27,10);
  lcd.println("Show Mode");
  lcd.setFontSize(FONT_SIZE_SMALL);
  lcd.println();
  lcd.println();
  lcd.println();
  lcd.setCursor(45,40);
  lcd.print("D5: Show Mode");

funcHue = funcHue + thisstep;
if (funcHue > 255) funcHue = 0;

for (int led = 0; led < NUM_LEDS_SIDES; led++){
leds_LH[led] = CHSV(funcHue, funcSat, 255);
leds_RH[led] = CHSV(funcHue, funcSat, 255);}
LEDS.show();
delay(funcDelay);
}

//blackout
if (buttonPushCounter == 6) {

  lcd.setFontSize(FONT_SIZE_MEDIUM);
  lcd.setCursor(40,10);
  lcd.println();
  lcd.setFontSize(FONT_SIZE_SMALL);
  lcd.println();
  lcd.println();
  lcd.println();
  lcd.setCursor(55,40);
  lcd.print("Off");

singleColor(0, 0, 0);

}

}//END DYNAMIC MODE

//STATIC MODE
if (modeState == HIGH){
// read the pushbutton input pin:
  buttonState2 = digitalRead(BUTTON_PIN);

// compare the buttonState to its previous state
  if (buttonState2 != lastButtonState2) {
    // if the state has changed, increment the counter
    if (buttonState2 == HIGH) {
      // if the current state is HIGH then the button
      // wend from off to on:
      buttonPushCounter2++;
      lcd.clear();
    }
  }
// save the current state as the last state,
// for next time through the loop
  lastButtonState2 = buttonState2;
  if (buttonPushCounter2 == 6) buttonPushCounter2 = 1;

// static modes
if (buttonPushCounter2 == 1)
{
  lcd.setFontSize(FONT_SIZE_MEDIUM);
  lcd.setCursor(50,10);
  lcd.println("Red");
  lcd.setFontSize(FONT_SIZE_SMALL);
  lcd.println();
  lcd.println();
  lcd.println();
  lcd.setCursor(110,40);
  lcd.print("S1");

  singleColor(0,255,225); //Red
}

if (buttonPushCounter2 == 2)
{
  lcd.setFontSize(FONT_SIZE_MEDIUM);
  lcd.setCursor(46,10);
  lcd.println("Aqua");
  lcd.setFontSize(FONT_SIZE_SMALL);
  lcd.println();
  lcd.println();
  lcd.println();
  lcd.setCursor(110,40);
  lcd.print("S2");

  singleColor(135,255,225); //Aqua
}

if (buttonPushCounter2 == 3)
{
  lcd.setFontSize(FONT_SIZE_MEDIUM);
  lcd.setCursor(37,10);
  lcd.println("Purple");
  lcd.setFontSize(FONT_SIZE_SMALL);
  lcd.println();
  lcd.println();
  lcd.println();
  lcd.setCursor(110,40);
  lcd.print("S3");

  singleColor(192,255,225); //Purple
}

if (buttonPushCounter2 == 4)
{
  lcd.setFontSize(FONT_SIZE_MEDIUM);
  lcd.setCursor(40,10);
  lcd.println("Green");
  lcd.setFontSize(FONT_SIZE_SMALL);
  lcd.println();
  lcd.println();
  lcd.println();
  lcd.setCursor(110,40);
  lcd.print("S4");

  singleColor(96,255,225); //Green
}

if (buttonPushCounter2 == 5)
{
  lcd.setFontSize(FONT_SIZE_MEDIUM);
  lcd.setCursor(40,10);
  lcd.println();
  lcd.setFontSize(FONT_SIZE_SMALL);
  lcd.println();
  lcd.println();
  lcd.println();
  lcd.setCursor(55,40);
  lcd.print("Off");

  singleColor(0, 0, 0); //Black
}

}//END STATIC

//LEDs*******************************************************************************

}//END LOOP

//WIP SECTION***********
/*
 - add buzzer functionality
 - add lcd printouts for each mode
   - for dyanmic modes, readouts for variables
   - for static modes, color followed by delay then chevy logo (might not work in loop)
   - for setup - chevy logo while connecting/initializing
 - add audio pins and code
*/
