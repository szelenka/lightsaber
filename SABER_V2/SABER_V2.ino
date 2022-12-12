//#include <Adafruit_NeoPixel_ZeroDMA.h>
#include <Adafruit_NeoPixel.h>
#include <AudioPlayer.h>
#include <Adafruit_DotStar.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "ArduinoLowPower.h"
#include <FlashStorage.h>
#include "ON_1S.h"
#include "OFF_06S.h"
#include "HUM_3SM.h"
#include "hit.h"
#include "swing.h"

// NEOPIXEL DMA
// using DMA mode frees up processor use and does not distort sound, however, there is no
// mechanisim to disable the library and set the neopixel pin back to an input, which means there
// is a higher current draw in sleep mode. For this reason I have switched to the standard neopixel
// lib and uses setPin() to disable the neopixel output reducing the sleep current. Also, the
// sound distortion produced is acceptable and actually quite good!


// this option does not playback sound, just uysed during development
//#define MUTE_MODE

// do not sleep when debuging / developing
//#define DEBUG_NO_SLEEP_MODE
//#define DEBUG_MODE

// this option uses pin change wake up, much faster power on than timed sleep mode!
#define USE_INTERRUPT_PIN

// this option uses internal DotStar (if provided)
//#define USE_DOTSTAR

// this option uses accel from LIS3DH sensor
#define USE_LIS3DH

// defines the strip update rate
#define FRAME_RATE 50

// defines the nuber of neopixels on one side of the strip. (actual count is twice this number)
#define STRIP_LED_COUNT 42

// boot delay useful when developing
#define BOOT_TIME_WAIT  30000 // mili seconds

// CUSTOMIZE SENSITIVITY HERE: smaller numbers = more sensitive to motion
#define HIT_THRESHOLD 250
#define SWING_THRESHOLD 125

// ************************************************************************
// device pins
// ************************************************************************
#define PIN_ANA_OUT     A0
#define PIN_NEOPIXEL    6
#define PIN_SPARE       2
#define PIN_AMP_SHDWM   1
#define PIN_WAKE        3
#define PIN_DUMMY       9

// ************************************************************************
// AUDIO SECTION
// ************************************************************************
/*
   WAV CONVERSION TO .h FILES
   https://bitluni.net/wp-content/uploads/2016/12/bin2h8bit.html
*/

#define WAV_FREQUENCEY  16000
#define WAV_OVERSAMPLE  0
#define SABER_ON_TIME (saberOnSize / (WAV_FREQUENCEY/1000)) // power up time in mS
#define SABER_OFF_TIME (saberOffSize / (WAV_FREQUENCEY/1000)) // power down time in mS

// ************************************************************************
// these values set the color and intesity. I have tuned these to keep below
// 650ma of power draw! This is due to the small wires used on the picoblade
// connector which have a mximum of 1000ma capacity.
// ************************************************************************
#define BLADE_COLOR_OFF (0x800000)
#define BLADE_COLOR_RED (0xA00000)
#define BLADE_COLOR_YEL (0x784200)
#define BLADE_COLOR_GRN (0x007200)
#define BLADE_COLOR_BLU (0x0000B0)
#define BLADE_COLOR_PUR (0x600060)
#define BLADE_COLOR_WHT (0x402855)

#define MAX_COLORS 6
const unsigned long saberColor[MAX_COLORS] = {BLADE_COLOR_RED, BLADE_COLOR_YEL, BLADE_COLOR_GRN, BLADE_COLOR_BLU, BLADE_COLOR_PUR, BLADE_COLOR_WHT};

// used by fast sin table
#define SIN_TABLE_SCALE 10  // fractions of a degree, e.g. 10 = 0.1 degrees per step tabel size = 360 * 10

// saber pixel array
unsigned long saberPixels[STRIP_LED_COUNT];

// saber pulse variables
#define WAVE_AMPLITUDE_RND (waveAmplitude) //(saberColorRndAmplitude[saberColIndex])
#define WAVE_FREQUENCY_MIN (20)
#define WAVE_FREQUENCY_RND (waveFrequency)
#define WAVE_PHASE_MIN     (-10)
#define WAVE_PHASE_RND     (wavePhase)


// ************************************************************************
// neopixel and dotstart strip
// ************************************************************************
Adafruit_NeoPixel strip(STRIP_LED_COUNT * 2, PIN_NEOPIXEL, NEO_GRB);
#ifdef USE_DOTSTAR
Adafruit_DotStar dotStar = Adafruit_DotStar(1, 7, 8, DOTSTAR_BGR);
#endif
#ifdef USE_LIS3DH
Adafruit_LIS3DH lis = Adafruit_LIS3DH();
#endif

// ************************************************************************
// variables and functions
// ************************************************************************
// The saber has four possible states it can be in
enum {saberIsOff, saberIsOpened, saberIsOn, saberIsClosed};
unsigned char saberState = saberIsOff;
#ifdef USE_LIS3DH
signed long acceleration = 0;
#endif

int   saberColIndex = 0;
int   waveFrequency = 25;
int   wavePhase = 10;
float waveAmplitude = 0.5;
float degrees = 0;
float sinTable[360 * SIN_TABLE_SCALE];
unsigned long lasttime = 0;

// track button state
unsigned long buttonPressedAt = 0;
bool buttonReleased = false;
byte buttonPrevState = 0;

void wakeUp(void);
void sleepSaber(void);
void powerUpSaber(void);
void powerDownSaber(void);
void powerDownDevice(void);
void programMode(void);
void playMotionSounds(void);
void checkPowerOffSequence(void);

// Reserve a portion of flash memory to store an "int" variable
// and call it "my_flash_store".
FlashStorage(colorFlashStore, int);


// ************************************************************************
// the setup routine runs once when you press reset:
// ************************************************************************
void setup()
{
  // initialize the digital pin as an output.
  pinMode(PIN_LED , OUTPUT);
  pinMode(PIN_AMP_SHDWM , OUTPUT);
  pinMode(PIN_WAKE , INPUT_PULLUP);
  pinMode(PIN_SPARE, INPUT);
  digitalWrite(PIN_AMP_SHDWM, 1);   // amp and nepixel ON
  analogWrite(PIN_ANA_OUT, 128);
  pinMode(PIN_NEOPIXEL , INPUT);

  // wait for serial port init
  int t = 20;
  while (!Serial && t--) delay(100);
  Serial.begin(115200);

  // get the stored saber color
  saberColIndex = colorFlashStore.read();

  // initialise standard neopixel library
  strip.begin();
  strip.clear();
  strip.show();

  // init the dotstart and turn off the led
#ifdef USE_DOTSTAR
  dotStar.begin();
  dotStar.setPixelColor(0, 0);
  dotStar.show();
#endif

  // send the boot message to serial port
  digitalWrite(PIN_LED, 1);
  Serial.print("\r\nSABER By Matt Denton\r\n\r\n");
  Serial.print("On Time = ");
  Serial.println(SABER_ON_TIME);
  Serial.print("Off Time = ");
  Serial.println(SABER_OFF_TIME);
  Serial.print("Hum Time = ");
  Serial.println(saberHumSize / (WAV_FREQUENCEY / 1000));

#ifdef USE_LIS3DH
  if (! lis.begin(0x18)) {
    Serial.println("Couldnt find LIS3DH");
    while (1) yield();
  }
  Serial.println("LIS3DH found!");
  lis.setRange(LIS3DH_RANGE_4_G);
  Serial.print("Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");
  
  //lis.setDataRate(LIS3DH_DATARATE_100_HZ);
  Serial.print("Data rate set to: ");
  switch (lis.getDataRate()) {
    case LIS3DH_DATARATE_1_HZ: Serial.println("1 Hz"); break;
    case LIS3DH_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3DH_DATARATE_25_HZ: Serial.println("25 Hz"); break;
    case LIS3DH_DATARATE_50_HZ: Serial.println("50 Hz"); break;
    case LIS3DH_DATARATE_100_HZ: Serial.println("100 Hz"); break;
    case LIS3DH_DATARATE_200_HZ: Serial.println("200 Hz"); break;
    case LIS3DH_DATARATE_400_HZ: Serial.println("400 Hz"); break;

    case LIS3DH_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
    case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.println("5 Khz Low Power"); break;
    case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.println("16 Khz Low Power"); break;
    default: Serial.println(lis.getDataRate()); break;
  }
#endif

#ifdef DEBUG_MODE
  // boot up delay (for development only)
  Serial.print("Boot Delay ");
  t = STRIP_LED_COUNT - 1;
  while (digitalRead(PIN_WAKE) && t--)
  {
    delay(BOOT_TIME_WAIT / STRIP_LED_COUNT);
    Serial.print(">");
    digitalWrite(PIN_LED, !digitalRead(PIN_LED) );
    strip.clear();
    strip.setPixelColor((STRIP_LED_COUNT * 2) - t - 1, 0x000080);
    strip.setPixelColor(t, 0x000080);
    strip.show();
  }
  Serial.println();
#endif

  // init WAV DAC output
  Serial.println("DAC Setup..");
  DACSetup(WAV_FREQUENCEY, WAV_OVERSAMPLE);                             // Set up DAC for 16kHz playback, 4x oversampling

  Serial.println("NEO Setup..");
  // clear neopixel strip
  strip.clear();
  strip.show();

  // generate SIN table
  Serial.println("Generate Sin Table..");
  float d = 0;
  for ( int t = 0; t < 360 * SIN_TABLE_SCALE; t++ )
  {
    sinTable[t] = sin(d);
    d += (1.0 / SIN_TABLE_SCALE) / 57.2957795;
  }

  // switch off all devices
  powerDownDevice();
}


// ************************************************************************
// the loop routine runs over and over again forever:
// ************************************************************************
void loop()
{
#ifdef USE_LIS3DH
  sensors_event_t event;
  lis.getEvent(&event);
  acceleration = event.acceleration.x * event.acceleration.x + event.acceleration.z * event.acceleration.z;
  // Then print out the raw data
//  Serial.print("X: "); Serial.print(event.acceleration.x);
//  Serial.print(", Y: "); Serial.print(event.acceleration.y);
//  Serial.print(", Z: "); Serial.print(event.acceleration.z);
//  Serial.print(", accel: "); Serial.println(acceleration);
#endif
  // system update at set frame rate of 50Hz
  if ( (micros() - lasttime) < (1000000 / FRAME_RATE) )
  {
    delay(100);
    return;
  }
  lasttime = micros();

  // animate based on current state of saber
  switch (saberState) {
    case saberIsOpened:
      // power up the saber
      powerUpSaber();
      break;
    case saberIsOn:
      // user keys to change settings ( debug mode / expreimental )
      checkKeys();
      // pulse the saber
      pulseSaber();
#ifdef USE_LIS3DH
      playMotionSounds();
#endif
      // update the neopixel strip
      strip.show();
      checkPowerOffSequence();
      break;
    case saberIsClosed:
      sleepSaber();
      break;
    case saberIsOff:
    default:
      break;
  }
}


void playMotionSounds() {
#ifndef MUTE_MODE
  if (acceleration > HIT_THRESHOLD) {
    pauseSample();
    playSample((const uint8_t*)saberSwing, saberSwingSize);
    Serial.println(acceleration);
    //loopSample((const uint8_t*)saberHum, saberHumSize);
  } else if (acceleration > SWING_THRESHOLD) {
    pauseSample();
    playSample((const uint8_t*)saberSwing, saberSwingSize);
    Serial.println(acceleration);
    //loopSample((const uint8_t*)saberHum, saberHumSize);
  }
#endif
}


void checkPowerOffSequence() {
  int state = digitalRead(PIN_WAKE);
  if (state != buttonPrevState) {
    if (state == HIGH) {
      buttonReleased = false;
      buttonPressedAt = millis();
    } else {
      buttonReleased = true;
    }
    buttonPrevState = state;
    if (buttonReleased == true && millis() - buttonPressedAt > 500) {
      powerDownSaber();
      buttonReleased = false;
      buttonPressedAt = 0;
    }
  }
}


// ************************************************************************
// put the saber to sleep
// put the device to sleep and wait for button press
// or timeout depending on which mode is selected
// ************************************************************************
void sleepSaber()
{

  // wait for button press mode
#ifdef USE_INTERRUPT_PIN
  Serial.println("Sleep..");
  delay(10); // message delay
#ifdef DEBUG_NO_SLEEP_MODE
  delay(150); // debounce delay
  if ( !digitalRead(PIN_WAKE) )
  {
    saberState = saberIsOpened;
  }
#else
  // wake from interrupt only seems to work if this is called before a sleep call!
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(PIN_WAKE), wakeUp, FALLING);
  LowPower.sleep();
  digitalWrite(PIN_LED, 1);
  delay(150); // debounce delay
  digitalWrite(PIN_LED, 0);
#endif
  // debounce signal and check for definate wake up?
  if ( digitalRead(PIN_WAKE) )
  {
    // no, go back to sleep.
    saberState = saberIsOff;
  }


  // sleep wake up mode without button press
#else
#ifdef DEBUG_NO_SLEEP_MODE
  delay(1000);
#else
  LowPower.sleep(1000);
#endif
  // reset input pullup, (not sure why this is needed but pullup seems to be reset without!)
  pinMode(PIN_WAKE , INPUT_PULLUP);
  // check the wake button
  if ( !digitalRead(PIN_WAKE) )
  {
    saberState = saberIsOpened;
  }
#endif
}


// ************************************************************************
// pulse the saber light
// ************************************************************************
void pulseSaber()
{
  unsigned long time = micros();

  float frequency = WAVE_FREQUENCY_MIN + random(WAVE_FREQUENCY_RND);
  float amplitude = (WAVE_AMPLITUDE_RND);
  float phaseshift = random(WAVE_PHASE_RND);

  degrees -= phaseshift; // phase shift
  //if( degrees >= 360 ) degrees -= 360;
  //else if( degrees < 0 ) degrees += 360;


  for ( int t = 0; t < STRIP_LED_COUNT; t++ )
  {
    // get the base color
    int r = (saberPixels[t] >> 16) & 0xff;
    int g = (saberPixels[t] >> 8) & 0xff;
    int b = saberPixels[t] & 0xff;

    // compute wave modificaion
    // using intrinsic funtion
    //float offset = 1.0 + (sin((degrees+(float(t)*frequency))/57.2957795) * amplitude); // amplitude
    // uisng lookup table
    float offset = 1.0 + (sinT(degrees + (float(t) * frequency)) * amplitude);

    // modify for new rgb color
    unsigned int rn = constrain((r * offset), 0, 255);
    unsigned int gn = constrain((g * offset), 0, 255);
    unsigned int bn = constrain((b * offset), 0, 255);

    // apply new color
    unsigned long rgb = (rn << 16) + (gn << 8) + bn;
    //saberPixelsModified[t] = rgb;

    strip.setPixelColor(t, rgb);
    strip.setPixelColor((STRIP_LED_COUNT * 2) - t - 1, rgb);

  }

  //Serial.println(micros()-time);
}



// ************************************************************************
// start the saber power up sequence
// ************************************************************************
void powerUpSaber()
{
  Serial.println("Power Up Start");
  // clear pixel array
  memset(saberPixels, 0, STRIP_LED_COUNT);
  // enable and clear strip
  digitalWrite(PIN_AMP_SHDWM, 1);
  delay(1);
  // reset neopixel pin
  strip.setPin(PIN_NEOPIXEL);
  strip.clear();
  unsigned long lasttime = millis() - (SABER_ON_TIME / STRIP_LED_COUNT);
#ifndef MUTE_MODE
  // play wake sound
  playSample((const uint8_t*)saberOn, saberOnSize);
#endif
  int t = 0;
  unsigned long start = millis();
  while ( t < STRIP_LED_COUNT )
  {
    if ( (millis() - lasttime) > (SABER_ON_TIME / STRIP_LED_COUNT) - 6 )
    {
      lasttime = millis();
      // set color
      saberPixels[t] = saberColor[saberColIndex];
      // apply phasing
      //strip.setPixelColor(t, saberPixels[t]);
      pulseSaber();
      // update strip
      strip.show();
      t++;
    }
  }
  saberState = saberIsOn;
  Serial.print("Power Up Time (mS): ");
  Serial.println(millis() - start);
  // wait sample end
  //Serial.println("WAIT SAMPLE END");
  //while( samplePlaying() ) continue;
  Serial.println("HUM Start");
#ifndef MUTE_MODE
  loopSample((const uint8_t*)saberHum, saberHumSize);
#endif

}



// ************************************************************************
// start the saber power down sequence and programming sequence
// ************************************************************************
void powerDownSaber()
{
  bool relaseButton = false;
  bool programCol = false;

  saberState = saberIsClosed;

  digitalWrite(PIN_LED, 1);
  Serial.println("SLEEP");

#ifndef MUTE_MODE
  playSample((const uint8_t*)saberOff, saberOffSize);
#endif

  unsigned long lasttime = millis() - (SABER_ON_TIME / STRIP_LED_COUNT);
  int t = 0;
  while ( t < STRIP_LED_COUNT )
  {
    if ( (millis() - lasttime) > (SABER_ON_TIME / STRIP_LED_COUNT) - 8 )
    {
      lasttime = millis();
      // switch off led
      saberPixels[STRIP_LED_COUNT - t - 1] = 0;
      // apply phasing
      pulseSaber();
      // update strip
      strip.show();
      t++;

      if ( relaseButton == false )
      {
        if ( digitalRead(PIN_WAKE) )
        {
          Serial.println("Button release");
          relaseButton = true;
        }
      }
      else
      {
        if ( !digitalRead(PIN_WAKE) && programCol == false)
        {
          programCol = true;
          Serial.println("Colour program start");
        }
      }
    }
  }

  int pressCount = 0;
  // check for program mode
  if (programCol)
  {
    programMode();
  }

  // wait for release
  while (!digitalRead(PIN_WAKE))
  {
    continue;
  }

  pauseSample();
  Serial.println("RELEASE");
  delay(50);

  powerDownDevice();
}



// ************************************************************************
// profram mode
// ************************************************************************
void programMode()
{
  // wait for switch release
  while ( !digitalRead(PIN_WAKE) );

  // wait for user colour set
  int tick = 0;
  while (1)
  {
    strip.fill(saberColor[saberColIndex], 0, (STRIP_LED_COUNT * 2) - 1);
    strip.show();
    delay(100);

    if ( !digitalRead(PIN_WAKE) )
    {
      Serial.println("Colour program end");
      strip.clear();
      strip.show();
      colorFlashStore.write(saberColIndex);
      break;
    }

    if ( tick++ > 13 )
    {
      tick = 0;
      saberColIndex++;
      if ( saberColIndex > MAX_COLORS - 1 ) saberColIndex = 0;
    }
  }
}

// ************************************************************************
// put device into low power state by disabling all devices
// ************************************************************************
void powerDownDevice()
{
  strip.clear();
  strip.show();
#ifdef USE_DOTSTAR
  dotStar.clear();
  dotStar.show();
#endif
  digitalWrite(PIN_AMP_SHDWM, 0);
  digitalWrite(PIN_LED, 0);
  // disable neopixel pin to stop leakage current through data pin
  strip.setPin(PIN_SPARE);
  saberState = saberIsClosed;
}

// ************************************************************************
// faster sin lookup table
// ************************************************************************
float sinT( float din )
{
  int tp;
  int sign = 1;
  float d = din;

  if ( d < 0 )
  {
    d = -d;
    sign = -1;
  }

  tp = fmod(d * SIN_TABLE_SCALE, 360 * SIN_TABLE_SCALE);

  if ( tp >= 0 && tp < 360 * SIN_TABLE_SCALE )
  {
    return sinTable[tp] * sign;
  }

  Serial.print("Table Error! ");
  Serial.print(din);
  Serial.print(" ");
  Serial.println(tp);
  return 0;
}


#ifdef USE_INTERRUPT_PIN
// ************************************************************************
// button press callback isr
// ************************************************************************
void wakeUp()
{
  if (saberState != saberIsOn) {
    saberState = saberIsOpened;
  }
}
#endif

// ************************************************************************
// check user keys for pulse change, useful while developing device, could
// be expanded for full user configuration via terminal port/app
// ************************************************************************
void checkKeys()
{
  while ( Serial.available() )
  {
    char c = Serial.read();
    switch ( c )
    {
      case 'q':
        waveAmplitude += 0.01;
        Serial.print("Wave Amplitude ");
        Serial.println(waveAmplitude);
        break;
      case 'a':
        waveAmplitude -= 0.01;
        Serial.print("Wave Amplitude ");
        Serial.println(waveAmplitude);
        break;
      case 'w':
        waveFrequency += 1;
        Serial.print("Wave Frequency ");
        Serial.println(waveFrequency);
        break;
      case 's':
        waveFrequency -= 1;
        Serial.print("Wave Frequency ");
        Serial.println(waveFrequency);
        break;
      case 'e':
        wavePhase += 1;
        Serial.print("Wave Phase ");
        Serial.println(wavePhase);
        break;
      case 'd':
        wavePhase -= 1;
        Serial.print("Wave Phase ");
        Serial.println(wavePhase);
        break;
    }
  }
}
