/*
Copyright (c) 2019 Shajeeb TM

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// https://create.arduino.cc/projecthub/Shajeeb/32-band-audio-spectrum-visualizer-analyzer-902f51
#include <avr/wdt.h>
#include <arduinoFFT.h>
#include <SPI.h>
#include <FastLED.h>
FASTLED_USING_NAMESPACE

//#include <DTMF.h>
//int sensorPin = A4;
//float n=128.0;
//float sampling_rate=8926.0;
//DTMF dtmf = DTMF(n,sampling_rate);
//float d_mags[8];
//String inString = "";    // string to hold input

#define SAMPLES 32            //Must be a power of 2
//#define HARDWARE_TYPE MD_MAX72XX::FC16_HW   // Set display type  so that  MD_MAX72xx library treets it properly
//#define MAX_DEVICES  4   // Total number display modules
//#define CLK_PIN   13  // Clock pin to communicate with display
//#define DATA_PIN  11  // Data pin to communicate with display
//#define CS_PIN    10  // Control pin to communicate with display
#define  xres 8      // Total number of  columns in the display, must be <= SAMPLES/2
#define  yres 15       // Total number of  rows in the display


//int MY_ARRAY[]={0, 128, 192, 224, 240, 248, 252, 254, 255}; // default = standard pattern
//int MY_MODE_1[]={0, 128, 192, 224, 240, 248, 252, 254, 255}; // standard pattern
//int MY_MODE_2[]={0, 128, 64, 32, 16, 8, 4, 2, 1}; // only peak pattern
//int MY_MODE_3[]={0, 128, 192, 160, 144, 136, 132, 130, 129}; // only peak +  bottom point
//int MY_MODE_4[]={0, 128, 192, 160, 208, 232, 244, 250, 253}; // one gap in the top , 3rd light onwards
//int MY_MODE_5[]={0, 1, 3, 7, 15, 31, 63, 127, 255}; // standard pattern, mirrored vertically

 
double vReal[SAMPLES];
double vImag[SAMPLES];
char data_avgs[xres];

int yvalue;
int displaycolumn , displayvalue;
int peaks[xres];
const int buttonPin = 2;    // the number of the pushbutton pin
int state = HIGH;             // the current reading from the input pin
int previousState = LOW;   // the previous reading from the input pin
int displaymode = 1;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers


//MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);   // display object
arduinoFFT FFT = arduinoFFT();                                    // FFT object
 
//#define LED_PIN     5
//#define NUM_LEDS    14
//#define BRIGHTNESS  64
#define LED_TYPE    WS2811
//#define LED_TYPE    NEOPIXEL
//#define COLOR_ORDER GRB
#define NUM_STRIPS xres
#define NUM_LEDS_PER_STRIP yres
CRGB leds[NUM_STRIPS][NUM_LEDS_PER_STRIP];

#define NUM_LEDS NUM_LEDS_PER_STRIP

void setup() {
//    int dummy;
//    dummy = analogRead(A5);
//    digitalWrite(A5,0);
    ADCSRA = 0b11100101;      // set ADC to free running mode and set pre-scalar to 32 (0xe5)
    ADMUX = 0b00000000;       // use pin 32u4 ADC0(arduino A5) and external voltage reference
    pinMode(buttonPin, INPUT);
//    mx.begin();           // initialize display
  FastLED.addLeds<LED_TYPE, 4>(leds[0], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<LED_TYPE, 5>(leds[1], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<LED_TYPE, 6>(leds[2], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<LED_TYPE, 7>(leds[3], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<LED_TYPE, 8>(leds[4], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<LED_TYPE, 9>(leds[5], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<LED_TYPE, 10>(leds[6], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<LED_TYPE, 11>(leds[7], NUM_LEDS_PER_STRIP);
  FastLED.setBrightness(8);
  
  Serial.begin(115200);

    delay(50);            // wait to get reference voltage stabilized
    
  wdt_enable(WDTO_250MS);
}


void printDouble( double val, unsigned int precision){
// prints val with number of decimal places determine by precision
// NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
// example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

   Serial.print (int(val));  //prints the int part
   Serial.print("."); // print the decimal point
   unsigned int frac;
   if(val >= 0)
     frac = (val - int(val)) * precision;
   else
      frac = (int(val)- val ) * precision;
   int frac1 = frac;
   while( frac1 /= 10 )
       precision /= 10;
   precision /= 10;
   while(  precision /= 10)
       Serial.print("0");

   Serial.print(frac,DEC) ;
   Serial.print("\t");
}


// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm };
//SimplePatternList gPatterns = { rainbow, confetti, sinelon, juggle, bpm };

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns

boolean isSoundIn = false;
long isSoundInCounter = 1000L;

void loop() {
  wdt_reset ();
   // ++ Sampling
   for(int i=0; i<SAMPLES; i++)
    {
      while(!(ADCSRA & 0x10));        // wait for ADC to complete current conversion ie ADIF bit set
      ADCSRA = 0b11110101 ;               // clear ADIF bit so that ADC can do next operation (0xf5)
      int value = ADC - 512 ;                 // Read from ADC and subtract DC offset caused value
      vReal[i]= value/8;                      // Copy to bins after compressing
      vImag[i] = 0;                         
    }
    // -- Sampling
//    printDouble(ADC,10);
    
    // ++ FFT
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    // -- FFT

    
    // ++ re-arrange FFT result to match with no. of columns on display ( xres )
    int step = (SAMPLES/2)/xres; 
    step=1;
    int c=0;
    for(int i=0; i<(SAMPLES/2); i+=step)  
    {
      data_avgs[c] = 0;
      for (int k=0 ; k< step ; k++) {
          data_avgs[c] = data_avgs[c] + vReal[i+k];
      }
      data_avgs[c] = data_avgs[c]/step; 
      c++;
    }
    // -- re-arrange FFT result to match with no. of columns on display ( xres )

    if(isSoundInCounter > 0) isSoundInCounter--;
    else {
      isSoundIn = false;
      FastLED.setBrightness(8);
    }
    // ++ send to display according measured value 
    for(int i=0; i<xres; i++)
    {
//      printDouble(log(data_avgs[i]),10);
      data_avgs[i] = data_avgs[i] * (i+1);  // suppress low freq
      data_avgs[i] = (10)*log(data_avgs[i]+1); // logrithm
      data_avgs[i] = constrain(data_avgs[i],0,50);            // set max & min values for buckets
      data_avgs[i] = map(data_avgs[i], 0, 50, -30, yres);        // remap averaged values to yres, minus for baseline
      yvalue=data_avgs[i];
      peaks[i] = peaks[i]-1;    // decay by one light
      if (yvalue > peaks[i])
          peaks[i] = yvalue;
      if (data_avgs[i] > 0) {
          isSoundIn = true;
          isSoundInCounter = 1000L;
          FastLED.setBrightness(128);
      }
      yvalue = peaks[i];
      //displayvalue=MY_ARRAY[yvalue];
      displaycolumn=i;
      for(int j=yvalue;j>=0;j--)
        leds[displaycolumn][j].setHue((j) * 255/yres);
      
      //delay(100);
//      mx.setColumn(displaycolumn, displayvalue);              // for left to right
     }
//     Serial.println();

  if (!isSoundIn){   
    // Call the current pattern function once, updating the 'leds' array
    gPatterns[gCurrentPatternNumber]();
  }   
     
     FastLED.show();
    
  for(int i=0;i<xres;i++)
    for(int j=0;j<yres;j++)
      leds[i][j] = CRGB::Black;
    
    delay(8);  
     // -- send to display according measured value 
     
    displayModeChange ();         // check if button pressed to change display mode

  if (!isSoundIn){   
    // do some periodic updates
    EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
    EVERY_N_SECONDS( 10 ) { nextPattern(); } // change patterns periodically
  }

//// DTMF decoder
//  char thischar;
//  /* while(1) */dtmf.sample(sensorPin);
//  dtmf.detect(d_mags,512);
//  for(int i = 0;i < 8;i++) {
//    d_mags[i] /= 100;
//  }
//  thischar = dtmf.button(d_mags,30.);
//  if(thischar) {
//    fill_rainbow( leds[int(thischar)], NUM_LEDS, gHue, 7);
//    //Serial.print(thischar);
//    if(thischar == '#'){
//      
//    }
//    else if(thischar == '*'){
//      
//      // inString = "";
//    }    
//    else{
//      // inString += thischar;
//    }
//  } // if thischar     
} 



#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}

void rainbow() 
{
  // FastLED's built-in rainbow generator
  for(int i=0; i<NUM_STRIPS; i++)
    fill_rainbow( leds[i], NUM_LEDS, gHue, 7);
}

void rainbowWithGlitter() 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( fract8 chanceOfGlitter) 
{
  for(int i=0; i<NUM_STRIPS; i++)
    if( random8() < chanceOfGlitter) {
      leds[i][ random16(NUM_LEDS) ] += CRGB::White;
    }
}

void confetti() 
{
  // random colored speckles that blink in and fade smoothly
  for(int i=0; i<NUM_STRIPS; i++)
    fadeToBlackBy( leds[i], NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  for(int i=0; i<NUM_STRIPS; i++)
    leds[i][pos] += CHSV( gHue + random8(64), 200, 255);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  for(int i=0; i<NUM_STRIPS; i++)
    fadeToBlackBy( leds[i], NUM_LEDS, 20);
  int pos = beatsin16( 13, 0, NUM_LEDS-1 );
  for(int i=0; i<NUM_STRIPS; i++)
    leds[i][pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for(int j; j<NUM_STRIPS; j++)
    for( int i = 0; i < NUM_LEDS; i++) { //9948
      leds[j][i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
    }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  for(int i=0; i<NUM_STRIPS; i++)
    fadeToBlackBy( leds[i], NUM_LEDS, 20);
  byte dothue = 0;
  for(int j; j<NUM_STRIPS; j++)
    for( int i = 0; i < 8; i++) {
      leds[j][beatsin16( i+7, 0, NUM_LEDS-1 )] |= CHSV(dothue, 200, 255);
      dothue += 32;
    }
}




void displayModeChange() {
  int reading = digitalRead(buttonPin);
  if (reading == HIGH && previousState == LOW && millis() - lastDebounceTime > debounceDelay) // works only when pressed
  
  {

   switch (displaymode) {
    case 1:    //       move from mode 1 to 2
      displaymode = 2;
//      for (int i=0 ; i<=8 ; i++ ) {
//        MY_ARRAY[i]=MY_MODE_2[i];
//      }
      break;
    case 2:    //       move from mode 2 to 3
      displaymode = 3;
//      for (int i=0 ; i<=8 ; i++ ) {
//        MY_ARRAY[i]=MY_MODE_3[i];
//      }
      break;
    case 3:    //     move from mode 3 to 4
      displaymode = 4;
//      for (int i=0 ; i<=8 ; i++ ) {
//        MY_ARRAY[i]=MY_MODE_4[i];
//      }
      break;
    case 4:    //     move from mode 4 to 5
      displaymode = 5;
//      for (int i=0 ; i<=8 ; i++ ) {
//        MY_ARRAY[i]=MY_MODE_5[i];
//      }
      break;
    case 5:    //      move from mode 5 to 1
      displaymode = 1;      
//      for (int i=0 ; i<=8 ; i++ ) {
//        MY_ARRAY[i]=MY_MODE_1[i];
//      }
      break;
  }

    lastDebounceTime = millis();
  }
  previousState = reading;
}
