/***********************************************************
08-14-2016 Mike North This is a little proof of concept 
to make a servo move in sync with audio.
*******************/
#include <Servo.h>
#include <math.h>

#define MOUTH_CLOSED 0
#define MOUTH_OPEN 60

#define NECK_LEFT 60
#define NECK_CENTER 90
#define NECK_RIGHT 120

#define PIN_MOUTH 2
#define PIN_NECK 4

#define ADC_COUNT 3 // number of ADC channels to read for position (starting at A0)

#define JAW_LIMIT 86 //should be 86 or 87 to match other value

#define SERVO_TIMEOUT_MINS 3 //sleep servos after 5 minutes of no sound or input

//Specifies the number of seconds automated control of features will be paused
//after receiving manual control for something.
//IE After sending a neck angle, it will wait this long before picking a new
//angle to look towards.
#define MANUAL_PAUSE_SECS 10

//================================================
// This should be the end of the tweakable values
//================================================

#define SLEEP_MS (SERVO_TIMEOUT_MINS * 1000 * 60) //if specified minutes go by with no signal, then put the servos to bed
#define PAUSE_MS (MANUAL_PAUSE_SECS * 1000)

boolean servosEnabled = false;

Servo servoMouth;
Servo servoNeck;

//Stores the current ADC channel being read
volatile uint8_t curADC = 0;

//Stores the value of the jaw as the channels are being read
volatile uint8_t nextJawValue = 0;

//REFS1:0 sets to the AVCC voltage ref
  
//ADLAR left aligns the ADC value into the high byte
//instead of only the upper 2 bits of the 10-bit
//ADC value being in the high byte.
//This lets us just read an 8bit value
const uint8_t admuxDefault = (0 << REFS1) | (1 << REFS0) | (1 << ADLAR);

//This gets set everytime we finish the round of analog inputs
//means there's a new valid value to read from jawValue
volatile bool cycleComplete = false;

volatile uint8_t jawValue = 0;

volatile bool runAnalogs = false;

//stores the current millis counter each loop iteration
//this lets the entire loop iteration run at the "same" time
//and we don't have to call millis multiple times
unsigned long curTime;

//last* stores the last time that feature was processed
unsigned long lastMouth;
unsigned long lastNeck;
unsigned long lastGaze;

//period* is time in ms between updates of that feature
unsigned long periodMouth = 20;
unsigned long periodNeck = 50;

//Gaze is the time between picking a new place to look
//this will also be randomly changed everytime gaze
//is updated
unsigned long periodGaze = 5000;

//For head turning we want to enable speed control and
//we want to slowly move towards a target over several
//update iterations.
//Floats are used for smoother processing across iterations
//but will be casted to ints when updating the servos.
float neckPos = NECK_CENTER;
float neckSpeed = 40.0; // in degrees per second
float neckTarget = NECK_CENTER;
float neckDT = periodNeck / 1000.0;

//Mouth is an instantaneous setting based on the
//audio level so we don't need to do fancy speed
//control or anything. Yet. We can add some
//other features like interpolation in the future.
float curMouth = MOUTH_CLOSED;

//Flag for whether automatic control of mouth (audio level)
//is enabled or not. Can disable auto via serial commands
//if wanting full manual control or to pause mouth action.
bool autoMouth = true;

//Flag to enable random gazing around
bool autoNeck = true;

//lastManual* tracks last manual command for feature so auto
//can be paused for the specified duration
unsigned long lastManualMouth = 0;
unsigned long lastManualNeck = 0;

//This is automagically called (when enabled) as soon as ADC finishes a conversion
ISR(ADC_vect) {
  //Read just the upper 8 bits (of the 10) of the ADC value
  uint8_t val = ADCH;

  //check threshold
  if (val < JAW_LIMIT) {
    ++nextJawValue;
  }

  //increment ADC # and check if we've finished
  if (++curADC == ADC_COUNT) {
    curADC = 0;
    jawValue = nextJawValue;
    nextJawValue = 0;
    cycleComplete = true;
  }

  //Set the input to the next pin to read
  ADMUX = admuxDefault | curADC;
}

void initAnalog() {
  Serial.print("Init Analog...");
  /* Going to do some specific setup to minimize the number of registers
   *  we have to touch during reads since we don't need much resolution
   *  we're just looking for a threshold (347 -> 86/87)
   */
  ADMUX = admuxDefault;

  //ADEN -> Enable ADC
  
  //ADIE -> Enable ADC Interrupt
  // We use the interrupt to constantly cycle through the inputs
  
  //ADPS# -> Prescaler values that drops 16MHz clock down to 125kHz - divides by 128
  //Conversions take 13 ADC clock cycles so this means about 9600 samples per sec
  // and with 3 channels this is roughly 3kHz that can be run which is WAY
  // faster than the 50Hz servos are typically updated at which means
  // we can add in better processing in the future if we want fancier response.
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

void startAnalog() {
  runAnalogs = true;
  
  //set the start conversion bit in the status and control register
  ADCSRA |= (1 << ADSC);
}

void stopAnalog() {
  //signal other analog functions that analog is not running
  runAnalogs = false;

  //disable ADC complete interrupt
  ADCSRA &= ~(1 << ADIE);
}

void attach_servos()
{   // attach the pin to the servo object
    servoMouth.attach(PIN_MOUTH); // pin 2 for mouth
    servoNeck.attach(PIN_NECK); // Pin 4 for Head swivel servo
    servosEnabled = true;
}

void detach_servos()
{   // detach the servo objects
    servoMouth.detach();
    servoNeck.detach();
    servosEnabled = false;
}

void updateMouth() {
  //check if we're outside a pause window and return to auto control
  if (autoMouth && curTime - lastManualMouth > PAUSE_MS) {
    if (cycleComplete) {
      //make a local copy of the value from the interrupt to prevent "issues"
      uint8_t jaw = jawValue;

      //clear the flag so we can wait for a new audio value.
      //we might actually miss a few but since the servo can only move
      //so fast it's not an issue. This flag will be more useful if
      //trying more processing on jaw movement outside of servo updates.
      cycleComplete = false;

      float jawRatio = jaw / (ADC_COUNT - 1);
      curMouth = ((MOUTH_OPEN - MOUTH_CLOSED) * jawRatio) + MOUTH_CLOSED;

      servoMouth.write(curMouth);
    }
  }
}

void updateNeck() {
  if (autoNeck && curTime - lastManualNeck > PAUSE_MS) {
    //anytime we're within 1 degree of where we want to be, just count as there
    float neckDiff = neckTarget - neckPos;
    
    if (fabs(neckDiff) > 1.0) {
      if (neckDiff < 0.0) { // to the left
        neckPos -= neckSpeed * neckDT;
        if (neckPos < neckTarget) neckPos = neckTarget;
      } else { //to the right
        neckPos += neckSpeed * neckDT;
        if (neckPos > neckTarget) neckPos = neckTarget;
      }
    } else {
      neckPos = neckTarget;
    }
  }

  servoNeck.write(neckPos);
}

void updateGaze() {
  if (autoNeck && curTime - lastManualNeck > PAUSE_MS) {
    //this is a bit different since we're expecting to change periodGaze
    //and we're not looking for precise timing here anyways since it's
    //pretty long delays as far as the avr is concerned
    lastGaze = curTime;

    //randomly pick next gaze delay
    periodGaze = random(15) * 500 + 3000; //3 - 10 seconds in 0.5 second intervals

    //pick movement speed 20-60 deg/sec
    neckSpeed = random(9) * 5 + 20;

    //pick next angle to look
    neckTarget = random(NECK_RIGHT - NECK_LEFT) + NECK_LEFT;
  }
}

void setup()
{
  //initialize serial first so we can use it for debugging output
  Serial.begin(115200);

  Serial.println("Starting...");
  initAnalog();
  attach_servos();

  startAnalog();

  curTime = millis();
  lastMouth = curTime;
  lastNeck = curTime;
} 

void loop()
{
  curTime = millis();

  //start checking update timing for features.

  //mouth is highest priority so we check it first for best results.
  if (curTime - lastMouth > periodMouth) {
    lastMouth += periodMouth;
    updateMouth();
  }

  //next see if it's time to update neck control
  if (curTime - lastNeck > periodNeck) {
    lastNeck += periodNeck;
    updateNeck();
  }

  if (curTime - lastGaze > periodGaze) {
    //In the other methods we updated timing before calling the method but this
    //one does it a bit differently given the length of the delays and not wanting
    //to repeat extended delays when coming off of manual command pause
    updateGaze();
  }

  //lastly we'll look at serial input and see if we're getting any manual commands to run
  if (Serial.available() > 0) {
    //get command character
    int c = Serial.read();

    switch (c) {
      case 'm': //disable auto mouth (audio control)
        autoMouth = false;
        break;

      case 'M': //enable auto mouth
        autoMouth = true;
        break;

      case 'n': //disable random neck (gazing)
        autoNeck = false;
        break;

      case 'N': //enable random neck
        autoNeck = true;
        break;

      case 'o': //mouth open halfway
        curMouth = (MOUTH_CLOSED + MOUTH_OPEN) / 2;
        lastManualMouth = curTime;
        break;

      case 'O': //mouth fully open
        curMouth = MOUTH_OPEN;
        lastManualMouth = curTime;
        break;

      case 'c': //close mouth
        curMouth = MOUTH_CLOSED;
        lastManualMouth = curTime;
        break;

      case 's': //set neck turn speed
        neckSpeed = Serial.parseInt() / 100.0;
        break;

      case ',': //turn neck left at current speed
        Serial.println("left");
        neckTarget = NECK_LEFT;
        lastManualNeck = curTime;
        break;

      case '.': //turn nect right at current speed
        Serial.println("right");
        neckTarget = NECK_RIGHT;
        lastManualNeck = curTime;
        break;

      case '/': //stop manual neck turning
        Serial.println("stop");
        neckTarget = neckPos;
        lastManualNeck = curTime;
        break;

      case 'f': //neck straight ahead
        neckTarget = NECK_CENTER;
        lastManualNeck = curTime;
        break;

      case '!': // set mouth to specified angle
        curMouth = Serial.parseInt();
        lastManualMouth = curTime;
        break;

      case '@': // set neck to specified angle
        neckTarget = Serial.parseInt();
        Serial.print("Neck: ");
        Serial.print(neckTarget);
        Serial.println("");
        
        neckPos = neckTarget;
        lastManualNeck = curTime;
        break;
    }
  }
} 
