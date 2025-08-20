#include <avr/io.h>-
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <EEPROM.h>
#include <tinyNeoPixel_Static.h>
#define NUMLEDS 2
byte pixels[NUMLEDS * 3];
tinyNeoPixel strip = tinyNeoPixel(NUMLEDS, PIN_PA6, NEO_GRB, pixels);

typedef struct AnalogReadResult {
    int16_t minValue;
    int16_t maxValue;
    int16_t delta;
};

#define BUTTON_PIN PIN_PA7

uint16_t time_pin_low(uint16_t max_ms)
{
  // blocking for up to max_ms
  if (digitalRead(BUTTON_PIN) == HIGH)
  {
    return(0);
  }
  delay(50); //debounce-
  uint16_t t = 50;
  while(digitalRead(BUTTON_PIN) == LOW)
  {
    delay(5);
    t = t + 5;
    if ( t > max_ms )
      return(max_ms);
  }
  return(t);
}

void RTC_init()
{
  /* Initialize RTC: */
  while (RTC.STATUS > 0)
  {
    ;                                   /* Wait for all register to be synchronized */
  }
  RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;    /* 1kHz Internal Ultra-Low-Power Oscillator (OSCULP32K) */
}

void sleep()
{
  // Prepare for sleep and set interrupts
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  PORTA.PIN7CTRL = PORT_PULLUPEN_bm | PORT_ISC_LEVEL_gc; // enable pullup and interrupt
  // TODO, sleep timer interrupt...
  // Turn off pins
  digitalWrite(PIN_PA3, LOW); // turn off LED power rail
  pinMode(PIN_PA2, OUTPUT);   // stop sense pin floating
  // Disable other power draining things
  ADC0.CTRLA &= ~ADC_ENABLE_bm; // disable adc
  // Enter sleep
  sleep_enable();
  sleep_cpu();
  // sleep resumes here
  // Remove interrupts
  PORTA.PIN7CTRL = PORT_PULLUPEN_bm; // renable pullup but no interrupt
  // Setup pins
  pinMode(PIN_PA2, INPUT);   // turn sense pin to input
  // Renable ADC
  ADC0.CTRLA |= ADC_ENABLE_bm; // enable ADC
}

//RTC_PERIOD_enum period = RTC_PERIOD_CYC32768_gc;
RTC_PERIOD_enum period;
void sleep_with_rtc()
{
  RTC.PITINTCTRL = RTC_PI_bm;  // Enable RTC interrupt
  RTC.PITCTRLA = period | RTC_PITEN_bm; // Set timer to 2s
  sleep();
  RTC.PITINTCTRL = ~(RTC_PI_bm); // Disable RTC interrupt
}

ISR(PORTA_PORT_vect) {
  PORTA.INTFLAGS = PORT_INT7_bm; // Clear Pin 7 interrupt flag otherwise keep coming back here
}

ISR(RTC_PIT_vect)
{
  RTC.PITINTFLAGS = RTC_PI_bm;  // Clear RTC interrupt flag otherwise keep coming back here
}



void pwm500khz() { 
  TCA0.SPLIT.CTRLD = TCA_SPLIT_SPLITM_bm;  // Enable split mode
  TCA0.SPLIT.CTRLB = TCA_SPLIT_HCMP2EN_bm; // Enable comparator in split mode
  TCA0.SPLIT.HPER = 39; // Set the period
  PORTA.DIRSET = 1 << 5; //PA5
  TCA0.SPLIT.CTRLA =
    TCA_SPLIT_CLKSEL_DIV1_gc | // divide sys. clock by 1
    TCA_SPLIT_ENABLE_bm;
   TCA0.SPLIT.HCMP2 = 20; // W05 / PA5
}

void setup()
{
  // Setup pins
  pinMode(PIN_PA1, OUTPUT);       // PWM
  pinMode(PIN_PA2, INPUT);        // SENSE
  pinMode(PIN_PA3, OUTPUT);       // LED PWR
  pinMode(PIN_PA6, OUTPUT);       // LED DATA
  pinMode(PIN_PA7, INPUT_PULLUP); // BUTTON
  // Set analog sample furation
  analogSampleDuration(1);
  // Enable serial
  //Serial.begin(9600, (SERIAL_8N1 | SERIAL_TX_ONLY));
  uint8_t ctrla = (TCA0.SINGLE.CTRLA &~(TCA_SINGLE_CLKSEL_gm)); // mask off old value
  ctrla |= TCA_SINGLE_CLKSEL_DIV1_gc; //set bits of new value // 80Khz signal
  TCA0.SINGLE.CTRLA = ctrla;  
  delay(500);
  RTC_init();
}

AnalogReadResult sense()
{
    AnalogReadResult result;
    analogRead(PIN_PA2); // Take and throw away a reading
    analogWrite(PIN_PA1, 128); // Enable PWM with 50% duty cycle
    delayMicroseconds(200); // Wait 10ms
    result.maxValue = 0;
    result.minValue = 1024;
    int16_t value = 0;
    for (int i = 0; i < 200; i++) {
      value = analogRead(PIN_PA2);
      if (value > result.maxValue) { result.maxValue = value; } // Update maxValue
      if (value < result.minValue) { result.minValue = value; } // Update minValue
    }
    digitalWrite(PIN_PA1, LOW); // Disable PWM
    result.delta = result.maxValue - result.minValue;
    return result;
}


AnalogReadResult sense2() // This takes about 3ms to read
{
    analogWrite(PIN_PA1, 128); // Enable PWM with 50% duty cycle
    ADC0.MUXPOS=0x02; //reads from PA6/arduino pin 2, ADC0 channel 6
    ADC0.CTRLA=ADC_ENABLE_bm|ADC_FREERUN_bm; //start in freerun 
    ADC0.COMMAND=ADC_STCONV_bm; //start first conversion!
    AnalogReadResult result;
    delayMicroseconds(500); // Wait 0.5ms
    result.maxValue = 0;
    result.minValue = 1024;
    int value = 0;
    for (int i = 0; i < 1000; i++) {
      value = ADC0.RES;
      if (value > result.maxValue) { result.maxValue = value; } // Update maxValue
      if (value < result.minValue) { result.minValue = value; } // Update minValue
    }
    digitalWrite(PIN_PA1, LOW); // Disable PWM
    ADC0.CTRLA = ~ADC_ENABLE_bm; // Disable ADC
    result.delta = result.maxValue - result.minValue;
    return result;    
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                

uint16_t calibration = 0;
uint8_t calibrate = 0;

void calibrate_sense()
{
  // This function takes an average of the current calibration value and a new reading. 
  // To calibrate from scratch, set calibrate to 0 first.
  AnalogReadResult result = sense2();
  if (calibrate == 0)
  {
    calibration = result.delta - 30; // Nothing to average, so take current minus a little bit so we alert immediately on calibration
  }
  else
  {
    calibration = (calibration + result.delta) >> 1; // Get mean of current and this one
  }
  calibrate++;
}

#define LEFT 0
#define RIGHT 1

#define OFF 0,0,0
#define RED 10,0,0
#define GREEN 0,10,0
#define BLUE 0,0,10
#define ORANGE 10,4,0
#define PINK 6,0,4
#define PURPLE 6,0,8
#define OLIVE 2,2,0
#define YELLOW 6,6,0

void flash(int r, int g, int b, uint16_t ms = 3000, int count = 1)
{
  digitalWriteFast(PIN_PA3, HIGH);
  strip.setPixelColor(LEFT, OFF);
  strip.setPixelColor(RIGHT, OFF);
  strip.show();
  for (int j = 0; j < count; j++) 
  {
    strip.setPixelColor(LEFT, r, g, b);
    strip.setPixelColor(RIGHT, r, g, b);
    for (int i = 0; i < 10; i++) 
    {
      // We need this loop otherwise the on time to data time is too quick
      strip.show();
      delayMicroseconds(100);
    }
    delay(ms);
    strip.setPixelColor(LEFT, OFF);
    strip.setPixelColor(RIGHT, OFF);
    strip.show();
    if (count > 1)
    {
      delay(ms);
    }
  }
}

int police(int x)
{
  if( x % 2 == 0)
  {
    strip.setPixelColor(LEFT,255,0,0);
    strip.setPixelColor(RIGHT,0,0,255);
  }
  else
  {
    strip.setPixelColor(RIGHT,255,0,0);
    strip.setPixelColor(LEFT,0,0,255);
  }
  strip.show();
  return 150;
}

int evil(int x)
{
  int offset = x % 40;
  if (offset < 20)
  {
    strip.setPixelColor(LEFT,offset,0,0);
    strip.setPixelColor(RIGHT,offset,0,0);
  }
  else
  {
    strip.setPixelColor(LEFT,39 - offset,0,0);
    strip.setPixelColor(RIGHT, 39 - offset,0,0);
  }
  strip.show();
  if (offset > 37)
    return 250;
   return 20;
}



int rainbow(int x, int rtn)
{
  int offset = x % 6;
  switch(offset){
    case 0:
      strip.setPixelColor(0,RED);
      strip.setPixelColor(1,ORANGE);
      break;
    case 1:
      strip.setPixelColor(0,ORANGE);
      strip.setPixelColor(1,YELLOW);
      break;
    case 2:
      strip.setPixelColor(0,YELLOW);
      strip.setPixelColor(1,GREEN);
      break;
    case 3:
      strip.setPixelColor(0,GREEN);
      strip.setPixelColor(1,BLUE);
      break;    
    case 4:
      strip.setPixelColor(0,BLUE);
      strip.setPixelColor(1,PURPLE);
      break;
    case 5:
      strip.setPixelColor(0,PURPLE);
      strip.setPixelColor(1,RED);
      break;
  }
  strip.show();
  return rtn;
}

int random_colour(int rtn)
{
  int r = random(0,20);
  int g = random(0,20);
  int b = random(0,20);
  strip.setPixelColor(LEFT,r,g,b);
  strip.setPixelColor(RIGHT,r,g,b);
  strip.show();
  return rtn;
}

int fire(int rtn)
{
  int r = random(8,10);
  int g = random(2,5);
  strip.setPixelColor(LEFT,r,g,0);
  strip.setPixelColor(RIGHT,r,g,0);
  strip.show();
  return rtn;
}

int aurora(int n)
{
  int x = n % 20;
  int y = n % 10;
  if (x < 10)
  {
    strip.setPixelColor(LEFT,15 - y,0,5 + y);
    strip.setPixelColor(RIGHT,5 + y,0,15 - y);
  }
  else
  {
    strip.setPixelColor(RIGHT,15 - y,0,5 + y);
    strip.setPixelColor(LEFT,5 + y,0,15 - y);
  }
  strip.show();
  return 10;
}

int sos(int n)
{
  int x = n % 50;
  int y = x % 4;
  bool on = false;
  if (x < 12)
  {
    // dot dot dot
    if (y == 0)
    {
      on = true;
    }
  }
  else if (x < 24)
  {
    // dash dash dash
    if (y < 3)
    {
      on = true;
    }
  }
  else if (x < 36)
  {
    // dot dot dot
    if (y < 2)
    {
      on = true;
    }
  }
  // dot dot dot pause dash dash dash pause dot dot dot pause pause
  // on off off on off off on off off off off off
  if (on) {
  strip.setPixelColor(LEFT,255,0,0);
  strip.setPixelColor(RIGHT,255,0,0);
  }
  else {
  strip.setPixelColor(LEFT,0,0,0);
  strip.setPixelColor(RIGHT,0,0,0); 
  }
  strip.show();
  return 150;
}

int halloween(int x)
{
  if( x % 2 == 0)
  {
    strip.setPixelColor(LEFT,ORANGE);
    strip.setPixelColor(RIGHT,GREEN);
  }
  else
  {
    strip.setPixelColor(RIGHT,ORANGE);
    strip.setPixelColor(LEFT,GREEN);
  }
  strip.show();
  return 150;
}

int torch()
{
  strip.setPixelColor(LEFT,255,255,255);
  strip.setPixelColor(RIGHT,255,255,255);
  strip.show();
  return 250;
}

// BADGE MODES
#define MODE_BLINKY 1
#define MODE_SENSE 2
#define MODE_OFF 3
#define MODE_END 4

// BLINKY MODES
#define BLINKY_POLICE 1
#define BLINKY_EVIL 2
#define BLINKY_RAINBOW_FAST 3
#define BLINKY_RAINBOW_SLOW 4
#define BLINKY_RANDOM 5
#define BLINKY_FIRE 6
#define BLINKY_AURORA 7
#define BLINKY_HALLOWEEN 8
#define BLINKY_SOS 9
#define BLINKY_TORCH 10
#define BLINKY_END 11

#define CALIBRATION_COUNT 5

int mode = MODE_SENSE;
int blinky_mode = BLINKY_RAINBOW_FAST;
int interval = 0;
int i = 0;

void loop() {
  if (i > 200) { i =0; }
  period = RTC_PERIOD_CYC32768_gc; // Reset timer period to 32s
  // Check if button pressed and handle action
  uint16_t btn_depress_time = time_pin_low(1000);
  bool short_press = false;
  bool long_press = false;
  if (btn_depress_time > 300){
    if (btn_depress_time > 800) {
      mode++; // Move to next mode
      flash(BLUE,40000,3);
      calibrate = 0;
    } else {
      short_press = true; // Set short press flag
    }
  }
  switch(mode) {
    case MODE_BLINKY:
      if (short_press)
      {
        blinky_mode++;
        i = 0;
        interval = 0;
        return;
      }
      digitalWriteFast(PIN_PA3,HIGH);
      delay(1);
      if (interval != 0) 
      {
        // Blinky modes involve long sleeps, so we check / delay / decrease the counter and return to the main loop
        delay(40);
        interval--;
        return;
      }
      switch(blinky_mode)
      {
        case BLINKY_POLICE:
          interval = police(i);
          break;
        case BLINKY_EVIL:
          interval = evil(i);
          break;
        case BLINKY_RAINBOW_SLOW:
          interval = rainbow(i,200);
          break;
        case BLINKY_RAINBOW_FAST:
          interval = rainbow(i,20);
          break;
        case BLINKY_RANDOM:
          interval = random_colour(100);
          break;
        case BLINKY_FIRE:
          interval = fire(10);
          break;
        case BLINKY_AURORA:
          interval = aurora(i);
          break;
        case BLINKY_HALLOWEEN:
          interval = halloween(i);
          break;
        case BLINKY_SOS:
          interval = sos(i);
          break;
        case BLINKY_TORCH:
        interval = torch();
          break;
        case BLINKY_END:
          blinky_mode = 1;
          return;
      }
      i++;
      return;
    case MODE_SENSE:
        if (short_press || long_press) { calibrate = 0; } // On short press, or if new to this mode, start calibration cycle
        if (calibrate < CALIBRATION_COUNT) { // If we need to calibrate, do that
          calibrate_sense();
          period = RTC_PERIOD_CYC4096_gc; // Shorter sleep on the next sleep as we calibrate faster
          flash(GREEN);
        }
        else // Start measuring forever
        {
          AnalogReadResult result;
          result = sense2();
          if (result.delta > calibration ) // bigger delta means dryer
          {
          flash(RED);
          period = RTC_PERIOD_CYC4096_gc; // Shorter sleep on the next sleep as we need to alert user
          }
          else
          {
            flash(GREEN);
          }
        }
        sleep_with_rtc();
        return;
    case MODE_OFF:
      sleep();
      return;
    case MODE_END:
      mode = 1;
      return;
  }
}