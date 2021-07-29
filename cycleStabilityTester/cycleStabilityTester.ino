/*
 * Clapper firmware - 2021 josh.com
 * 
 * Theory of operation 
 * ===================
 * 1. As the cam rotates and pushes the blade, the spring is progessively streched. 
 * 2. The farther the spring is streched, the harder it gets to strech it further (googgle "Hooke's law"). 
 * 3. The more force it takes to turn the motor, the more current it uses. 
 * 4. The more current the motor uses, the lower the voltage across the batteries becomes (google "internal resistance"). 
 * 5. We can indirectly measure the voltage across the battery with the internal ADC (https://wp.josh.com/2014/11/06/battery-fuel-guage-with-zero-parts-and-zero-pins-on-avr/)
 * 
 * Q.E.D.: We can (sort of, barely, sextuple indirectly) measure the position of the blade by measuring the internal voltage 
 * bandgap of the chip using the ADC!
 * 
 * In practice, the change in ADC readings between the blade being fully open and fully shut is aorund one bit (i.e. the 
 * difference between a reading of 84 and 85) and there is lots of noise also in these readings. 
 * 
 * So we aim to detect the most abupt change in that signal - the moment when the blade claps and the motor goes from pushing 
 * a fully streched to spring to not pushing on the blade at all (there is a but of a dead zone after a clap before the cam
 * touches the blade again).
 * 
 * To find this edge, we first filter out high freqnecy noise by taking an ADC sample every millisecond and summing these over 
 * 100 millseconds. We call this 100ms a "window". At the end of each window, we compare the sum for this window with the one
 * that was two windows ago. If the most recent window's sum was more than SAMPLE_THRESHOLD less than the window before the
 * window before (the dopo-previous) one, then we detect a clap. 
 * 
 * Why do we skip a window wwhen comparing? Becuase the change is small enough that we need to compare before the change started
 * to after the change completed. If we just compared successive windows, we might be sampling durring the change and so we would 
 * see some part of the change in one window and the rest in the other window and this would be a much smaller (and likely 
 * undetectable ammount of) change. We pick our window size so that the change happens quickly relative to ammount of time 
 * a single window takes. 
 * 
 * Why then do we not skip two windows? This would further increase our SNR, but at the cost of having to wait for another window
 * before being able to detect the change (a total an avverage of 250ms). In practice, skipping one window seems to be good enough
 * to always find the change and withouth false triggers, and saves us 100ms of detection latency. 
 * 
 */


// TO COMPLILE AND DOWNLOAD:
//  In Prefs, add to Additional Board URLs: http://drazzy.com/package_drazzy.com_index.json
//  In Board Manager, add: ATTinyCore boards
//  In Tools->Boards menu, select...
//   Board        ->attiny 441/481 (no bootloader)
//   Chip         ->attiny441
//   Clock Source ->8mhz internal <4.5V.
//   Millis       ->Enabled
//   (other stuff doesn't matter)

// After you do this, you should do "Burn Bootloader" one time on each chip to set the fuses to the above settings.

#include <avr/pgmspace.h>     // Almost no RAM on this chip, so we keep all strings in flash with `F()`
#include <limits.h>             // Get UNINT_MAX

#define MOTOR 8     // Motor MOSFET pin

void motorOn(){
  digitalWrite(MOTOR, HIGH);  
}

void motorOff(){
  digitalWrite(MOTOR, LOW);  
}

void initMotor() {  
  pinMode( MOTOR , OUTPUT );
}


// We take an ADC reading every millisecond, basically becuase the finest handy timer Arduino gives us is millis()
// Then we average each 100 readings together to filter noise
// A clap seems to take about 200ms, so we are sampling at 2x 
// Then we compare current sample to the sample 2 windows ago to find the definitive clap
// Skipping the sample in the middle should filter out the transision while the clap is actually happening
// This is all very hard becuase there is less than 1 step of signal in the ADC reading!

#define SAMPLE_COUNT_PER_WINDOW 100      // Max 256 without expanding size of window_total variables
#define SAMPLE_THRESHOLD 50              // How much this window has to drop compared to 2nd previous window to detect a clap. Emperically determined

// Note that all times are in milliseconds

typedef unsigned long millis_t;


// Here is the sequnce of events in a clap cycle, starting at a clap

// 1. CLAP_HOLDOFF_TIME  (specified)
// 2. clap_to_preload_runtime (calulated)
// 3. trigger wait time (externally driven)
// 4. preload_to_clap_runtime (measured, target is CLAP_PRELOAD_TIME)

// Becuase the cam is a phsycial object, and also becuase we are sampling it at close to its change rate, we might sometimes see the same
// clap even show up in multipule windows. Avoiding this is easy, we just wait for a holdoff period after detecting a clap before we can
// detect another. Since the clap period is very long compared to the time period when we can potentially see the same clap twice, this is easy. 
// No need to try pick the smallest possible value here. 

#define CLAP_HOLDOFF_TIME 200    // Minimum time between consecutive claps. This prevents us from seeing the same clap twice.                                   
                                 // Should be much less than a clap cycle time, which emperically seems to be 3-4 seconds. 

#define START_HOLDOFF_TIME 200   // Minimum time to wait before lookingg for a clap after starting the motor. 

#define CLAP_MIDPOINT_RUNTIME  2000     // A nice place to stop and rest the motor when we are doing our calibration cycle
                                        // Should be sort of in the middle, not too close to the star or end


#define MOTOR_REST_TIME 1000      // Time we let the motor rest right after a clap. 
                                  // Also the time we rest in the middle of our calibration cycle to simulate the motor stopping and waiting for the trigger. 


// We want to be able to clap right after we get a trigger, so we have to position the cam right before the edge that makes a clap.
// We call the time between when we stop the cam to wait for a trigger and when the blade drops the "preload" time.
// Ideally we want this to be as close to zero as possible so there is as little a delay betweeen the trigger and the Resulting clap,
// but there is variation in the time it takes for the motor to make a rotation, and inaccuracies in measuring that time.
// So we need to aim for a time that is is far enough away from the actual clap that we have low risk of ovvershooting and clapping
// before we get a trigger. `CLAP_PRELOAD_TIME` sets how long that is. 
                                  

#define CLAP_PRELOAD_TIME 800     // Aim to get the cam this far away from the edge when preloading. MUST be bigger than START_HOLDOFF_TIME


byte readADC() {

    ADCSRA |= _BV( ADSC );    // Start the conversion    

    while ( ADCSRA &  _BV( ADSC ) );      // Wait for bit to clear (indicates conversion done)

    const uint8_t a = (uint8_t) ADC;                     // Read conversion result. 

    // Note that we toss the high bits of the sample. Our battery voltage range happens to put our readings right in the middle of the ADC low byte range.

    return a;
  
}

void initADC() {

  ADMUXA = 0b001101;      // Read the internal 1.1 reference. Datasheet 16.13.1
  ADMUXB = 0x00;          // Against the Vcc

  ADCSRA = 0b10000111;    // Enable ADC, No autotrigger or interrupt, divide by 32 (gives 250Khz @ 8Mhz clock). We are suppoed to be less than 200Khz, but should be ok. 
    
}


static millis_t next_sample_time = 0; 

const byte WINDOW_SIZE=100;
const byte WINDOW_TIME=4;

const unsigned CLAP_DETECT_THRESHOLD=75;

byte window1[WINDOW_SIZE];
byte window2[WINDOW_SIZE];

unsigned window1_total=0;
unsigned window2_total=0;

// Reset the clap check filter. It will be at least 2 windows until another clap can be detected after reset. 

void resetPolledClapCheck() {

  next_sample_time = 0; // Not possible to detect a clap before this time
  
  window1_total = 0;    
  window2_total = 0;           

  memset( window1 , 0x00 , WINDOW_SIZE ); 
  memset( window2 , 0x00 , WINDOW_SIZE ); 
  
}

// Returns true if a clap is detected
// Call more than once per millisecond

uint8_t polledClapCheck() {

  millis_t now = millis();   // Take an atomic snapshot 

  if (now >= next_sample_time) {

    next_sample_time= now + WINDOW_TIME;     // Do it this way for testing so we can have delay()s in there. 
    
    const byte a=readADC(); 

    const byte window1_pull = window1[WINDOW_SIZE-1];
    window1_total -= window1_pull;
    
    const byte window2_pull = window2[WINDOW_SIZE-1];
    window2_total -= window2_pull;

    for( byte i = (WINDOW_SIZE-1) ; i>0 ; i-- ) {
      window1[i]=window1[i-1];
      window2[i]=window2[i-1];      
    }

    window1[0] = a;
    window1_total += a;
    
    window2[0] = window1_pull;
    window2_total += window1_pull;

/*    

    Serial.print( now );
    Serial.print(" ");
    Serial.print( a );    
    Serial.print(" ");
    Serial.print( window1_total );
    Serial.print(" ");
    Serial.print( window2_total );

    if (window2_total > window1_total) {
      Serial.print(" ");
      Serial.print( window2_total - window1_total );      
    } else {
      
      Serial.print(" ");
      Serial.print( 0 );      
    }

    Serial.println();    
    
*/

    if ( window2_total > window1_total ) {

      const unsigned drop = window2_total - window1_total; 
      
      if (drop >= CLAP_DETECT_THRESHOLD) {

        return true;
        
      }
    }
            
  }

  return false;


}


// Delay `delaytime` milliseconds, but return early if a clap is detected
// Returns ture if clap. 

uint8_t polledClapCheckDelay( uint16_t delaytime ) {

  millis_t endtime = millis() + delaytime;

  while ( millis() < endtime ) {
    if (polledClapCheck() ) {
      return 1;
    }
  }

  return 0;
  
}


void setup()
{

  // Set up ADC
  initADC();

  initMotor(); 

  // For debug info
  //Serial.begin( 1000000  );  

  delay(100);   // Let serial port warm up

  //Serial.println( F("\n\rCWT-CLAPPER, https://github.com/bigjosh/CWT-clapper\n\r"));

  // On power up we need a baseline for the length of a clap before to drop into our loop
  // so we run a full cycle to time one

  
  motorOn();
  delay( MOTOR_REST_TIME );

  resetPolledClapCheck();


  while (!polledClapCheck());  
  motorOff();

  //Serial.println("Start");  
}

millis_t prevClapCycleDuration=0;  // Total motor on time of last clap cycle,
                                    // Starting when we started our motor after previous clap,
                                    // ending when the most recent clap was detected,
                                    // excluding time waiting for trigger

// By convention, loop() is entered immedeiately after a clap has been detected, motor on.

void loop() {

  delay( MOTOR_REST_TIME );

  motorOn();

  millis_t last=millis();

  
  while (1) {

    resetPolledClapCheck();
    while (!polledClapCheck());     
    const millis_t now=millis();
    motorOff();
    delay(1000);
    motorOn();
    
    //resetPolledClapCheck();
    
    //Serial.println( now - last );    
    last = now; 
    
  };
}
  
