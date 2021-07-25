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


// We want to be able to clap right after we get a trigger, so we have to position the cam right before the edge that makes a clap.
// We call the time between when we stop the cam to wait for a trigger and when the blade drops the "preload" time.
// Ideally we want this to be as close to zero as possible so there is as little a delay betweeen the trigger and the Resulting clap,
// but there is variation in the time it takes for the motor to make a rotation, and inaccuracies in measuring that time.
// So we need to aim for a time that is is far enough away from the actual clap that we have low risk of ovvershooting and clapping
// before we get a trigger. `CLAP_PRELOAD_TIME` sets how long that is. 
                                  

#define CLAP_PRELOAD_TIME 500     // Aim to get the cam this far away from the edge when preloading. MUST be bigger than START_HOLDOFF_TIME


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


static millis_t last_sample_time = 0; 
static byte curr_window_sample_count =0; 
static unsigned curr_window_total = 0;    

static unsigned prev1_window_total = 0;    // Previous window samples
static unsigned prev2_window_total = 0;    // Dopo previous window samples  


// Reset the clap check filter. It will be at least 2 windows until another clap can be detected after reset. 

void resetPolledClapCheck() {

  last_sample_time = 0; 
  curr_window_sample_count =0; 
  curr_window_total = 0;    
  
  prev1_window_total = 0;    // Previous window samples
  prev2_window_total = 0;    // Dopo previous window samples  
  
}


// Returns true if a clap is detected
// Call more than once per millisecond

uint8_t polledClapCheck() {

  uint8_t clap_flag=0;

  millis_t now = millis();   // Take an atomic snapshot 

  if (now > last_sample_time) {

    last_sample_time= now;     // Do it this way for testing so we can have delay()s in there. 
    
    const byte a=readADC(); 

    curr_window_total += a;      // Accumulate the readings for this window
    
    curr_window_sample_count++;
    
    if (curr_window_sample_count >= SAMPLE_COUNT_PER_WINDOW) {

      // End of current window

      // Did we go down since the dopo previous window?

      if ( curr_window_total < prev2_window_total)  {

        // Did we go down by more than the threshold?  

        // Note that the math below depends on already knowing that curr < prev2 from the above `if`
        
        if  ( (prev2_window_total - curr_window_total) > SAMPLE_THRESHOLD )  {

              // CLAP DETECTED

              clap_flag = 1; 

              // A hacky way to get us a 2 window-period hold-off before next detection event
              resetPolledClapCheck();
              
        }

      }

      // Bucket brigade the samples down
      prev2_window_total = prev1_window_total;
      prev1_window_total = curr_window_total;
      curr_window_total = 0;
      curr_window_sample_count=0;
            
    }
        
  }

  return clap_flag;
  
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


millis_t prevClapCycleDuration;       // How long did it actually take for us to clap after the last preload pause? Target is CLAP_PRELOAD_TIME

void setup()
{

  // Set up ADC
  initADC();

  initMotor(); 

  // For debug info
  Serial.begin( 1000000  );  

  delay(100);   // Let serial port warm up

  Serial.println( F("\n\rCWT-CLAPPER, https://github.com/bigjosh/CWT-clapper\n\r"));

  // On power up we need a baseline for the length of a clap before to drop into our loop
  // so we run a full cycle to time one

  // Turn on motor and wait a backoff period to ignore the case where we instantly detect a clap
  motorOn();
  delay(CLAP_HOLDOFF_TIME);

  Serial.print( F("1."));
  
  // Now wait for an actual detected clap
  while (!polledClapCheck());  

  Serial.print( F("2."));
  
  const millis_t lastClapTime = millis();
  delay(CLAP_HOLDOFF_TIME);

  // Now wait for a second actual detected clap (full clap cycle)
  while (!polledClapCheck());  

  // Collect data to get ready for real cycle
  prevClapCycleDuration  = millis() - lastClapTime;

  // ...now OK to drop into loop() 
    
}

// By convention, loop() is entered immedeiately after a clap has been detected, motor on.
// Expects `prevClapCycleDuration` to be set to the time the motor was on in the previous clap cycle. 

void loop() {
  
  const millis_t clapCycleStartTime = millis();   
  
  Serial.print( F("\r\nStart,d="));   
  Serial.print( prevClapCycleDuration  );  
  
  Serial.print(".Holdoff");
  // Delay to avoid detecting the same clap twice
  delay(CLAP_HOLDOFF_TIME);         

  // Now we want to position our cam to right before it is about to clap 

  const millis_t clap_to_preload_runtime = (prevClapCycleDuration - CLAP_PRELOAD_TIME) - CLAP_HOLDOFF_TIME;

  // Now run until we get to the preload stoping point - while checking for a premature clap...
  Serial.print( F(".preload=") );
  Serial.print( clap_to_preload_runtime );

  if (polledClapCheckDelay(clap_to_preload_runtime)) {

    // If we get here, then we clapped before we got to the preload stop point so we overshot.
  
    // Update our cycle durration, which will now be shorter so hopefully we will not overshoot again
    prevClapCycleDuration = millis() - clapCycleStartTime;
    
    Serial.print( F(".overshoot!") );   

    // ... and start a new cycle
    return;
    
  }

  // OK, we are at the expected preload stop point now, so stop
  
  motorOff();

  // Wait
  Serial.print( F(".wait.") );  
  delay(5000);

  // Time to clap!
  motorOn();
  resetPolledClapCheck();   // We are starting the motor from stop, so reset our filter to we don't detect a false positive. 
  const millis_t preload_run_start_time = millis();   
  Serial.print( F(".triggered") );

  Serial.print(".Holdoff");
  // Delay to avoid detecting the same clap twice
  delay(START_HOLDOFF_TIME);

  Serial.print(".untilclap");
   
  // Wait for the clap to happen (expected about CLAP_PRELOAD_TIME ms after we started, but not nessisarily)
  while (!polledClapCheck()); 

  // OK, we just clapped so remember how long the full cycle took
  // Total cycle = time we ran to get to the preload stop point + time we ran from the stop point to the clap

  const millis_t preload_to_clap_runtime = (millis() - preload_run_start_time);
  
  Serial.print( "postload=" );
  Serial.println( preload_to_clap_runtime  );     // We want this to be close to CLAP_PRELOAD_TIME
  
  // We do this calculation of the total time the motor had to run to complete a full cycle
  // Does not include the time we were waiting for a trigger since motor was off.
  prevClapCycleDuration = clap_to_preload_runtime + preload_to_clap_runtime ;

  // OK, drop back into loop() and do it all over again
}  
