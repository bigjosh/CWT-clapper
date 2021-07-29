/*
 * CWT-CLAPPER firmware, 2021 josh.com, https://github.com/bigjosh/CWT-clapper
 * 
 * Theory of operation 
 * ===================
 * 1. As the cam rotates and pushes the blade, the spring is progessively streched. 
 * 2. The farther the spring is streched, the harder it is to strech it further (googgle "Hooke's law"). 
 * 3. The more force it takes to turn the motor, the more current it uses. 
 * 4. The more current the motor uses, the lower the voltage across the batteries becomes (google "internal resistance"). 
 * 5. We can indirectly measure the voltage across the battery with the internal ADC (https://wp.josh.com/2014/11/06/battery-fuel-guage-with-zero-parts-and-zero-pins-on-avr/)
 * 
 * Q.E.D.: We can (sort of, barely, sextuple indirectly) measure the position of the blade by measuring the internal voltage 
 * bandgap of the chip using the ADC!
 * 
 * In practice, the change in ADC readings between the blade being fully open and fully shut is very small 
 * and there is lots of noise also in these readings.
 * 
 * So we aim to detect the most abupt change in that signal - the moment when the blade claps and the motor goes from pushing 
 * a fully streched to spring to not pushing on the blade at all (there is a but of a dead zone after a clap before the cam
 * touches the blade again).
 * 
 * To find this edge, we both filter out high freqnecy noise and amplify the longer running signal by taking periodic ADC samples 
 * and summing these over a longer window. We have two windows consecutive windows, so when we get to the moment where one window is
 * filled with samples from before the flap and the other is filled with samples from after the clap, there should be enough SNR to detect
 * that by comparing the sums of all the values in each window. 
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

// After you do this, you should do "Burn Bootloader" at least one time on each chip to set the fuses to the above settings.

#include <avr/pgmspace.h>     // Almost no RAM on this chip, so we keep all strings in flash with `F()`
#include <limits.h>             // Get UNINT_MAX

// Note that all times are in milliseconds

typedef unsigned long millis_t;

// Number of samples in a window (the are two windows - before- and after-clap)
// More samples mean better filtering and more amplifcation of the small changes, but bigger also means it takes more
// samples (and therefore more time) to detect a change. More samples also needs more memory, which we have very little of. 
const byte WINDOW_SIZE=100;     

// Time between samples. Shorter would be great, but then we would need more memory. The optimal value captures the full drop 
// within one window. Emperically this seems to be about 400ms.
const byte WINDOW_TIME=4;       // Time between samples

// This sets how much the 1st window accumulator must be lower than the 2nd to detect a clap.
// This depends on how much the value changes between pre- and post-clap (very little) times the number of samples in each window.
// Lower values can detect a clap more quickly, but can also lead to false positives. 

const unsigned CLAP_DETECT_THRESHOLD=75;

// Aim to be this far away from the the edge of the cam when we stop to wait for a trigger.
// Too long means higher latency between when we trigger and when we clap
// Too short means a greater chance of overshooting and clapping before a trigger and having to go all the way around again. 

// Remember that this includes the time it takes to detect a clap which is about 2 windows, so the minimum time here that will avoid
// premature clapping before preload is WINDOW_SIZE * WINDOW_TIME * 2.

const unsigned POSTLOAD_TARGET_DURRATION=900;

// We turn off the motor for this long immedeately after a clap is detected. 
// While this is not required for operation, functionally it serves two purposes...
// Firstly it gives us some external feedback that the clap was accurately detected. Good for troubleshooting.
// Secondly it gives an erie silience (no motors running) after the loud clap, which I quite like. Gives a moment for the clap echos to make an impact.

const unsigned ERIE_SILIENCE_DURRATION=2000;


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



byte window1[WINDOW_SIZE];
byte window2[WINDOW_SIZE];

unsigned window1_total=0;
unsigned window2_total=0;

static millis_t next_sample_time = 0; 

// Reset the clap check filter. It will take about 2 windows until another clap can be detected after reset. 
void resetPolledClapCheck() {

  next_sample_time = 0; 
  
  window1_total = 0;    
  window2_total = 0;         

  memset( window1 , 0x00 , WINDOW_SIZE ); 
  memset( window2 , 0x00 , WINDOW_SIZE ); 
  
}

// Returns true if a clap is detected
// Call more often than once every WINDOW_TIME

uint8_t polledClapCheck() {

  millis_t now = millis();   // Take an atomic snapshot 

  if (now >= next_sample_time) {

    next_sample_time= now + WINDOW_TIME;     // Do it this way for testing so we can have delay()s in there. 
    
    const byte a=readADC(); 

    const byte window1_pull = window1[WINDOW_SIZE-1];
    window1_total -= window1_pull;
    
    const byte window2_pull = window2[WINDOW_SIZE-1];
    window2_total -= window2_pull;

    // Shift the samples in the windows over. I know a ring buffer would be more efficient, but we don't need efficiency here
    // so keep it simple. 
    for( byte i = (WINDOW_SIZE-1) ; i>0 ; i-- ) {
      window1[i]=window1[i-1];
      window2[i]=window2[i-1];      
    }

    window1[0] = a;
    window1_total += a;
    
    window2[0] = window1_pull;
    window2_total += window1_pull;

    if ( window2_total > window1_total ) {

      const unsigned drop = window2_total - window1_total; 
      
      if (drop >= CLAP_DETECT_THRESHOLD) {

        return true;
        
      }
    }
            
  }

  return false;

}


// Delay `delaytime` milliseconds, but return early if a clap is detected while waiting
// Returns ture if clap detected.

uint8_t polledClapCheckDelay( uint16_t delaytime ) {

  millis_t endtime = millis() + delaytime;

  while ( millis() < endtime ) {
    if (polledClapCheck() ) {
      return 1;
    }
  }

  return 0;
  
}


// Total motor on time of last clap cycle,

millis_t prevClapCycleDuration=0;   // clap to clap, excluding time waiting for trigger.


void setup()
{

  initADC();

  initMotor(); 


  // On power up we need a baseline for the length of a clap before to drop into our loop
  // so we run a full cycle to time one.

  // Run until we see a clap...
  motorOn();
  resetPolledClapCheck();
  while (!polledClapCheck());  

  //Puase for a second to simulate the time we will be waiting for a trigger
  // (Also gives external user indication that we did accurately see the clap)
  motorOff();
  delay(1000);
  motorOn();

  resetPolledClapCheck();  

  // Time the next clap cycle
  const millis_t start = millis(); 
  while (!polledClapCheck());  
  const millis_t end = millis(); 

  // Calculate the durration.
  prevClapCycleDuration = end - start;

  // OK, we just clapped, motor is running, we have a timed cycle. 
  // We are ready to drop into normal operation...
}

// By convention, loop() is entered immedeiately after a clap has been detected, motor on.

void loop() {

  motorOff();
  delay(ERIE_SILIENCE_DURRATION);    // Breifly turn off the motor after a clap is detected. Not required, but I like it. 
  motorOn();  

  // "preload" is the time we run after a clap to get the cam ready a spot where will we stop and wait for a trigger

  const uint16_t preload_duration = prevClapCycleDuration - POSTLOAD_TARGET_DURRATION;
  
  resetPolledClapCheck();

  const millis_t preload_start_time = millis();  // Only used if we clap prematurely to compute the cycle time 
  
  if (polledClapCheckDelay( preload_duration )) {
    
    const millis_t preload_end_time = millis();  // Only used if we clap prematurely to compute the cycle time 
    
    prevClapCycleDuration = preload_end_time  - preload_start_time;    // Adjust to the actual clap cycle time, hopefuly next cycle will be better.
    
    return;
  }

  // We are now at the preload location

  motorOff();
  resetPolledClapCheck();
  
  delay(1000);    

  // Triggered. Now run until we detect the clap (should be in about POSTLOAD_TARGET_DURRATION).

  const millis_t postload_start_time = millis();
  motorOn();
  
  while (!polledClapCheck());

  // We just clapped, completing this clap cycle. Compute the time this cycle took so we can be ready for the next one.
  
  const millis_t postload_end_time = millis();

  const millis_t postload_durration = (postload_end_time - postload_start_time);

 // Calculate the total motor run time, which is the combination of the time we spent running to get to the
  // preload position, plus the time we spent running from the preload to the clap (the "post load" time). 

  prevClapCycleDuration = preload_duration + postload_durration;  
  
}
  
