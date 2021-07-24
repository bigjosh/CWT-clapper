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



#define MOTOR 8     // Motor MOSFET pin

void motorOn(){
  digitalWrite(MOTOR, HIGH);  
}

void motorOff(){
  digitalWrite(MOTOR, LOW);  
}


void setup()
{

  pinMode(MOTOR, OUTPUT);

  // For debug info
  Serial.begin( 1000000  );

  // Set up ADC

  ADMUXA = 0b001101;    // Read the internal 1.1 reference. Datasheet 16.13.1
  ADMUXB = 0x00;         // Against the Vcc

  ADCSRA = 0b10000111;    // Enable ADC, No autotrigger or interrupt, divide by 32 (gives 250Khz @ 8Mhz clock). We are suppoed to be less than 200Khz, but should be ok. 

  motorOn(); 

}


// We take an ADC reading every millisecond, basically becuase the finest handy timer Arduino gives us is millis()
// Then we average each 100 readings together to filter noise
// A clap seems to take about 200ms, so we are sampling at 2x 
// Then we compare current sample to the sample 2 windows ago to find the definitive clap
// Skipping the sample in the middle should filter out the transision while the clap is actually happening
// This is all very hard becuase there is less than 1 step of signal in the ADC reading!

#define SAMPLE_COUNT_PER_WINDOW 100      // Max 256 without expanding size of window_total variables
#define SAMPLE_THRESHOLD 50              // How much this window has to drop compared to 2nd previous window to detect a clap. Emperically determined

byte curr_window_sample_count =0; 
unsigned curr_window_total = 0;    

unsigned prev1_window_total = 0;    // Previous window samples
unsigned prev2_window_total = 0;    // Dopo previous window samples

// All times in ms

unsigned long last_clap_time = 0;

unsigned long last_sample_time = 0; 

#define CLAP_BACKOFF_TIME 200     // Aim to run the motor long enough to get this far away from clapping
                                  // Remember that our sample window is 100ms, so we need at least double that to avoid overrun


byte readADC() {

    ADCSRA |= _BV( ADSC );    // Start the conversion    

    while ( ADCSRA &  _BV( ADSC ) );      // Wait for bit to clear (indicates conversion done)

    const uint8_t a = (uint8_t) ADC;                     // Read conversion result. 

    // Note that we toss the high bits of the sample. Our battery voltage range happens to put our readings right in the middle of the ADC low byte range.

    return a;
  
}


// Returns true if a clap is detected
// Call more than once per millisecond

uint8_t polledClapCheck() {

  uint8_t clap_flag=0;

  unsigned long now = millis();   // Take an atomic snapshot 

  if (now >= last_sample_time) {

    last_sample_time= now;     // Do it this way for testing so we can have delay()s in there. 
    
    const byte a=readADC(); 

    curr_window_total += a;      // Accumulate the readings for this window
    
    curr_window_sample_count++;
    
    if (curr_window_sample_count >= SAMPLE_COUNT_PER_WINDOW) {

      // End of current window

      Serial.print( now );
      Serial.print(" ");

      Serial.print( curr_window_total );
      Serial.print(" ");
      Serial.print( prev1_window_total );
      Serial.print(" ");
      Serial.print( prev2_window_total );
      Serial.print(" ");

      // Did we go down since the dopo previous window?

      if ( curr_window_total < prev2_window_total)  {

        // Did we go down by more than the threshold?
        
        
        if  ( (prev2_window_total - curr_window_total) > SAMPLE_THRESHOLD )  {

              // CLAP DETECTED

              clap_flag = 1; 

              // A hacky way to get us a 2 window-period hold-off 
              prev2_window_total = 0;
              prev1_window_total = 0;
              curr_window_total = 0;
              
        }

      }

      Serial.print("\n");

      // Bucket brigade the samples down
      prev2_window_total = prev1_window_total;
      prev1_window_total = curr_window_total;
      curr_window_total = 0;
      curr_window_sample_count=0;
      
      
    }
        
  }

  return clap_flag;
  
}

void loop() {

  if (polledClapCheck()) {
    
    unsigned long now = millis();   // Take an atomic snapshot 

    long unsigned this_clap_time = now; 
  
    // THIS DO IS TESTING ONLY CODE
  
    do {
  
      // Wait a second so people know that we know that we just clapped
      // (This is just in here for testing.) 
      motorOff();
  
      Serial.print("CLAP ");
  
      Serial.print(prev2_window_total - curr_window_total);              
      
      Serial.print(" ");
  
      const unsigned clap_period = this_clap_time - last_clap_time;
  
      // Next time we should stop- aim for backoff-time before the next clap
      // next_stop_time = this_clap_time + clap_period  - CLAP_BACKOFF_TIME_MS;
  
      Serial.print( clap_period/1000.0  );
  
      
      delay(1000);
      motorOn();
      this_clap_time = now;               // Adjust for the fact that we stopped. 
      delay(1000);                        // Warm up motor. Prevents us from seeing the same clap again.
      
  
    } while (0);
    
  
    last_clap_time = this_clap_time;
    
  }  

  
}
