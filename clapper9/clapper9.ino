//http://drazzy.com/package_drazzy.com_index.json
//add ATTinyCore boards
//select attiny 441 (no bootloader)
//8mhz internal >4.5V. Note that this can run up to 10Mhz at 2.7V. Datasheet 25.1.3

//add to enable ICE programmer for ATTiny core
//add to programmers.txt file under Library/Arduino15/packages/ATTinyCore/hardware/avr/1.4.1
/*

*/


unsigned int waitTime = 4000; //pause between claps

const int sampleWindow = 20; // Sample window for mic
const float threshold = 550.0; //threshold for neighbor clap
//const float selfThreshold = 999.0234; //threshold for self clap // maxed
const float selfThreshold = 1022; //threshold for self clap // maxed

boolean turnMotorOff;

int cycleCount;

#define MOTOR 8
float selfCount = 0;
unsigned int clap; //clap volume

void setup()
{


  
  pinMode(MOTOR, OUTPUT);
  turnMotorOff = false;
  digitalWrite(MOTOR, HIGH);

  Serial.begin( 1000000  );

  // Set up ADC 

  ADMUXA = 0b001101;    // Read the internal 1.1 reference. Datasheet 16.13.1
  ADMUXB = 0x00;         // Against the Vcc

  ADCSRA = 0b10000111;    // Enable ADC, No autotrigger or interrupt, divide by 32 (gives 250Khz @ 8Mhz clock). We are suppoed to be less than 200Khz, but should be ok. 


}

template <class T, unsigned size>
struct unsigned_rolling_average {

  T buffer[ size ];

   T *end = buffer + size;

  T *next=buffer;

  unsigned long total;

  // Add a new value, returns old value that was removed.
  T add_sample( T v ) {

    T pulled_value = *next;

    total -= pulled_value;

    total+=v;    

    *next = v;

    next++;
    if (next==end) next=buffer;

    return pulled_value;
    
  }

  unsigned get_average(void) {

    return total/size;
    
  }
  
};

const unsigned buffer_size = 10;

unsigned_rolling_average<uint8_t , buffer_size> old_samples;
unsigned_rolling_average<uint8_t , buffer_size> mid_samples;
unsigned_rolling_average<uint8_t , buffer_size> new_samples;

unsigned long next_sample = 0;

// We take a reading every millisecond 
/// ..and average each 100 together to filter noise
// A clap seems to take about 200ms, so we are sampling at 2x 
// and then we compare pre and post samples to find the definitive clap
// this is very hard becuase there is less than 1 bit of siggnal in the ADC reading!

#define SAMPLE_PERIOD_MS 1
#define SAMPLE_COUNT_PER_WINDOW 100      // Max 256 without expanding size of window_total variables
#define SAMPLE_THRESHOLD (SAMPLE_COUNT_PER_WINDOW/2)  // We have to move down by more than 1/2 of an ADC step. Sort of a low pass filter. 

byte curr_window_sample_count =0; 
unsigned curr_window_total = 0;    

unsigned prev1_window_total = 0;    // Previous window samples
unsigned prev2_window_total = 0;    // Dopo previous window samples

byte direction=0;             // 1 up, 0 down 

void loop()
{

  if (millis() >= next_sample) {

    next_sample +=  SAMPLE_PERIOD_MS;     // Make sure nothing in loop takes more than 100ms!

    ADCSRA |= _BV( ADSC );    // Start the first conversion    

    while ( ADCSRA &  _BV( ADSC ) );      // Wait for bit to clear (indicates conversion done)

    const uint8_t a = (uint8_t) ADC;                     // Read conversion result. 

    curr_window_total += a;      
    
    curr_window_sample_count++;
    if (curr_window_sample_count >= SAMPLE_COUNT_PER_WINDOW) {

      // End of current window

      Serial.print( millis() );
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
                                
              Serial.print("CLAP");

              // A hacky way to get us a 2 window-period hold-off until next clap.
              prev2_window_total = 0;
              prev1_window_total = 0;
              curr_window_total = 0;

              
        }

      }

      Serial.print("\n");

      // Bucket bridage the samples down
      prev2_window_total = prev1_window_total;
      prev1_window_total = curr_window_total;
      curr_window_total = 0;
      curr_window_sample_count=0;

      
      
    }
        
  }
  
}
