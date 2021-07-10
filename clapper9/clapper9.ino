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

  // Set clock division factor = 1, run directly off internal 8Mhz clock. Datasheet 6.6.2
  // Need to unlock CCP first. Datasheet 4.8.1

  CCP = 0xd8;
  CLKPR = 0x00;     

  
  pinMode(MOTOR, OUTPUT);
  turnMotorOff = false;
  digitalWrite(MOTOR, HIGH);

  Serial.begin( 250000 );

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
#define SAMPLE_PERIOD_MS 100

float rolling_average = 0;    // Start at zero so we will slowly build up to working voltage

void loop()
{

  if (millis() >= next_sample) {

    next_sample = millis() + SAMPLE_PERIOD_MS;

    ADCSRA |= _BV( ADSC );    // Start the first conversion    

    while ( ADCSRA &  _BV( ADSC ) );      // Wait for bit to clear (indicates conversion done)

    const uint8_t a = (uint8_t) ADC;                     // Read conversion result. 

    const uint8_t a1 = new_samples.add_sample( a ); 
    const uint8_t a2 = mid_samples.add_sample( a1 );
    old_samples.add_sample( a2 );

    Serial.print( old_samples.get_average() );
    Serial.print(" ");
    Serial.print( mid_samples.get_average() );
    Serial.print(" ");    
    Serial.print( new_samples.get_average() );
    Serial.print("\n");
  }
  
}
