/*
 * Turn on the motor for 1 sec any time an audible clap is detected.
 */


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
//    digitalWrite(MOTOR, HIGH);


}

void loop()
{
  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;
  unsigned int signalCount = 0;

  // collect data sample Window
  while (signalCount < sampleWindow)
  {
    clap = analogRead(0);
    if (clap < 1024)  //This is the max of the 10-bit ADC so this loop will include all readings
    {
      if (clap > signalMax) signalMax = clap;  // save just the max levels
      else if (clap < signalMin) signalMin = clap;  // save just the min levels
    }
    signalCount++;
  }

  unsigned int peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
  //float mic = (peakToPeak * 1000.0) / 1024.0;  // convert to percentage
  float mic = peakToPeak;

  //self clap
  if (mic > selfThreshold) {
    turnMotorOff = true;
    selfCount = 0; //reset count
  }

  //neighbor clap
  else if (mic > threshold) {
    if (selfCount <= waitTime) {

      selfCount = 0;

    }
  }

  if (turnMotorOff && selfCount > 10) {
    digitalWrite(MOTOR, LOW);
    turnMotorOff = false;
  }

  if (selfCount > waitTime ) {
    digitalWrite(MOTOR, HIGH);
  }

  selfCount++; //always count up
}
