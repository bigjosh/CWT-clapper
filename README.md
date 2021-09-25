# CWT-CLAPPER firmware, 2021 josh.com


## The mission

Be able to synchonise the clapping of the clappers, which requires sensing when the local clapper last clapped so that it can be positioned for the next clap. This is complicated becuase the only intentional sensor on board is a microphone and it is hard to differentiate beween the sound of a local clap and a nearby clap. 


## Theory of operation 

 1. As the cam rotates and pushes the blade, the spring is progessively streched. 
 2. The farther the spring is streched, the harder it is to strech it further (googgle "Hooke's law"). 
 3. The more force it takes to turn the motor, the more current it uses. 
 4. The more current the motor uses, the lower the voltage across the batteries becomes (google "internal resistance"). 
 5. We can indirectly measure the voltage across the battery with the internal ADC (https://wp.josh.com/2014/11/06/battery-fuel-guage-with-zero-parts-and-zero-pins-on-avr/)
 
 Q.E.D.: We can (sort of, barely, sextuple indirectly) measure the position of the blade by measuring the internal voltage 
 bandgap of the chip using the ADC!
  
 In practice, the change in ADC readings between the blade being fully open and fully shut is very small 
 and there is lots of noise also in these readings.
 
 So we aim to detect the most abupt change in that signal - the moment when the blade claps and the motor goes from pushing 
 a fully streched to spring to not pushing on the blade at all (there is a bit of a dead zone after a clap before the cam
 touches the blade again).
  
 To find this edge, we both filter out high freqnecy noise and amplify the longer running signal by taking periodic ADC samples 
 and summing these over a longer window. We have two consecutive windows, so when we get to the moment where one window is
 filled with samples from before the flap and the other is filled with samples from after the clap, there should be enough SNR to detect
 that by comparing the sums of all the values in each window. 
 
## Demo 
 
See them clap hare....
https://www.youtube.com/watch?v=MOf7x3NkilU&lc=UgzuyXRK9PwrqrHehkx4AaABAg.9SaJbr_eWT19Sc9pGRepJp
