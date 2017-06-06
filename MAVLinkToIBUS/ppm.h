//copyright povlhp
//source https://github.com/povlhp/iBus2PPM
/*licnese
povlhp/iBus2PPM is licensed under the
GNU General Public License v3.0
Permissions of this strong copyleft license are conditioned on making available complete source code of licensed works and modifications, which include larger works using a licensed work, under the same license. Copyright and license notices must be preserved. Contributors provide an express grant of patent rights.
// iBus2PPM v2 version 1.01
// Arduino Nano/Pro code to read FlySky iBus and output
// x channels of CPPM
// BaseFlight supports iBus, CleanFlight got it, and even betaflight.
// But since iBus is 115k2 serial, it requires an UART, sacrificing the ability to
// use GPS on the NAZE32.
// Another use for this is as a link to a 433MHz radio Tx
//          FlySky i6 (with 10 channel patch) -> iA6B -> iBus2PPM -> 433MHz LRS
// Nobody wants to use trainer cable if they can run wireless.

// I use Turnigy TGY-i6 aka Flysky FS-i6 transmitter and iA6B Rx. I am running hacked firmware from
// this thread to get 10 channels over iBus: http://www.rcgroups.com/forums/showthread.php?t=2486545
// the i6 has the 4 channels for the sticks, 4 switches, and 2 analog potentiometers. So can generate
// 10 independent channels.
// Latest hacked firmware allow to combine 2 switches to one 4 or 6 channel switch, which I use for flight modes,
// thus I get only 9 channels (could make channel 10 a derived channel).
// As a result, I chose to send  9 channels over PPM
// Unfortunately, there is no way to input high channels in trainer mode yet. Would be nice for head-tracker.
*/
 
#pragma once
#include <stdio.h>
#include <stdlib.h>
#ifndef IBUS_TO_PPM
#define IBUS_TO_PPM

#define FAIL_SAFE_AFTER_MISSING_FRAMES  5
#define PPM_CHANS 8   
#define IBUS_MAXCHANNELS 14

#define CH1 0
#define CH2 1
#define CH3 2
#define CH4 3
#define CH5 4
#define CH6 5
#define CH7 6
#define CH8 7
#define CH9 8
#define CH10 9
#define CH11 10
#define CH12 11
#define CH13 12
#define CH14 13

static uint16_t channelMap[IBUS_MAXCHANNELS] = { CH1, CH2, CH3, CH4, CH9, CH10, CH7, CH8, CH5, CH6, CH11, CH12, CH13, CH14 };

//////////////////////PPM CONFIGURATION///////////////////////////////
///// PPM_FrLen might be lowered a bit for higher refresh rates, as all channels
///// will rarely be at max at the same time. For 8 channel normal is 22.5ms.

#define PPM_PulseLen 400  //set the pulse length
#define PPM_Pause 3500    // Pause between PPM frames in microseconds (1ms = 1000µs) - Standard is 6500
#define PPM_FrLen (((1700+PPM_PulseLen) * PPM_CHANS)  + PPM_Pause)  //set the PPM frame length in microseconds 
    // PPM_FrLen can be adjusted down for faster refresh. Must be tested with PPM consumer (Flight Controller)
    // PPM_VariableFrames uses variable frame length. I.e. after writing channels, wait for PPM_Pause, and then next packet.
    // Would work as long as PPM consumer uses level shift as trigger, rather than timer (standard today).
    // 8 channels could go from 22500 us to an average of 1500 (center) * 8 + 3500 = 15500 us. That is
    // cutting 1/3rd off the latency.
    // For fastest response, make sure as many values as possible are low. I.e. fast response flight mode has lower value.
    // Make sure unused channels are assigned a switch set to value 1000. Or configure to fewer channels PPM .
#define PPM_VariableFrames 0  // Experimental. Cut down PPM latency. Should work on most Flight Controllers using edge trigger.
#define PPM_offset -7 // How much are the channels offset  ? Compensate for timer difference, CPU spent elsewhere
            // Use this to ensure center is 1500. Then use end-point adjustments on Tx to hit endpoints.
#define onState 0  //set polarity: 1 is positive, 0 is negative
#define sigPin 5  //set PPM signal digital pin on the arduino

//////////////////////////////////////////////////////////////////
static uint16_t rcFailsafe [IBUS_MAXCHANNELS] = {  1500, 1500, 950, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000 };
static uint16_t rcValue[IBUS_MAXCHANNELS] = {  1500, 1500, 950, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000 };
volatile byte currentChannel = 0;
void setupPpm(){  
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  currentChannel = 0;
  cli();
  TCCR3A = 0; // set entire TCCR4 register to 0
  TCCR3B = 0;
  OCR3A = 100;  // compare match register, changed on the fly. First call in 100*0.5us.
  TCCR3B |= (1 << WGM12);  // turn on CTC mode
  TCCR3B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK3 |= (1 << OCIE3A); // enable timer compare interrupt
  sei();
}
void updateChannelData(uint8_t* ibusRXBuff, bool checkSum){
    uint8_t i;
    uint8_t offset;
    uint8_t index;
    uint32_t sum = 0;
    if(checkSum){
      for (i = 0, offset = 2; i < PPM_CHANS; i++, offset += 2) {
          rcValue[channelMap[i]] = (uint16_t)ibusRXBuff[offset] | (uint16_t)(ibusRXBuff[offset + 1] << 8);
      }
    }
    else{
        memcpy(rcValue, rcFailsafe, PPM_CHANS * sizeof(uint16_t));
    }
}
ISR(TIMER3_COMPA_vect){
  static boolean state = true;
  cli();
  TCNT3 = 0;
  if(state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR3A = PPM_PulseLen * 2;
    state = false;
  }
  else{  //end pulse and calculate when to start the next pulse
    static unsigned int calc_signal; 
    digitalWrite(sigPin, !onState);
    state = true;
    if(currentChannel >= PPM_CHANS){
      currentChannel = 0;
      if (PPM_VariableFrames) {
        OCR3A = PPM_Pause * 2;  // Wait for PPM_Pause
      } else { // static frame length
        calc_signal = calc_signal + PPM_PulseLen; //Compute time spent
        OCR3A = (PPM_FrLen - calc_signal) * 2;  // Wait until complete frame has passed
      }
      calc_signal = 0;
    }
    else{                                    
      OCR3A = (rcValue[currentChannel] - PPM_PulseLen) * 2 + (2*PPM_offset); // Set interrupt timer for the spacing = channel value                                                                                                        
      calc_signal +=  rcValue[currentChannel];
      currentChannel++;
    }     
  }
  sei();
}
#endif
