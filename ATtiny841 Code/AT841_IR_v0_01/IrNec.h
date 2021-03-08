/*
 * Light Weight NEC Infrared decoder using interrupt instead of timer.
 *
 * Uses a statemachine model. State changes are triggered by edge transitions.
 * Duration of states checked to eliminate implausible data.
 * Needs an external interrupt pin. Could be hacked to use a pin change interrupt
 *  if you need to use other pins. see https://playground.arduino.cc/Main/PinChangeInterrupt/ etc.
 *
 * Data from:
 * https://exploreembedded.com/wiki/NEC_IR_Remote_Control_Interface_with_8051
 * https://www.vishay.com/docs/80071/dataform.pdf
 *
 * Basically an NEC frame has a beader pulse followed by 32 bit pulses as 4 bytes in LSB first order.
 *
 * 26_06_2019 v0_06 bug fix code invertion corrected. Repeat active (gives 0xFFFFFFFF)
 */


#ifndef IRNEC_H_
#define IRNEC_H_

#include <Arduino.h>


namespace nsIrNec {

extern uint32_t dataOut ;
extern volatile uint32_t dataRaw ;


void begin( uint8_t ) ;  // parameter is the chosen EXT INT pin
void loop() ;            // must be called from the main loop()

} // namespace nsIrNec


#endif /* IRNEC_H_ */
