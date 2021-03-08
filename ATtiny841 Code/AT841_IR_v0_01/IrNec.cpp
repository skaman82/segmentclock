/*
   IrNec.cpp

  See IrNec.h for documentation
*/

#include "IrNec.h"

namespace nsIrNec {


enum class state_t { pendingFrame , inHeaderMark, inHeaderSpace, inBitMark, inBitSpace, inRepeat,  dataReady } ;

volatile state_t state = state_t::pendingFrame  ;
uint8_t ir_pin  ;     // ir_pin must be an external interrupt pin (e.g. only 2 or 3 on a Uno)
uint32_t frameStartAtMs = 0 ;
uint32_t frameStartAtMs_last = 0 ;  // for repeat
volatile uint32_t dataRaw = 0;
uint32_t dataOut = 0 ;
uint32_t bitSpaceStartAtUs = 0 ;
uint8_t bitcount = 0;
uint32_t inHeaderSpaceAtUs = 0 ;




void isr( ) {
  /*
     called by external interrupt
     tests timings are within tolerance range for noise immunity.
     sets status back to pendingFrame to drop a failed frame.
  */

  uint32_t ms = millis() ;
  uint32_t us = micros() ;

  bool isFalling = digitalRead( ir_pin ) ;  // isFalling - trigger was falling edge (inverted logic)
  bool isRising = ! isFalling;              // isRising - trigger was rising edge  (inverted logic)

  if ( ms - frameStartAtMs >= 150 ) state = state_t::pendingFrame ;  // force clean up of any failed frame

  if ( state == state_t::pendingFrame  ) {
    //digitalWrite( 13, HIGH ) ;
    frameStartAtMs = ms ;
    bitcount = 0 ;
    dataRaw = 0 ;
    if ( isRising ) {
      state = state_t::inHeaderMark ;
    }
  }
  else if ( state == state_t::inHeaderMark ) {
     //digitalWrite( 13, LOW ) ;
    if ( isFalling && ms - frameStartAtMs >= 7 && ms - frameStartAtMs <= 11  ) { // spec = 9mS
      state = state_t::inHeaderSpace ;
      inHeaderSpaceAtUs = us ;
    }
    else state = state_t::pendingFrame ; // error so quit
  }
  else if ( state == state_t::inHeaderSpace ) {
    if ( isRising && us - inHeaderSpaceAtUs >= 3500  && us - inHeaderSpaceAtUs <= 5500 ) { // spec = 4.5mS
      state = state_t::inBitMark ;
    }
    else if ( isRising && us - inHeaderSpaceAtUs >= 1750  && us - inHeaderSpaceAtUs <= 3250  &&  ms - frameStartAtMs_last >= 90 &&  ms - frameStartAtMs_last <= 130 ) { // spec = 2.5mS / 110mS
      state = state_t::inRepeat ;
    }
    else state = state_t::pendingFrame ;  // error so quit
  }
  else if (  state == state_t::inRepeat ) {
    if ( isFalling ) {
      bitcount = 0 ;
      state = state_t::dataReady ;
      frameStartAtMs_last = frameStartAtMs ;
      dataRaw = 0xFFFFFFFF ;
    }
    else state = state_t::pendingFrame ;  // error so quit
  }
  else if ( state == state_t::inBitMark ) {
    if ( isFalling ) {
      state = state_t::inBitSpace ;
      bitSpaceStartAtUs = us ;
    }
    else state = state_t::pendingFrame ;  // error so quit
  }
  else if ( state == state_t::inBitSpace ) {
    if (  isRising ) {
      state = state_t::inBitMark ;
      if ( us - bitSpaceStartAtUs > 1400 &&  us - bitSpaceStartAtUs < 1900) {  //  0 = 562uS ; 1 = 1688 uS
        // 1 found
        bitSet( dataRaw, bitcount) ;
      }
      else if ( us - bitSpaceStartAtUs > 400 &&  us - bitSpaceStartAtUs < 750 ) {  //  0 = 562uS ; 1 = 1688 uS
        // 0 found - do nothing
      }
      else state = state_t::pendingFrame ; // error so quit

      if ( ++bitcount >= 32 ) {
        bitcount = 0 ;
        state = state_t::dataReady ;
        frameStartAtMs_last = frameStartAtMs ;
      }
    }
    else state = state_t::pendingFrame ; // error so quit
  }
}



void begin( uint8_t intPin) {
  ir_pin = intPin ;

  pinMode( ir_pin, INPUT_PULLUP ) ;

  state = state_t::pendingFrame ;

  //attachInterrupt( digitalPinToInterrupt(ir_pin) , isr, CHANGE ) ;
  attachInterrupt( 0, isr, CHANGE ) ; //ATTINY85 pin 2 (also atmega)
}


void loop() {


  // convert raw data bytes (LSB first order) to output format
  if ( state == state_t::dataReady ) {
    dataOut = 0 ;
    for ( uint8_t j = 0 ; j < 32 ; j++ ) {
      bitWrite( dataOut,  ( 31 - j ) , bitRead( dataRaw ,  j  ) ) ;
    }
    state = state_t::pendingFrame ;
  }

}
} // namespace nsIrNec
