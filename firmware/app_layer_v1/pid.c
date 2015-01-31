/* 
 * File:   pid.c
 * Author: johnlam
 *
 * Created on January 5, 2015, 1:18 PM
 */

#include "pid.h"

#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <libpic30.h>
#include <PPS.h>

#include "Compiler.h"
#include "logging.h"
#include "protocol.h"
#include "pins.h"
#include "sync.h"
#include "pwm.h"


typedef struct{
    int dirPin;
    int pwmPin;
    long P, I, D;
    long samples[5];
    long velocity;
    long dest;
    long err;
}PID;

PID channels[4];

void PIDInit(){
  int i,j;
  TMR4  = 0x0000;  // reset counter
  _T4IF = 0;
  _T4IE = 1;
  
  //PPSUnLock;
  iPPSInput( IN_FN_PPS_INT4, PinToRpin(14));
  iPPSInput( IN_FN_PPS_INT3, PinToRpin(13));
  iPPSInput( IN_FN_PPS_INT2, PinToRpin(12));
  iPPSInput( IN_FN_PPS_INT1, PinToRpin(11));
  //PPSLock;

  _INT4IF = 0;
  _INT4IE = 0;
  _INT4EP = 1;

  _INT3IF = 0;
  _INT3IE = 0;
  _INT3EP = 1;

  _INT2IF = 0;
  _INT2IE = 0;
  _INT2EP = 1;

  _INT1IF = 0;
  _INT1IE = 0;
  _INT1EP = 1;
  for (i=0;i<4;i++){
      channels[i].dirPin = 44-i;
      channels[i].pwmPin = i+1;
      channels[i].P = 0;
      channels[i].I = 0;
      channels[i].D = 0;
      channels[i].err = 0;
      channels[i].velocity = 0;
      channels[i].dest = 0;
      for(j=0;j < 5;j++){
         channels[i].samples[j] = 0;
      }
  }
}

void PIDConfig(int chn,long P,long I,long D){
    channels[chn-1].P = P;
    channels[chn-1].I = I;
    channels[chn-1].D = D;
}

void PIDSet(int chn,long val){
    channels[chn-1].dest = val;
    OUTGOING_MESSAGE msg;
    msg.type = PID_REPORT;
    msg.args.pid_get.chn = chn;
    msg.args.pid_get.val = channels[chn-1].samples[0];
    AppProtocolSendMessage(&msg);
}

void __attribute__((__interrupt__, auto_psv)) _T4Interrupt() {
    //update PID here
    int i,j;
    for(i=0;i<4;i++){
        for(j=3;j>0;j--){
            channels[i].samples[j] = channels[i].samples[j-1];
        }
        channels[i].samples[0] = channels[i].velocity;
        channels[i].velocity = 0;
        int speed = channels[i].dest;
        // send drive command to motors
        SetPwmDutyCycle(channels[i].pwmPin, abs(speed) >> 8,1);
        PinSetLat(channels[i].dirPin,speed>0?1:0);
    }
    _T4IF = 0;
}

void __attribute__((__interrupt__, auto_psv)) _INT4Interrupt() {
    channels[3].velocity += PinGetPort(20)?1:-1;
    _INT4IF = 0;
}

void __attribute__((__interrupt__, auto_psv)) _INT3Interrupt() {
    channels[2].velocity += PinGetPort(19)?1:-1;
    _INT3IF = 0;
}

void __attribute__((__interrupt__, auto_psv)) _INT2Interrupt() {
    channels[1].velocity += PinGetPort(18)?1:-1;
    _INT2IF = 0;
}
void __attribute__((__interrupt__, auto_psv)) _INT1Interrupt() {
    channels[0].velocity += PinGetPort(17)?1:-1;
    _INT1IF = 0;
}

