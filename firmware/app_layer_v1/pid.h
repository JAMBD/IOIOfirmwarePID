/* 
 * File:   pid.h
 * Author: johnlam
 *
 * Created on January 5, 2015, 12:15 PM
 */

#ifndef PID_H
#define	PID_H

void PIDInit();

void PIDConfig(int chn,long P,long I,long D);

void PIDSet(int chn,long val);


#endif	/* PID_H */

