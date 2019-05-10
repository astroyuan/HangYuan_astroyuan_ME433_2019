#ifndef I2C_MASTER_UTILITIES_H__
#define I2C_MASTER_UTILITIES_H__
// Header file for i2c_master_utilities.c
// helps implement use I2C2 as a master without using interrupts
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro

void I2C_master_setup(void);              // set up I2C 2 as a master, at 100 kHz

void I2C_master_start(void);              // send a START signal
void I2C_master_restart(void);            // send a RESTART signal
void I2C_master_send(unsigned char byte); // send a byte (either an address or data)
unsigned char I2C_master_recv(void);      // receive a byte of data
void I2C_master_ack(int val);             // send an ACK (0) or NACK (1)
void I2C_master_stop(void);               // send a stop

#endif