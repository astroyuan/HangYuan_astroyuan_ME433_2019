/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    appData.LED_blink_flag = 1;
    appData.LCD_flag = 1;
    appData.IMU_flag = 1;
    
    appData.SysCLK_freq = 48e6;
    appData.LoopCLK_freq = appData.SysCLK_freq/2;
    appData.LED_blink_freq = 2;
    appData.LCD_COM_freq = 20;
    
    appData.T_LED_BLINK = appData.LoopCLK_freq/appData.LED_blink_freq/2;
    appData.T_LCD_COM = appData.LoopCLK_freq/appData.LCD_COM_freq;
    
    _CP0_SET_COUNT(0);
    appData.TimerStart = _CP0_GET_COUNT();
    appData.TimerNow = _CP0_GET_COUNT();
    appData.Timer_LED = appData.TimerStart;
    appData.Timer_LCD = appData.TimerStart;
    
    appData.fullscale = ILI9341_TFTWIDTH-4;
    
    appData.SLAVE_ADDRESS = 0b1101011;
    
    // ports
    if (appData.LED_blink_flag) LED_blink_init();
    if (appData.LCD_flag)
    {
        SPI1_init();
        LCD_init();
        LCD_clearScreen(ILI9341_BLACK);
    }
    if (appData.IMU_flag)
    {
        I2C_master_setup();
        if (I2C2_getIMU_Address() != 105)
            LCD_drawString("Error: Unexpected IMU address.", 0, 0, ILI9341_RED, ILI9341_BLACK);
        I2C2_initIMU();
    }
}

void LED_blink_init()
{
    // setup initial pins configuration for LED
    TRISAbits.TRISA4 = 0; //A4 output
    TRISBbits.TRISB4 = 1; //B4 input
    LATAbits.LATA4 = 1; //A4
}

void I2C2_setIMU(unsigned char reg_address, unsigned char config_byte)
{
    // set bits of IMU
    I2C_master_start();                       // start bit
    I2C_master_send(appData.SLAVE_ADDRESS << 1 | 0);  // send control byte 0 for writing
    I2C_master_send(reg_address);             // send register address
    I2C_master_send(config_byte);             // send configuration bits
    I2C_master_stop();                        // stop bit
}

void I2C2_getIMUdata(unsigned char reg_address, unsigned char *data, int length)
{
    // get sequential data from IMU
    int i;
    I2C_master_start();                             //start bit
    I2C_master_send(appData.SLAVE_ADDRESS << 1 | 0);        // send control byte 0 for writing
    I2C_master_send(reg_address);                   // register address
    I2C_master_restart();
    I2C_master_send(appData.SLAVE_ADDRESS << 1 | 1);        // send control byte 1 for reading
    
    for (i=0;i<length-1;i++)
    {
        data[i] = I2C_master_recv();
        I2C_master_ack(0);
    }
    data[i+1] = I2C_master_recv();
    I2C_master_ack(1);
    I2C_master_stop();
}

void I2C2_initIMU()
{
    // initialize I2C2 slave device
    
    // CTRL1_XL register sample rate 1.66kHz 2g sensitivity 100Hz filter
    //0b 1000 00 10
    I2C2_setIMU(0x10, 0b10000010);
    // CTRL2_G register sample rate 1.66kHz 1000 dps sensitivity
    //0b 1000 10 0 0
    I2C2_setIMU(0x11, 0b10001000);
    // CTRL3_C enable IF_INC
    //0b 0 0 0 0 0 1 0 0
    I2C2_setIMU(0x12, 0b00000100);
}

unsigned char I2C2_getIMU_Address()
{
    // get WHO_AM_I bits
    I2C_master_start();                          // start bit
    I2C_master_send(appData.SLAVE_ADDRESS << 1 | 0);     // send control byte 0 for writing
    I2C_master_send(0x0F);                       // GPIO register 0x09
    I2C_master_restart();
    I2C_master_send(appData.SLAVE_ADDRESS << 1 | 1);     // send control byte 1 for reading
    unsigned char recv_byte = I2C_master_recv(); // get GPIO byte
    I2C_master_ack(1);                           // ack done
    I2C_master_stop();                           // stop bit
    return recv_byte;
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    if (appData.LED_blink_flag == 1)
    {
        if(_CP0_GET_COUNT() - appData.Timer_LED > appData.T_LED_BLINK)
        {
            LATAbits.LATA4 = !LATAbits.LATA4;
            appData.Timer_LED = _CP0_GET_COUNT();
        }
        while (!PORTBbits.RB4){LATAbits.LATA4 = 0;}
    }
    
    if (appData.LCD_flag == 1)
        {
            if (_CP0_GET_COUNT() - appData.Timer_LCD > appData.T_LCD_COM)
            {
                appData.frame_timer = _CP0_GET_COUNT();
                appData.pos_x=0;
                appData.pos_y=0;
                // start from OUT_TEMP_L to OUTZ_H_XL
                I2C2_getIMUdata(0x20, appData.data, 14);
                // decode data block
                appData.temperature = (appData.data[0] & 0x00ff) | (((short)appData.data[1])<<8);
                appData.gyroX = (appData.data[2] & 0x00ff) | (((short)appData.data[3])<<8);
                appData.gyroY = (appData.data[4] & 0x00ff) | (((short)appData.data[5])<<8);
                appData.gyroZ = (appData.data[6] & 0x00ff) | (((short)appData.data[7])<<8);
                appData.accelX = (appData.data[8] & 0x00ff) | (((short)appData.data[9])<<8);
                appData.accelY = (appData.data[10] & 0x00ff) | (((short)appData.data[11])<<8);
                appData.accelZ = (appData.data[12] & 0x00ff) | (((short)appData.data[13])<<8);
                sprintf(appData.s, "Temperature: %i            ", appData.temperature);
                LCD_drawString(appData.s,appData.pos_x,appData.pos_y,ILI9341_ORANGE,ILI9341_BLACK);
                sprintf(appData.s, "Gyroscope: %i %i %i           ", appData.gyroX, appData.gyroY, appData.gyroZ);
                LCD_drawString(appData.s,appData.pos_x,appData.pos_y+8,ILI9341_ORANGE,ILI9341_BLACK);
                sprintf(appData.s, "Accelerometer: %i %i %i           ", appData.accelX, appData.accelY, appData.accelZ);
                LCD_drawString(appData.s,appData.pos_x,appData.pos_y+16,ILI9341_ORANGE,ILI9341_BLACK);
                // scale values in unit of 1g
                appData.Xcomp = ((double)appData.accelX)/SHRT_MAX * 2; // scale by 1g
                appData.Ycomp = ((double)appData.accelY)/SHRT_MAX * 2; // scale by 1g
                // remap for display length
                appData.Xdelta = (short)(appData.Xcomp*(double)appData.fullscale);
                appData.Ydelta = (short)(appData.Ycomp*(double)appData.fullscale);

                LCD_drawCross(120, 160, appData.fullscale, appData.Xdelta, appData.Ydelta, ILI9341_RED, ILI9341_YELLOW);
                
                appData.Timer_LCD = _CP0_GET_COUNT();
            }
        }
}

 

/*******************************************************************************
 End of File
 */
