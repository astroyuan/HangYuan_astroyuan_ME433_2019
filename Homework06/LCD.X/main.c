#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h>
//#include"i2c_master_utilities.h"
#include"ili9341.h"
#include<stdio.h>

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF// no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable secondary osc
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1 // use slowest wdt
#pragma config WINDIS = OFF // wdt no window mode
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

void LED_blink_init()
{
    // setup initial pins configuration for LED
    TRISAbits.TRISA4 = 0; //A4 output
    TRISBbits.TRISB4 = 1; //B4 input
    LATAbits.LATA4 = 1; //A4
}

int main() {

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
    
    // functionality control
    int LED_blink_flag = 1;
    int LCD_flag = 1;

    // do your TRIS and LAT commands here
    if(LED_blink_flag == 1) LED_blink_init();
    if(LCD_flag == 1)
    {
        SPI1_init();
        LCD_init();
        LCD_clearScreen(ILI9341_BLACK);
    }

    __builtin_enable_interrupts();
    
    // frequency config
    int SysCLK_freq = 48e6; // system clock frequency
    int LoopCLK_freq = SysCLK_freq/2; // main loop frequency
    // LED
    int LED_blink_freq = 2; // LED blink frequency
    // LCD
    int LCD_COM_freq = 10;
    
    // periods
    int T_LED_BLINK = LoopCLK_freq/LED_blink_freq/2;
    int T_LCD_COM = LoopCLK_freq/LCD_COM_freq;
    int T_TIMER_RESET;
    
    // init counter
    _CP0_SET_COUNT(0);
    int TimerStart = _CP0_GET_COUNT();
    int TimerNow = _CP0_GET_COUNT();
    int Timer_LED = TimerStart;
    int Timer_LCD = TimerStart;
    
    int progress = 0;

    while(1) {
	// use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
	// remember the core timer runs at half the sysclk
         /*-------------------------------------
         -------- LCD Functionalities ----------
         --------------------------------------*/
        if (LCD_flag == 1)
        {
            int pos_x, pos_y, pos_idx;
            int frame_timer;
            double fps;
            char s[10];
            if (_CP0_GET_COUNT() - Timer_LCD > T_LCD_COM)
            {
                frame_timer = _CP0_GET_COUNT();
                pos_x=28;
                pos_y=32;
                pos_idx = LCD_drawString("hang@hang-TFTLCD",pos_x,pos_y,ILI9341_GREEN,ILI9341_BLACK);
                pos_x = pos_idx / ILI9341_TFTWIDTH;
                pos_y = pos_idx % ILI9341_TFTWIDTH;
                pos_idx = LCD_drawString(":~$ Hello world!",pos_x,pos_y,ILI9341_WHITE,ILI9341_BLACK);
                sprintf(s, " %d%%", progress);
                pos_x = pos_idx / ILI9341_TFTWIDTH;
                pos_y = pos_idx % ILI9341_TFTWIDTH;
                LCD_drawString(s,pos_x,pos_y,ILI9341_ORANGE,ILI9341_BLACK);
                pos_x=28;
                pos_y=32+8;
                LCD_drawProgressBar(pos_x, pos_y, 100, 8, progress, ILI9341_RED, ILI9341_BLACK);
                progress=(progress+1)%101;
                Timer_LCD = _CP0_GET_COUNT();
                
                // show fps
                fps = LoopCLK_freq / (double)(Timer_LCD - frame_timer);
                sprintf(s, "fps: %.2f", fps);
                LCD_drawString(s,0,0,ILI9341_YELLOW, ILI9341_BLACK);
            }
        }
        /*-------------------------------------
         ------- LED Functionalities ----------
         --------------------------------------*/
        if (LED_blink_flag == 1)
        {
            if(_CP0_GET_COUNT() - Timer_LED > T_LED_BLINK)
            {
                LATAbits.LATA4 = !LATAbits.LATA4;
                Timer_LED = _CP0_GET_COUNT();
            }
            while (!PORTBbits.RB4){LATAbits.LATA4 = 0;}
        }
        
        // Handle timer overflow pending
    }
}