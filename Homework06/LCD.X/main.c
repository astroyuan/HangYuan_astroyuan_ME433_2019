#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h>
#include"i2c_master_utilities.h"

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

#define CS LATBbits.LATB15
#define SLAVE_ADDRESS 0b0100000 // 0100 + A2 A1 A0 + (W/R)

void LED_blink_init()
{
    // setup initial pins configuration for LED
    TRISAbits.TRISA4 = 0; //A4 output
    TRISBbits.TRISB4 = 1; //B4 input
    LATAbits.LATA4 = 1; //A4
}

void I2C2_setExpander(unsigned char reg_address, unsigned char config_byte)
{
    // set bits for expander
    I2C_master_start();                       // start bit
    I2C_master_send(SLAVE_ADDRESS << 1 | 0);  // send control byte 0 for writing
    I2C_master_send(reg_address);             // send register address
    I2C_master_send(config_byte);                  // send configuration bits
    I2C_master_stop();                        // stop bit
}

void I2C2_initExpander()
{
    // initialize I2C2 slave device
    
    // set up I/O ports IODIR 0x00
    // GP7 - GP4 as input ports GP3 - GP0 as output ports
    I2C2_setExpander(0x00, 0b11110000);
    
    // set up Output Latch OLAT 0x0A
    I2C2_setExpander(0x0A, 0b00001111);
}

unsigned char I2C2_getExpander()
{
    // get GPIO bits of expander
    I2C_master_start();                          // start bit
    I2C_master_send(SLAVE_ADDRESS << 1 | 0);     // send control byte 0 for writing
    I2C_master_send(0x09);                       // GPIO register 0x09
    I2C_master_restart();
    I2C_master_send(SLAVE_ADDRESS << 1 | 1);     // send control byte 1 for reading
    unsigned char recv_byte = I2C_master_recv(); // get GPIO byte
    I2C_master_ack(1);                           // ack done
    I2C_master_stop();                           // stop bit
    return recv_byte;
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
    int SPI_flag = 1;
    int I2C_flag = 0;

    // do your TRIS and LAT commands here
    if(LED_blink_flag == 1) LED_blink_init();
    if(SPI_flag == 1) SPI1_init();
    if(I2C_flag == 1) I2C_master_setup();
    if(I2C_flag == 1) I2C2_initExpander();

    __builtin_enable_interrupts();
    
    // frequency config
    int SysCLK_freq = 48e6; // system clock frequency
    int LoopCLK_freq = SysCLK_freq/2; // main loop frequency
    int DAC_COM_freq = 1e3; // 1KHz DAC communication frequency
    // LED
    int LED_blink_freq = 2; // LED blink frequency
    // DAC
    int DAC_A_freq = 10; // channel A frequency
    int DAC_B_freq = 5; // channel B frequency
    // I/O Expander
    int Expander_freq = 1e3; // I2C2 communication frequency
    
    // scaled time variables
    double t_A=0.0;
    double t_B=0.0;
    
    // voltages
    int voltage_res = 0xfff; // 12 bits voltage resolution
    double vol_A_amp=1;  // volt
    double vol_B_amp=1;  // volt
    double vol_A_val=0.0;
    double vol_B_val=0.0;
    int vol_A_bits = 0x0;
    int vol_B_bits = 0x0;
    
    // periods
    int T_DAC_COM = LoopCLK_freq/DAC_COM_freq;
    int T_LED_BLINK = LoopCLK_freq/LED_blink_freq/2;
    int T_DAC_A = LoopCLK_freq/DAC_A_freq;
    int T_DAC_B = LoopCLK_freq/DAC_B_freq;
    int T_I2C2_COM = LoopCLK_freq/Expander_freq;
    int T_TIMER_RESET;
    
    // init counter
    _CP0_SET_COUNT(0);
    int TimerStart = _CP0_GET_COUNT();
    int TimerNow = _CP0_GET_COUNT();
    int Timer_LED = TimerStart;
    int Timer_DAC = TimerStart;
    int Timer_I2C2 = TimerStart;

    while(1) {
	// use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
	// remember the core timer runs at half the sysclk
         /*-------------------------------------
         -------- I2C Functionalities ----------
         --------------------------------------*/
        if (I2C_flag == 1)
        {
            unsigned char GPIO_byte = 0x00;
            // Talk to MCP23008
            if (_CP0_GET_COUNT() - Timer_I2C2 > T_I2C2_COM)
            {
                GPIO_byte = I2C2_getExpander();
                if (GPIO_byte >> 7 == 1)
                {
                    // GP7 port is high, turn on LED
                    // change OLAT-GP0 to high
                    I2C2_setExpander(0x0A, 0b00000001);
                }
                else
                {
                    // GP7 port is low, turn off LED
                    // change OLAT-GP0 to low
                    I2C2_setExpander(0x0A, 0b00000000);
                }
                
                Timer_I2C2 = _CP0_GET_COUNT();
            }
        }
         /*-------------------------------------
         -------- DAC Functionalities ----------
         --------------------------------------*/
        if (SPI_flag == 1)
        {
            // Talk to DAC MCP4922
            if (_CP0_GET_COUNT() - Timer_DAC > T_DAC_COM)
            {
                // pending
                Timer_DAC = _CP0_GET_COUNT();
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