#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro

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

#define CS LATAbits.LATB15

void LED_blink_init()
{
    // setup initial pins configuration for LED
    TRISAbits.TRISA4 = 0; //A4 output
    TRISBbits.TRISB4 = 1; //B4 input
    LATAbits.LATA4 = 1; //A4
}

void SPI1_init()
{
    // SPI1 initialization
    
    //set up the chip select pin as an output - RPB15
    TRISBbits.TRISB15 = 0;
    //chip selection pin is used for indicating the begin and end of a command
    CS = 1; //block communication
    
    //setup SPI1
    // output pins
    RPB15Rbits.RPB15R = 0b0011; // set RPB15 as SS1
    RPB11Rbits.RPB11R = 0b0011; // set RPB11 as SDO1
    // input pins
    SDI1Rbits.SDI1R = 0b0100; // set RPB8 as SDI1
    
    SPI1CON = 0;              // turn off the spi module and reset it
    SPI1BUF;                  // clear the rx buffer by reading from it
    SPI1BRG = 0x3;            // baud rate to 10 MHz [SPI1BRG = (80000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0;  // clear the overflow bit
    SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1;    // master operation
    SPI1CONbits.ON = 1;       // turn on spi 1
    
    CS = 0; // detect status of slaves
    // do operations here
    CS = 1; // finish initialization
}

// send a byte via spi and return the response
unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

void setVoltage(char channel, int voltage)
{
    // set voltage for DAC of targeted channel
    unsigned short bitmask = 0x0; // initial mask
    // 15th bit for channel selection
    if (channel == 'A')
    {
        bitmask |= 0x0 << 15;
    }
    else if (channel == 'B')
    {
        bitmask |= 0x1 << 15;
    }
    // 14th bit for buffer switch
    bitmask |= 0x1 << 14;
    // 13th bit for output gain selection
    bitmask |= 0x1 << 13;
    // 12th bit for shutdown control
    bitmask |= 0x1 << 12;
    // 11th to 0th are input data bits
    bitmask |= (voltage & (0xfff));
    
    CS = 0; // select device
    // send in pieces of 8 bits
    spi_io(bitmask>>8);
    spi_io(bitmask&0xff);
    CS = 1; // end communication
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

    // do your TRIS and LAT commands here
    LED_blink_init();
    SPI1_init();

    __builtin_enable_interrupts();
    
    // init counter
    _CP0_SET_COUNT(0);
    
    // frequency config
    int SysCLK_freq = 48e6; // system clock frequency
    int LoopCLK_freq = SysCLK_freq/2; // main loop frequency
    // LED
    int LED_blink_freq = 1e3; // LED blink frequency
    // DAC
    int DAC_A_freq = 10; // channel A frequency
    int DAC_B_freq = 5; // channel B frequency
    
    while(1) {
	// use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
	// remember the core timer runs at half the sysclk
        
        /*-------------------------------------
         ------- LED Functionalities ----------
         --------------------------------------*/
        if(_CP0_GET_COUNT() % (LoopCLK_freq/LED_blink_freq/2) )
        {
            LATAbits.LATA4 = !LATAbits.LATA4;
            _CP0_SET_COUNT(0);
        }
        while (!PORTBbits.RB4){LATAbits.LATA4 = 0;}
    }
}