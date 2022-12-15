#include "p18f2220.h"

#define SI5351 0x60

#pragma pic 0
char _temp;
char _temp2;
char _eedata;

/* pre-calculated si5351 solutions in eeprom */
/* 1st 2 values are P3, last two are P2, 5th is P1 LSB */
/* solution is with 3 fractional bits, so audio tone * 8 is the offset to add in */
/* 6th value must be zero for the algorithms to work */
extern char s40ft8[] = { 0x36, 0x7F, 0x00, 0x0D, 0xD8, 0x00, 0x0C, 0x58 };
extern char s40ft4[] = { 0x36, 0x7F, 0x00, 0x0D, 0xC9, 0x00, 0x01, 0xC9 };
extern char s40js8[] = { 0x36, 0x7F, 0x00, 0x0D, 0xDA, 0x00, 0x1C, 0x5A };
extern char s40wsp[] = { 0x36, 0x7F, 0x00, 0x0D, 0xC3, 0x00, 0x32, 0xC3 };
extern char s20ft8[] = { 0x65, 0xBB, 0x00, 0x0E, 0xE3, 0x00, 0x17, 0x2F };
extern char s20ft4[] = { 0x65, 0xBB, 0x00, 0x0E, 0xE5, 0x00, 0x07, 0x39 };
extern char s20js8[] = { 0x65, 0xBB, 0x00, 0x0E, 0xE4, 0x00, 0x2E, 0x74 };
extern char s20wsp[] = { 0x65, 0xBB, 0x00, 0x0E, 0xE9, 0x00, 0x57, 0xCD };
extern char s15ft8[] = { 0x98, 0x99, 0x00, 0x0E, 0xDB, 0x00, 0x63, 0x9D };
extern char s15ft4[] = { 0x98, 0x99, 0x00, 0x0E, 0xE9, 0x00, 0x19, 0xBF };
extern char s15js8[] = { 0x98, 0x99, 0x00, 0x0E, 0xDC, 0x00, 0x48, 0x04 };
extern char s15wsp[] = { 0x98, 0x99, 0x00, 0x0E, 0xDF, 0x00, 0x85, 0x39 };
extern char s10ft8[] = { 0xCB, 0x76, 0x00, 0x0E, 0xD7, 0x00, 0xAF, 0xE6 };
extern char s10ft4[] = { 0xCB, 0x76, 0x00, 0x0E, 0xE8, 0x00, 0x1D, 0x90 };
extern char s10js8[] = { 0xCB, 0x76, 0x00, 0x0E, 0xD8, 0x00, 0x61, 0x70 };
extern char s10wsp[] = { 0xCB, 0x76, 0x00, 0x0E, 0xDF, 0x00, 0x81, 0xB6 };


extern char dividers[4] = { 112,60,40,30 };

extern char hello[] = {'H','e','l','l','o','\r','\n',0};


/* char sec4;         /* 1/4 seconds counts, !!! not sure will need this*/

char msec;           /* about 1ms counts */
char mode;
char band;
char solution[8];    /* si5351 freq solution to send to si5351 */

/* have functions get_rx_solution, get_tx_solution which can use the rx_solution then add some values
   send_solution( reset ), will want reset for rx, not for tx, also will need to set the dividers for rx,
   perhaps can precalc that too  */


main(){
static char c;
static char i;

    delay( 250 );    /* delay 244 is closer to 1/4 second */
    delay( 250 );
    led_on();
    delay( 250 );
    delay( 250 );
    led_off();

    /* send hello on the uart. sending eeprom data */
    i = 0;
    while( c = hello[i++] ) putch( c );

    while(1){               /* generate some I2C traffic for testing */
       i2start( SI5351 );
       i2send( 3 );
       i2send( 0xff );
       i2stop();
       led_on();
       delay(2);
       led_off();
    }

}


init(){

  /* OSCCON = 0x72;              /* !!! testing 8 mhz internal clock */

  
  /* init vars that need values */
  mode = 3;       /* wspr */
  band = 0;       /* 40 meters */


  /* set up the uart for debug, conflicts with B6 and B7 led control
     so disable this code when start driving the LED's */
   SPBRG = 51;           /* 9600 baud */
   TXSTA = 0x20;         /* enable tx, slow baud rates */
   #asm
     bsf TRISC, RC7      ; rx pin input
     bcf TRISC, RC6      ; tx pin output
     bsf RCSTA,SPEN      ; enable Uart
   #endasm

     /* can't have two asm blocks in a row, compiler error, add some statements here */

  /* setup port I/O */
   LATB = 0;
   /* TRISB = 0x07;            /* 00000111 LED's and switches */
   TRISB = 0xc7;            /* 11000111 while using the UART for debug printing */

   #asm
     bcf TRISC, RC0      ; on board LED 
     bcf LATC, RC5       ; disable rx until get the si5351 clocks correctly setup
     bcf TRISC, RC5      ; rx/tx enable pin on portC
   #endasm

  /* set up timer 0, don't call delay before interrupts are enabled */
  T0CON = 0xc4;         /* 8 bit mode, 32 prescale */
   #asm
      bsf  INTCON,TMR0IE    ;  enable timer 0 interrupts
   #endasm
   interrupts();

   i2begin();
}

_interrupt(){
/*static char ms;*/

  ++msec;                 /* ms elapsed timer, a little slow */
/***
  if( ++ms == 250 ){
     ++sec4;              /* 1/4 sec timer, 4 hz, not sure this is needed
     ms = 0;
  }
***/


   #asm
      bcf  INTCON,TMR0IF
      retfie FAST
   #endasm
}


delay( char tm ){        /* 8 bit, less than 255 ms delay */

   msec = 0;
   while( msec < tm );
}


led_on(){            /* !!! inline these if only called from one place */

   #asm
     bsf LATC,RC0
   #endasm
}

led_off(){
   
   #asm
     bcf LATC,RC0
   #endasm
}


interrupts(){

   #asm
      bsf   INTCON,GIE
      bsf   INTCON,PEIE   ;  need for tmr2 ? : yes  
   #endasm
}

/* is the compiler sensitive to no extra returns on the end of the file? Seems it is */

no_interrupts(){

   #asm
     bcf   INTCON,GIE
     btfsc INTCON,GIE    ;see AN576.  What devices have this issue?
     goto $-2
   #endasm
}

putch( char c ){             /* send a character on the serial line */

   while( (TXSTA & 2) == 0 );     /* wait previous completed TRMT */
   TXREG = c;
}


/*    I2C routines */

i2begin(){

   SSPCON1 = 0;         /* clear any errors by turning off ? */
   delay(1);
   SSPCON1 = 0x28;      /*  00101000 enable and master mode */
   #asm
     bsf TRISC,RC3      ; pins are set as inputs, tristate drivers
     bsf TRISC,RC4
   #endasm
   SSPADD = 0x4e;       /* 100khz at 32 mhz clock */
}

i2start( char adr ){

   SSPCON2 = 1;               /* set start bit */
   adr <<= 1;                 /* shift address over */
   while( SSPCON2 & 1 );      /* wait for start to clear */
   i2send( adr );
}

i2send( char dat ){

   #asm
     bcf  PIR1,SSPIF          ; clear int flag
   #endasm
   SSPBUF = dat;              /* decided to wait after loading buf as ack timing not clear to me */
   while( SSPSTAT & 1 );      /* wait for buffer clear, is this useful if we also need to wait for SSPIF? */
   while( (PIR1 & 8) == 0 );  /* wait for int flag showing bit9 completed */
}

i2stop( ){

   SSPCON2 = 4;               /* set stop and wait for clear */
   while( SSPCON2 & 4 );
}



