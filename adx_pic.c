#include "p18f2220.h"

#define SI5351 0x60

#pragma pic 0
char _temp;
char _temp2;
char _eedata;

extern char hello[] = {'H','e','l','l','o','\r','\n',0};


/* char sec4;         /* 1/4 seconds counts, !!! not sure will need this*/

char msec;         /* about 1ms counts */


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
       i2start( SI5351 );   /* will errors hang everything ? */
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

  LATC = 0;                      /* just in case the I2C pins get set to outputs, have them drive low */
  
  /* init vars that need values */

  /* set up the uart for debug, conflicts with B6 and B7 led control so disable this code when start driving the LED's */
   SPBRG = 51;           /* 9600 baud */
   TXSTA = 0x20;         /* enable tx, slow baud rates */
   #asm
     bsf TRISC, RC7      ; rx pin input
     bcf TRISC, RC6      ; tx pin output
     bsf RCSTA,SPEN      ; enable Uart
   #endasm

   /* msec = 0;             /* can't have two asm blocks in a row, compiler error, add dummy statement */

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

   SSPCON1 = 0;         /* clear any errors by turning off */
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



