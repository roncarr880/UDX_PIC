#include "p18f2220.h"

#define SI5351 0x60
#define CLK_RX  2
#define CLK_TX  1
#define CLK_OFF 0
#define PLLA 26
#define PLLB 34

 /* switch states */
#define NOTACTIVE 0
#define ARM 1
#define DTDELAY 2
#define FINI 3
#define TAP  4
#define DTAP 5
#define LONGPRESS 6
#define DBOUNCE 50

char sw_state[3];

#pragma pic 0
char _temp;
char _temp2;
char _eedata;
char i,j,k;              /* base level looping/temp variables, watch for conflicts */
char si_adr;             /* single si5351 write register address */

char acc0, acc1, acc2, acc3;    /* 32 bit math */
char arg0, arg1, arg2, arg3;
char divq0, divq1, divq2, divq3;
char divi;

/* pre-calculated si5351 solutions in eeprom */
/* 1st 2 values are P3, last two are P2, 5th is P1 LSB */
/* solution is with 3 fractional bits, so audio tone * 8 is the offset to add in */
/* 6th value must be zero for the algorithms to work */
extern char cal[4] = { 10, 20, 15, 10 };       /* 40 was a good value for 40 meters */
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

extern char hello[] = {'H','e','l','l','o','\r','\n',0};    /* junk !!! */


char sec4;         /* 1/4 seconds counts */

char msec;           /* about 1ms counts */
char mode;
char band;
char solution[8];    /* si5351 freq solution to send to si5351 */
char rcal[4];        /* ram copy of the calibrate eeprom values, these values will be adjusted */
                     /* and later written back to eeprom, but cal values overwritten on program load */


init(){

  /* OSCCON = 0x72;              /* !!! testing 8 mhz internal clock */

  EECON1 = 0;
  
  /* init vars that need values */
  mode = 3;       /* wspr */
  band = 0;       /* 40 meters */
  sec4 = 0;
  for( i = 0; i < 4; ++i ) rcal[i] = cal[i];
  for( i = 0; i < 3; ++i ) sw_state[i] = 0;

  /* set up the uart for pickit2 uart tool debug, conflicts with B6 and B7 led control
     so disable this code when start driving the LED's */
   SPBRG = 51;           /* 9600 baud when clock is 32mhz */
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
   TRISB = 0xc7;            /* 11000111 while using the UART for debug printing. tx and ft8 led may light */

   #asm
     bcf TRISC, RC0      ; on board LED 
     bsf LATC, RC5       ; enable rx
     bcf TRISC, RC5      ; rx/tx enable pin on portC
   #endasm

  /* set up timer 0, don't call delay before interrupts are enabled */
  T0CON = 0xc4;         /* 8 bit mode, 32 prescale */
   #asm
      bsf  INTCON,TMR0IE    ;  enable timer 0 interrupts
   #endasm
   interrupts();

   delay(100);
   i2begin();
   si5351_init();
   si_get_base();
   wrt_solution();
   wrt_dividers( dividers[band] );
   clock(CLK_RX);
   
}

main(){
static char c;
static char i;

   led_control();
   if( sec4 == 255 ) save_calibrate();
   button_state();
   switch_action();

  /********
    delay( 250 );    /* delay 244 is closer to 1/4 second 
    delay( 250 );
    led_on();
    delay( 250 );
    delay( 250 );
    led_off();
   *************/

    /* send hello on the uart. sending eeprom data 
    i = 0;
    while( c = hello[i++] ) putch( c ); */

/********
    while(1){               /* generate some I2C traffic for testing 
       i2start();
       i2send( 3 );
       i2send( 0xff );
       i2stop();
       led_on();
       delay(2);
       led_off();
    }
 */

}


void switch_action(){

   /* guess at which switch is which */
   k = sw_state[0];
   if( k >= TAP ){
     switch( k ){
        case LONGPRESS:  adj_cal( 252 );  break;
     }
     sw_state[0] = FINI;
   }

   k = sw_state[1];
   if( k >= TAP ){
     switch( k ){
     }
     sw_state[1] = FINI;
   }

   k = sw_state[2];
   if( k >= TAP ){
     switch( k ){
        case LONGPRESS:  adj_cal( 4 );  break;
     }
     sw_state[2] = FINI;
   }

}


_interrupt(){
static char ms;

  ++msec;                 /* ms elapsed timer, a little slow */
  if( ++ms == 250 ){
     ++sec4;              /* 1/4 sec timer, 4 hz */
     ms = 0;
  }
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

i2start(){

   SSPCON2 = 1;               /* set start bit */
   while( SSPCON2 & 1 );      /* wait for start to clear */
   i2send( SI5351 << 1 );
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

clock( char val ){            /* control si5351 clocks, CLK_RX, CLK_TX, CLK_OFF */

   val ^= 0xff;
   i2start();
   i2send( 3 );
   i2send( val );
   i2stop();
}

si5351_init(){

   clock(CLK_OFF);

   for( si_adr = 16; si_adr < 24; ++si_adr ) si_write( 0x80 );     /* power down all outputs */

   for( si_adr = 42; si_adr <= 49; ++si_adr ) si_write( 0x00 );     /* dividers that don't change */
   for( si_adr = 42+8; si_adr <= 49+8; ++si_adr) si_write(0x00);
   si_adr = 43;   si_write( 1 );
   si_adr = 43+8; si_write( 1 );
   si_adr = 16;  si_write( 0x6c );    /* clock 0 assigned to pllb with 2ma drive */
   si_adr = 17;  si_write( 0x6c );    /* clock 1 assigned to pllb with 2ma drive */
}

si_write( char val){          /* single si5351 write register, address in global */ 

   i2start();
   i2send( si_adr );
   i2send( val );
   i2stop( );
}

wrt_dividers( char div ){      /* 128 * val - 512, same as (val-4) * 128 */
/* static char hbyte;              /* for even dividers the low byte is always zero, use div for high byte */

   div -= 4;
   div >>= 1;                          /* mult by 256 and divide by 2 == mult by 128 */
   si_adr = 45;     si_write( div );   /* write high bytes for both clocks */
   si_adr = 45+8;   si_write( div );
   si_adr = 177;                       /* reset pll's */
   si_write( 0xA0 );
   delay(5);
   si_write( 0xA0 );                   /* double reset needed ? */
}

wrt_solution( ){                  /* loads the PLL information */

   i2start();
   i2send(PLLB);
   for( k = 0; k < 8; ++k ) i2send( solution[k] );
   i2stop();
}

si_get_base(){               /* lacking double subscript array, eeprom offset is figured from s40ft8[] */

   i = band;
   i = i << 5;               /* band offset into table, mult 32 */
   i += (mode << 3);         /* mode offset into table */
   for( j = 0; j < 8; ++j ) solution[j] = s40ft8[i++];

   /* add per band calibrate value here, or manually adjust the eeprom values */
   zacc();
   acc0 = rcal[band];
   copy_acc_arg();
   dadd();  copy_acc_arg();    /* mult by 8 as we have 3 bits of fraction */
   dadd();  copy_acc_arg();
   dadd();  copy_acc_arg();
   zacc();
   acc0 = solution[7];  acc1 = solution[6];   /* adjusting P2 values */
   dadd();
   zarg();                                    /* check if P2 is larger than P3 */
   arg0 = solution[1];  arg1 = solution[0];   /* load P3 */
   while( dsub() == 0 ) ++solution[4];
   dadd();                                    /* add back on borrow */
   solution[6] = acc1;  solution[7] = acc0;
}

/* adjusting cal.  Assumes eeprom values are always low for start. Display the cal value in the
   LED's, so will see 8 counts as the 1st LED on.  Disable led_control while this is active.  */

adj_cal( char val ){         /* called on long press up/down, val will be 1,255, or 2,254 */

   rcal[band] += val;   /* no guard for underflow, eeprom values should be low for starters */
   LATB = rcal[band];
   sec4 = 25;           /* will need to count up to 255 and then LED's will be normal again */
                        /* !!! write eeprom when sec4 is 255, then set it to zero */
   si_get_base();       /* implement the change */
   wrt_solution();   
}



led_control(){   /* leds in order are wspr,js8,ft4,ft8,tx on bits RB3-RB7 */
static char last_time;

   if( sec4 == last_time ) return;
   last_time = sec4;

   k = 64;    /* build display */
   j = mode;
   while( j-- ) k >>= 1;    /* k has mode led */

   switch( band ){
    case 0:   j = 16;   break;
    case 1:   j = 32;   break;
    case 2:   j = 8+32+64;  break;
    case 3:   j = 64;   break;
   }

   switch( sec4 ){
    /*case 0: LATB = 0; break;*/
    case 1: LATB = k;  break;      /* show mode */
    case 8: LATB = 0;  break;
    case 10:
    case 14:
    case 18: LATB = j; break;
    case 12:
    case 16:
    case 20: LATB = 0; break;
    case 21: sec4 = 0; break;
   }

}


/**********************  32bit math **********************/

zacc(){       /* clear acc */

  acc0 = acc1 = acc2 = acc3 = 0;
}

zarg(){       /* clear arg */

   arg0 = arg1 = arg2 = arg3 = 0;
}

copy_acc_arg(){

   arg0 = acc0;  arg1 = acc1; arg2 = acc2; arg3 = acc3;

}


char dadd(){  /* add the accum and argument, return overflow */

   #asm
      movf    arg0,W
      addwf   acc0,F
      movf    arg1,W
      addwfc  acc1,F
      movf    arg2,W
      addwfc  acc2,F
      movf    arg3,W
      addwfc  acc3,F
   #endasm

   return STATUS & 1;

}

char dsub(){  /* sub the arg from the accum, return borrow */

   #asm
     movf    arg0,W
     subwf   acc0,F
     movf    arg1,W
     subwfb  acc1,F
     movf    arg2,W
     subwfb  acc2,F
     movf    arg3,W
     subwfb  acc3,F
   #endasm

   return (( STATUS & 1 ) ^ 1 );       /* return borrow as a true value */
}

/* a divide algorithm */
/* dividend quotient has the dividend, argument has the divisor, remainder in accumulator */
/* shift upper bits of the dividend into the remainder, test a subtraction of the divisor */
/* restore the remainder on borrow, or set a bit in quotient if no borrow */
/* divisor remains unchanged */

divide(){
 
   zacc();
   for( divi = 0; divi < 32; ++divi ){
      #asm
        bcf   STATUS,C
        rlf   divq0,F
        rlf   divq1,F
        rlf   divq2,F
        rlf   divq3,F
        rlf   acc0,F
        rlf   acc1,F
        rlf   acc2,F
        rlf   acc3,F
      #endasm
      if( dsub() ) dadd();     /* borrow, add back */
      else divq0 |= 1;         /* no borrow, so set a 1 in quotient */
   }
}


#asm
_eewrite
     ; the example from data sheet not 100% correct, leaves bit 6 in eecon1 undefined, cleared eecon1 in init() to fix.
     ; banksel EEADR
      movwf   EEADR
      movf    _eedata,W
      movwf   EEDATA

     ; banksel EECON1
      bcf     EECON1,EEPGD
      bsf     EECON1,WREN
      bcf     INTCON,GIE           ; disable interrupts
     ; btfsc   INTCON,GIE
     ; goto    $-2

      movlw   0x55      
      movwf   EECON2
      movlw   0xaa
      movwf   EECON2
      bsf     EECON1,WR

      nop
      nop
      bsf     INTCON,GIE
      nop
      nop
      btfsc   EECON1,WR
      goto    $-2
      bcf     EECON1,WREN

      return      

#endasm



save_calibrate(){              /* read current data in eeprom, save new data if different */
static char adr;

   adr = &cal[0];
   for( j = 0; j < 4; ++j ){
      if( rcal[j] != cal[j] ){
         _eedata = rcal[j];
         adr = adr;                   /* loads W reg with address */
         _eewrite();
      }
      ++adr;
   }
   sec4 = 0;
}


char read_buttons(){

   return ( PORTB ^ 7 ) & 7;
}

/* read switches, need level detect on TX switch or latch on the transmitter */
/* !!!!! 10 * debounce is too long */
void button_state(){              /* state machine running at 1ms rate */
static char sw,st;
static char press_, nopress;
static char tm;

      if( tm == msec ) return;
      tm = msec;

      sw = read_buttons(); 
      if( sw ) ++press_, nopress = 0;
      else ++nopress, press_= 0;

      if( press_ == 255 ) --press_;
      if( nopress == 255 ) --nopress;
      
      /* switch state machine */
      for( i= 0; i < 3; ++i ){
         st= sw_state[i];      /* temp the array value to save typing */

         if( st == NOTACTIVE && (sw & 0x1) && press_ >= DBOUNCE ) st= ARM;
         if( st == FINI && nopress >= DBOUNCE ) st = NOTACTIVE;   /* reset state */

         /* double tap detect */
         if( st == ARM && nopress >= DBOUNCE/2 )     st= DTDELAY;
         if( st == ARM && (sw & 0x1 ) && press_ >= 254 )  st= LONGPRESS; 
         if( st == DTDELAY && nopress >= 240 ) st= TAP;
         if( st == DTDELAY && ( sw & 0x1 ) && press_ >= DBOUNCE )   st= DTAP;
         
         sw_state[i]= st;      
         sw >>= 1;   /* next switch */
      }        
}



