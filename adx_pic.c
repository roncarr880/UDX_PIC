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


#pragma pic 0

/* access page variables */
char _temp;
char _temp2;
char _eedata;
char i,j,k;              /* base level looping/temp variables, watch for conflicts */
char si_adr;             /* single si5351 write register address */

char acc0, acc1, acc2, acc3;    /* 32 bit math */
char arg0, arg1, arg2, arg3;
char divq0, divq1, divq2, divq3;
char divi;

extern char eecal[4] = { 0x36, 0x50, 15, 10 };        /* keep at eeprom address zero */

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

extern char hello[] = {'H','e','l','l','o','\r','\n',0};    /* junk !!! */


char sec4;         /* 1/4 seconds counts */

char msec;           /* about 1ms counts */
char mode;
char band;
char solution[8];    /* si5351 freq solution to send to si5351 */
char rcal[4];        /* ram copy of the calibrate eeprom values, these values will be adjusted */
                     /* and later written back to eeprom, but cal values overwritten on program load */
char sw_state[3];
char tx_inhibit;     /* transmit disabled on band change, tap tx to enable */
char transmitting;
char vox;
char cap1L, cap1H, cap2L, cap2H;

/* char pval;           /* !!! debug variable */


init(){

  /* OSCCON = 0x72;              /* !!! testing 8 mhz internal clock */

  EECON1 = 0;                 /* clear an undefined power up bit */
  
  /* init vars that need values */
  mode = 3;       /* wspr */
  band = 0;       /* 40 meters */
  sec4 = 0;
  for( i = 0; i < 4; ++i ) rcal[i] = eecal[i];
  for( i = 0; i < 3; ++i ) sw_state[i] = 0;
  tx_inhibit = 1;
  transmitting = 0;
  vox = 0;

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
   ADCON1 = 7;                 /* enable B port inputs */
   LATB = 0;
   /*TRISB = 0x07;          /* 00000111 LED's and switches */
   TRISB = 0xc7;          /* 11000111 while using the UART for debug printing. tx and ft8 led may light */

   #asm
     bcf TRISC, RC0      ; on board LED 
     bsf LATC, RC5       ; enable rx
     bcf TRISC, RC5      ; rx/tx enable pin on portC
     bcf TRISA, RA5      ; comparator output pin
     bsf TRISC, RC2      ; CCP1 input pin
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

  /* set up the comparator and reference */
   CMCON = 0x23;        /* 23 == invert comparitor #2, 2 independant comparators with outputs */
   CVRCON = 0xE1;       /* (1 == 0.2) (0 == 0.0) volt reference on pin vref- RA2 */
 /*  CMCON = 0x03;        /* 23 == invert comparitor #2, 2 independant comparators with outputs */
 /*  CVRCON = 0xE0;       /* (1 == 0.2) (0 == 0.0) volt reference on pin vref- RA2 */

  /* set up timer 3 */
   T3CON =  0x41;       /* timer 3 is source for capture module ( confusing, bits are split in register ) */

  /* set up capture CCP1 */
   CCP1CON = 4;         /* capture on edge no prescale, 5 == rising edge, 4 == falling edge */


  /*  8 * 8meg is 0x03D0 9000, so fits in 32 bits.  Can divide this value by the captured timer value
      to get the tone * 8.  Tone * 8 is added to the base to transmit the tone */

   
   
}

main(){
static char c;
static char i;

   led_control();
   if( sec4 == 255 ) save_calibrate();
   button_state();
   switch_action();
   vox_check();
   if( transmitting ) send_tone();


    /**** send hello on the uart. sending eeprom data 
    i = 0;
    while( c = hello[i++] ) putch( c ); ****/


}


/* data setup and modes LED's are reversed, fixed here */
void switch_action(){

   k = sw_state[0];
   if( k >= TAP ){
     switch( k ){
        case LONGPRESS:  adj_cal( 252 );  break;
        case TAP: mode_change( 1 );  break;
        case DTAP:  band_change(255);  break;
     }
     sw_state[0] = FINI;
   }

   k = sw_state[2];
   if( k >= TAP ){
     switch( k ){
        case TAP:  tx_inhibit = 0; break;
        case LONGPRESS:      break;            /* tune mode, manual transmit */
     }
     sw_state[2] = FINI;
   }

   k = sw_state[1];
   if( k >= TAP ){
     switch( k ){
        case LONGPRESS:  adj_cal( 4 );  break;
        case TAP: mode_change(255); break;
        case DTAP:  band_change( 1 ); break;
     }
     sw_state[1] = FINI;
   }

}

void mode_change( char val ){
  
   mode += val;
   if( mode > 3 ) mode = 3;
   si_get_base();
   wrt_solution();
}

void band_change( char val ){

   band += val:
   if( band > 3 ) band = 3;
   si_get_base();
   wrt_solution();
   wrt_dividers( dividers[band] );
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


void vox_check(){
static char tm;

   if( tm == msec ) return;
   tm = msec;

   if( CMCON & 128 ){
      led_on();
      vox = 11;
      if( transmitting == 0 ) ;    /* switch to tx */
      transmitting = 1;
   }
   else{
      led_off();
   }

   if( vox ){
      --vox;
      if( vox == 0 ){
         transmitting = 0;
         /* switch to rx */
      }
    }

}

void send_tone(){
static char state;


   if( (PIR1 & 4) == 0 ) return;

   #asm
     bcf PIR1,2
   #endasm

   if( state == 0 ){
      cap1L = CCPR1L;
      cap1H = CCPR1H;
      ++state;
      return;
   }
   else{
      cap2L = CCPR1L;
      cap2H = CCPR1H;
      ++state;
   }
   if( state > 1 ) state = 0;

  /***  debug prints ***/

/********
    phex( cap1H );
    phex( cap1L );
    putch( ' ' );
    phex( cap2H );
    phex( cap2L );
*********/

    #asm
      movf cap1L,W         ; get the delta count between captures
      subwf cap2L,F
      movf  cap1H,W
      subwfb cap2H,F
    #endasm

/*****   
    putch(' ');
    phex( cap2H );
    phex( cap2L );
*****/

    divq3 = 0x03;        /* 8 * 8mhz is dividend */
    divq2 = 0xd0;
    divq1 = 0x90;
    divq0 = 0;

    zarg();
    zacc();
    arg1 = cap2H;        /* divisor is delta capture counts */
    arg0 = cap2L;
    divide();

   /*** putch(' ');  ***/

    if( divq2 || divq1 > 0x5D ||  divq1 < 0x6 ){         /* out of range */
      /**** putch('B');
       putch('A');
       putch('D');
       crlf(); *****/
       return;
    }

 /*****
    phex( divq1 );
    phex( divq0 );
    crlf();
 *****/
    
    si_get_base();

  /* add in the tone offset */
   zacc();
   zarg();
   acc0 = solution[7];  acc1 = solution[6];   /* adjusting P2 values */
   arg1 = divq1;  arg0 = divq0;
   dadd();
   zarg();                                    /* check if P2 is larger than P3 */
   arg0 = solution[1];  arg1 = solution[0];   /* load P3 */
   while( dsub() == 0 ) ++solution[4];        /* sub until no borrow */
   dadd();                                    /* add back on borrow */
   solution[6] = acc1;  solution[7] = acc0;
   wrt_solution();

     #asm
       bcf PIR1,2                  ; start fresh next time
     #endasm
}

led_on(){

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



/***********    I2C routines   ************/

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
   LATB = bitrev(rcal[band]);      /* flash some LED's */
   sec4 = 25;           /* will need to count up to 255 and then LED's will be normal again */

   si_get_base();       /* implement the change */
   wrt_solution();   
}

char bitrev( char b ){     /* j variable used here, pick out the bits to display in the LED's */

   j = 0;
   #asm
      btfsc _bitrev,2
      bsf   j,6
      btfsc _bitrev,3
      bsf   j,5
      btfsc _bitrev,4
      bsf   j,4
      btfsc _bitrev,5
      bsf   j,3
   #endasm
   return j;
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

   if( tx_inhibit && sec4 < 10 ) LATB ^= 128;
   else if( transmitting ) LATB |= 128;

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
eewrite
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

   adr = 0;                             /* &cal[0], address of ee data not figured correctly */
   for( j = 0; j < 4; ++j ){
      if( rcal[j] != eecal[j] ){
         _eedata = rcal[j];
         eewrite( adr );
      }
      ++adr;
   }
   sec4 = 0;
}


char read_buttons(){

   return ( PORTB ^ 7 ) & 7;
}

/* read switches, need level detect on TX switch or latch on the transmitter */
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

/***********   debug functions  ***************/

void phex( char val ){

   k = val >> 4;
   k += '0';
   if( k > '9' ) k += 7;
   putch( k );
   k = val & 0xf;
   k += '0';
   if( k > '9' ) k += 7;
   putch( k );

}

crlf(){

   putch( '\r' );
   putch( '\n' );
}

putch( char c ){             /* send a character on the serial line */

   while( (TXSTA & 2) == 0 );     /* wait previous completed TRMT */
   TXREG = c;
}



