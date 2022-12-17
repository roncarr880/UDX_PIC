// Target Nano, but any arduino type may/will work
// see how P1 P2 and P3 change with different frequency inputs to a si5351 function
// ignore the fractional frequency code for this test, that was for WSPR and not related to this test.
// 2nd version test for direct P2 change by 1hz per freq change by 1hz, removes multiply by 128
// 3rd version has 3 bits of fraction for the freq
// 3rd version pre-calcs bytes for base frequency, may need P1,P2,P3 also or could we just manipulate the bytes for 
//   0 to 3000 offsets

#define SI5351 0x60
#define PLLA  26
#define PLLB  34

 uint32_t P1;            // PLL config register P1
 uint32_t P2;            // PLL config register P2
 uint32_t P3;            // PLL config register P3

 uint32_t bP1;           // save the calculated values for base freq
 uint32_t bP2;
 uint32_t bP3;
 
 int cal = 0;
 
//uint32_t freq = 14095600UL;
//int divider = 50;            // divider must be >= 26 for 27mhz crystal, max freq generated 34mhz
 uint32_t freq = 7038000;

//   uint32_t freq = 28923456;
//   int divider = 26;
 //  uint32_t freq = 7038600;
 //  int divider = 100;

  uint32_t freqs[] =  {  7074000,  7047500,   7078000,   7038600 };    // divider = 112
  // uint32_t freqs[] =  {21074000, 21140000,  21078000,  21094600};      // divider = 40
 // uint32_t freqs[] =  {28074000, 28180000,  28078000,  28124600};     // divider = 30
  // uint32_t freqs[] = { 14074000, 14080000,  14078000,  14095600};      // divider = 60
  //!!! make sure to change the divider when pre-calc another band
  int divider = 112;


void setup() {
 
  Serial.begin(9600);
  Serial.print( freq );   Serial.write(' ');
  si_pll_x( PLLB, freq, divider, 0.0 );
  Serial.println(P3,HEX);
  bP1 = P1;                            // save base P1 P2 P3
  bP2 = P2;
  bP3 = P3;

}

void loop() {

static int i = 0;
int32_t delta;

  //delta = random( -3000, 3001 );
  //delta += 10;
  //if( delta > 2000 ) delta = 0;
  
  Serial.print( "Base ");  Serial.println( freqs[i] );
  si_pll_x( PLLB, freqs[i], divider, 0.0 );    // 0.0 == not using the fractional freq's in this test
  bP1 = P1;                            // save base P1 P2 P3
  bP2 = P2;
  bP3 = P3;

  // P3 is the first two bytes of the solution, so don't really need to save a separate version
  Serial.println();
  Serial.print("P3 "); Serial.println(P3,HEX);

  //delta = offset[i];
  delta = 3000;
  Serial.print( "Delta "); Serial.println( delta );
  si_pll_x( PLLB, freqs[i]+delta, divider, 0.0 );    // 0.0 == not using the fractional freq's in this test
  

  
  i = i + 1;
  if( i > 3 ) i = 0;

  //Serial.print( delta );  Serial.write(' ');  // directly calculated values
  //Serial.print( P1  );   Serial.write(' ');
  //Serial.print( P2  );   Serial.write(' ');

  // calculate the same values from the saved base values using one 32 bit mult and some adds
  // int32_t val = bP2 + 128L * delta;   // add 128*delta to the base.  always changes by 128 each hz is the theory
          // int32_t val = bP2 + delta;            // C was scaled down by a further 128
  int32_t val = bP2 + 8*delta;          // C was scaled down by 16 leaving 3 fraction bits
  int32_t val1 = bP1;
  //if( delta > 0 ){                  // without cast to int32_t, this test is needed to skip negative delta
     while(val >= (int32_t)bP3){      // signed unsigned compare when val negative fails
        val -= bP3;                   // compare is unsigned unless say otherwise
        ++val1;
     }
  //}
  //if( delta < 0 ){          
     while( val < 0 ){
        val += bP3;
        --val1;
     }
  //}

   p_bytes( val1, val, bP3 );
   
   // Serial.print( bP3 ); Serial.write(' ');
   // Serial.print( val1); Serial.write(' ');          // match P1 ?
   // Serial.print( val ); Serial.write(' ');          // match P2 ?
   // if( val1 != P1 || val != P2 ) Serial.print( -100 );        // show large blip on arduino plotter on errors
   // else Serial.print( 0 );
    Serial.println();
    
  delay( 20000 );

}

void  si_pll_x(unsigned char pll, uint64_t freq, int out_divider, float fract ){
 uint64_t a,b,c;
 uint64_t bc128;             // floor 128 * b/c term of equations
 uint64_t pll_freq;
 uint64_t clock_freq = (uint64_t)(25001740L + cal);

 //uint32_t P1;            // PLL config register P1
 //uint32_t P2;            // PLL config register P2
 //uint32_t P3;            // PLL config register P3
 uint64_t r;


//  for fractional part, just add fraction * out_divider to the pll_freq
//  as a full increment of one will add full value of out_divider to pll_freq
//  only works here for positive frequency changes.

   //c = 1000000;     // max 1048575
   c = clock_freq / out_divider;               // each b is 1 hz
         //c /= 128;                                 // each P2 is 1 hz, each b is 128, so calc can be +-128 hz off freq
   c /= 16;                                    // each P2 is 1/8 hz ?
   pll_freq = freq * (uint64_t)out_divider;
   //fract = fract * (float)out_divider;
   //if( fract > 0.0 ) pll_freq += (uint64_t)fract;
   a = pll_freq / (clock_freq);
   r = pll_freq - a * (clock_freq);
   b = ( c * r ) / (clock_freq);
   bc128 =  (128 * r)/ (clock_freq);
   P1 = 128 * a + bc128 - 512;
   P2 = 128 * b - c * bc128;
   if( P2 > c ){     // large unsigned number, would be a negative signed number, unsigned add to fix
      P2 += c;       // fixed point truncation issue?
      --P1;
   }
   P3 = c;

   i2cd(SI5351, pll + 0, (P3 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 1, (P3 & 0x000000FF));
   i2cd(SI5351, pll + 2, (P1 & 0x00030000) >> 16);
   i2cd(SI5351, pll + 3, (P1 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 4, (P1 & 0x000000FF));
   i2cd(SI5351, pll + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
   i2cd(SI5351, pll + 6, (P2 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 7, (P2 & 0x000000FF));
   
 //  i2cd( SI5351, 177, 0xAC );         // PLLA PLLB soft reset  
}

void p_bytes( uint32_t P1, uint32_t P2, uint32_t P3 ){

  Serial.print( (P3 & 0x0000FF00) >> 8 , HEX);  Serial.write(' ');
    Serial.print( (P3 & 0x000000FF) , HEX);  Serial.write(' ');
    Serial.print( (P1 & 0x00030000) >> 16 , HEX);  Serial.write(' '); 
    Serial.print( (P1 & 0x0000FF00) >> 8 , HEX);  Serial.write(' ');
    Serial.print( (P1 & 0x000000FF) , HEX);  Serial.write(' ');
    Serial.print( ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16) , HEX);  Serial.write(' ');
    Serial.print( (P2 & 0x0000FF00) >> 8 , HEX );  Serial.write(' ');
    Serial.print( (P2 & 0x000000FF) , HEX);  Serial.write(' ');
    Serial.println();
}

void i2cd( uint8_t adr, uint8_t reg, uint8_t data ){

    Serial.print( adr,HEX );  Serial.write(' ');
    Serial.print( reg,HEX );  Serial.write(' ');
    Serial.print( data,HEX );  Serial.write(' ');
    Serial.println();
}
