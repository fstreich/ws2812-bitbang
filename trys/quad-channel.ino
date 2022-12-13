
#define NUM_LEDS 112*4

static uint8_t led[NUM_LEDS*3];



void bitbang() {
//   timing worked with 16MHz clock
//   X R27:R26 -> address counter of current data byte
//   Y R29:R28 -> temp data pointer
//   Z R31:R30 -> num bytes/strip
  asm volatile(

    // initialising X and Z

    "ldi r26,0 \n\t"
    "ldi R27, 1 "                  "\n\t" // high adr of led
    "ldi R30, 80"                  "\n\t" // low of NUM_LEDs 112*3
    "ldi R31, 1"                   "\n\t" // hi of NUM_LEDs 112*3
    
    "nextbyte:"                    "\n\t"

    "mov r28,r26"                  "\n\t" // mov y,x
    "mov r29,r27"                  "\n\t"
    "LDI R16,80"                   "\n\t" // low of NUM_LEDs 112*3
    "LDI R17,1"                    "\n\t" // hi of NUM_LEDs 112*3

    // load strip bytes in R18 - R21
    "LDD r18,Y+0"                  "\n\t" //
    "ADD r28, r16"                 "\n\t" // YL + r16
    "ADC r29, r17"                 "\n\t" // YH + r17
    "LDD r19,Y+0"                  "\n\t" //	
    "ADD r28, r16"                 "\n\t" // YL + r16
    "ADC r29, r17"                 "\n\t" // YH + r17
    "LDD r20,Y+0"                  "\n\t" //
    "ADD r28, r16"                 "\n\t" // YL + r16
    "ADC r29, r17"                 "\n\t" // YH + r17
    "LDD r21,Y+0"                  "\n\t" //

    "ADIW X,1"                     "\n\t" // increase x by one: address of next data
    
    "LDI R22,8"                    "\n\t" //	bitcounter: 8 bits in bytes

    "nextbit:"                     "\n\t"

    "LSL R21"                      "\n\t"
    "ROL R23"                      "\n\t"
    "LSL R20"                      "\n\t"
    "ROL R23"                      "\n\t"
    

    "LDI R24,255"                  "\n\t"  // data for port c
    "OUT 8,R24"                    "\n\t"  // all ones!

    "LSL R19"                      "\n\t"
    "ROL R23"                      "\n\t"
    "LSL R18"                      "\n\t"
    "ROL R23"                      "\n\t"
    
    "OUT 8,R23"                    "\n\t"   // data for port c
    
    "NOP"                          "\n\t"
    "NOP"                          "\n\t"
    "NOP"                          "\n\t"
    "NOP"                          "\n\t"
   
    "LDI R23,00"                   "\n\t"  // all zeros
    "OUT 8,R23"                    "\n\t"  // data for port c

    "NOP"                          "\n\t"
 
    "DEC R22"                      "\n\t"
    "BRNE nextbit"                 "\n\t" // branch if not zero 

    "SBIW Z,1"                     "\n\t"
    "BRNE nextbyte"                "\n\t" // branch if not zero 
  :::"r16","r17","r18","r19","r20","r21","r22","r23","r24",  "r26","r27","r28","r29","r30","r31");
}


void setup() {
  DDRC = 1+2+4+8; // setting PORTC0..3 to output
  int i;
  for (i=0;i<NUM_LEDS;i++) {
    led[i*3+0]=0x0; // red
    led[i*3+1]=0x0; // green
    led[i*3+2]=0x0; // blue
  }
  bitbang();
  delay(1000);
}

uint8_t counter = 0;
uint8_t counter2 = 0;
int i;
int start = 0;

void loop() {
  counter2++;
  cli();
  
  start = 0; // strip 1
  for (i=start;i<start+112;i++) {
    counter=counter ^ 0x55;
    led[i*3]=counter;
    led[i*3+1]=counter2;
    led[i*3+2]=0x00;
  }
  start = 112; // strip 2
  for (i=start;i<start+112;i++) {
    counter=counter ^ 0x55;
    led[i*3]=0x00;
    led[i*3+1]=counter;
    led[i*3+2]=counter2;
  }
  start = 224; // strip 3
  for (i=start;i<start+112;i++) {
    counter=counter ^ 0x55;
    led[i*3]=counter2;
    led[i*3+1]=0x00;
    led[i*3+2]=counter;
  }
  start = 336; // strip 4
  for (i=start;i<start+112;i++) {
    counter=counter ^ 0x55;
    led[i*3]=counter;
    led[i*3+1]=0x00;
    led[i*3+2]=counter2;
  }
  bitbang();
  delay(5);
}
