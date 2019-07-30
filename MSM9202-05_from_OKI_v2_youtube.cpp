/*  This code is for controlling the OKI Driver VFD with the reference:
*   MSM9202-05
*   Control also the 2 pins with reference P1 and P2, which are general purpose pins.
*   This driver uses 2 bits as an ADRAM C0 & C1
* 
*   Please consider this code as a basis for evolution, this is not a clean or final code.
*   And is not careful as strong code.
*   For these reasons the code can be optimized and made robust, but that's not 
*   the purpose of this video.
*/

//set your clock speed
#define F_CPU 8000000

#include <util/delay.h>

// Standard Input/Output functions 1284
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

volatile bool update = false;    // update = true;  // see the <lib stdbool.h>

#define FALSE 0
#define TRUE (!FALSE)

typedef uint8_t byte;

char tmp[14];
//char data[14];
int dataOverflows[14];

volatile uint8_t chr1;
volatile uint8_t chr2;

#define rst9202 PINC2  // You can use a pin of Arduino to do the Reset of driver... active at LOW value, normal running at HIGH

byte data[14];
byte dataIndex;

byte inData;
byte recData;

byte edge;
byte flagTF =0;  // flag to test true ou false

unsigned int timer=0;


#define start_length 8
#define ONE_SIZE 4
#define ZERO_SIZE 1

#define BIT_IS_SET(byte, bit) (byte & (1 << bit))

#define BIT_IS_CLEAR(byte, bit) (!(byte & (1 << bit)))
// Definition of pins used to communicate with Arduino.
#define VFD_test 10 // this pin is only to debug
#define VFD_in 7
#define VFD_clk 8
#define VFD_stb 9


#define isLow() (!(PIND&~_BV(PIND2)))
  
  #define BIN(x) \
  ( ((0x##x##L & 0x00000001L) ? 0x01 : 0) \
  | ((0x##x##L & 0x00000010L) ? 0x02 : 0) \
  | ((0x##x##L & 0x00000100L) ? 0x04 : 0) \
  | ((0x##x##L & 0x00001000L) ? 0x08 : 0) \
  | ((0x##x##L & 0x00010000L) ? 0x10 : 0) \
  | ((0x##x##L & 0x00100000L) ? 0x20 : 0) \
  | ((0x##x##L & 0x01000000L) ? 0x40 : 0) \
  | ((0x##x##L & 0x10000000L) ? 0x80 : 0))

void ShowData();
void BuildArray();
void ShowArray();

/*************************************************************************/
/**************************** Start VFD Zone *****************************/
/*************************************************************************/
void MSM9202_init(void){
  unsigned char i;

  _delay_ms(300); //power_up delay

  // set GPO pins to low
  
  cmd_with_stb(0b01000000); //
  _delay_us(60);
  
  // Configure VFD display (number of grids)
  
  cmd_with_stb(0b01100100); //(0b01100110); //14 grids  (0b01100111) //15 grids  //bin(00000001) 9grids
  _delay_us(60);
  

  // set DIMM/PWM to value
  
  cmd_with_stb((0b01010000) | 7); //(0b01010000) | 7); //0 min - 7 max  )(0b01010000)
  _delay_us(60);
  
  
  // switch off extra "ADRAM"
  
  cmd_with_stb(0b00110000); //
  for(i=0;i<12;i++)
  {
    cmd_with_stb(0x20);
  }
  _delay_us(60);
  
  // test mode: light all
  
  cmd_with_stb(0b01110011); //
  _delay_us(60);
  
  // normal mode

  cmd_with_stb(0b01110000); //((0b01110000); //test off-normal mode on  (0b01110000)
  _delay_us(60);
}

void cmd_without_stb(unsigned char a){
  // send without stb
  unsigned char transmit = 7; //define our transmit pin
  unsigned char data = 170; //value to transmit, binary 10101010
  unsigned char mask = 1; //our bitmask
  
  data=a;
  //This don't send the strobe signal, to be used in burst data send
   for (mask = 00000001; mask>0; mask <<= 1) { //iterate through bit mask
     digitalWrite(VFD_clk, LOW);
     if (data & mask){ // if bitwise AND resolves to true
        digitalWrite(VFD_in, HIGH);
     }
     else{ //if bitwise and resolves to false
       digitalWrite(VFD_in, LOW);
     }
    delayMicroseconds(5);
    digitalWrite(VFD_clk, HIGH);
    delayMicroseconds(5);
   }
}

void cmd_with_stb(unsigned char a){
  // send with stb
  unsigned char transmit = 7; //define our transmit pin
  unsigned char data = 170; //value to transmit, binary 10101010
  unsigned char mask = 1; //our bitmask
  
  data=a;
  
  //This send the strobe signal
  //Note: The first byte input at in after the STB go LOW is interpreted as a command!!!
  digitalWrite(VFD_stb, LOW);
  delayMicroseconds(1);
         for (mask = 00000001; mask>0; mask <<= 1) { //iterate through bit mask
           digitalWrite(VFD_clk, LOW);
           delayMicroseconds(1);
                   if (data & mask){ // if bitwise AND resolves to true
                      digitalWrite(VFD_in, HIGH);
                   }
                   else{ //if bitwise and resolves to false
                     digitalWrite(VFD_in, LOW);
                   }
          digitalWrite(VFD_clk, HIGH);
          delayMicroseconds(1);
         }
   digitalWrite(VFD_stb, HIGH);
   delayMicroseconds(1);
}

void clear_VFD(void){
  /*
  Here I clean all registers 
  Could be done only on the number of grid
  to be more fast. The 12 * 3 bytes = 36 registers
  */
      for (int n=0; n < 11; n++){  // important be 10, if not, bright the half of wells./this on the VFD of 5 grids)
        cmd_with_stb(0b01100100); //       cmd 1 // 5 Grids & 16 Segments
        cmd_with_stb(0b01000000); //       cmd 2 //Normal operation; Set pulse as 1/16
        digitalWrite(VFD_stb, LOW);
        delayMicroseconds(1);
            cmd_without_stb((0b01100100) | n); // cmd 3 //wich define the start address (00H to 15H)
            cmd_without_stb(0b01000000); // Data to fill table of 5 grids * 16 segm = 80 bits on the table
            //
            //cmd_with_stb((0b10001000) | 7); //cmd 4
            digitalWrite(VFD_stb, HIGH);
            delayMicroseconds(100);
     }
}

void clear_VFD_ADRAM(void){
  // Only to clear the 2 additional bits of segments. See: (ADRAM DATA WRITE COMMAND ) pay attention to C0&C1
  /*
  Here I clean all registers of Aditional RAM
  Could be done only on the number of grid
  to be more fast. The 12 * 3 bytes = 36 registers
  This is the bit in front of matrix of 5*7 and the letter up of matrix
  */
      for (int n=0; n < 11; n++){  // 
          digitalWrite(VFD_stb, LOW);
          delayMicroseconds(1);
            cmd_without_stb((0b00110000) | n); // cmd 3 //wich define the start address (00H to 15H)
            cmd_without_stb(0b00000000); // Data to fill table 16 grids 
            digitalWrite(VFD_stb, HIGH);
            delayMicroseconds(10);
      }
}

void MSM9202_print(unsigned char address, unsigned char *text){
  unsigned char c;
  digitalWrite(VFD_stb, LOW);
  cmd_without_stb((0b00010000) | (address & 0x0F)); //((0b00010000)) + (address & 0x0F) );//)(0b00010000
        while ( c = (*text++) )
        {
          cmd_without_stb(c); // & 0x7F); // 0x7F to 7bits ASCII code
        }
  digitalWrite(VFD_stb, HIGH);
  _delay_us(60);
}

void MSM9202_cls(void){
unsigned char i;
  // cls DCRAM
 digitalWrite(VFD_clk, LOW);
  cmd_without_stb(0b00010000);//
      for(i=0;i<16;i++)
      {
      cmd_without_stb(' '); // Send space to clear the tube. Zone of DCRAM...
      }
 _delay_us(60); 
 digitalWrite(VFD_clk, HIGH);
  //
  // cls Data Write
  _delay_us(60);
 digitalWrite(VFD_clk, LOW);
  cmd_without_stb(0b00110000);//
      for(i=0;i<16;i++) 
      { 
      cmd_without_stb(' ');
      }
 _delay_us(60);
 digitalWrite(VFD_clk, HIGH);
 _delay_us(60);
}

 void binaryToHex(const char *inStr, char *outStr) {
   // outStr must be at least strlen(inStr)/4 + 1 bytes.
   static char hex[] = "0123456789ABCDEF";
   int len = strlen(inStr) / 4;
   int i = strlen(inStr) % 4;
   char current = 0;
     if(i) { // handle not multiple of 4
             while(i--) {
               current = (current << 1) + (*inStr - '0');
               inStr++;
             }
       *outStr = hex[current];
       ++outStr;
     }
           while(len--) {
             current = 0;
             for(i = 0; i < 4; ++i) {
               current = (current << 1) + (*inStr - '0');
               inStr++;
             }
             *outStr = hex[current];
             ++outStr;
           }
   *outStr = 0; // null byte
 }
 
 void strrevert1(char *string){
   // Inverter the contents of pointer of string
   // and let it reverted until the next call of
   // function! exp: char letter[16] = "jogos.....tarde";
   // To do a declaration of pointer:  char* ptr=letter;
   // don't forget the null char "\0", this is a 1 more char
   // presente on the string.
   
   int len = 0;
   while (string[len])
   len++;

   int down = 0;
   int up = len - 1;

         while (up > down)
         {
           char c = string[down];
           string[down++] = string[up];
           string[up--] = c;
         }
 }
 
/*****************************************************************/
/************************ END VFD Zone ***************************/
/*****************************************************************/

void ShowData(){
  MSM9202_cls();
  //strcpy(tmp, "FEA5");
  //strrevert1(data);
  //itoa(overflows, tmp, 10);
  binaryToHex(data, tmp);
  //MSM9202_cls();
  //strcpy(data, "RUNNING");
  //itoa(dataIndex, data, 10);
  MSM9202_print(1, tmp);
  _delay_ms(500);
}
/*
void BuildArray(){
  dataOverflows[dataIndex]=overflows;
  overflows=0;
  dataIndex++;
}
*/
void ShowArray(){ 
      for(int i=0; i<=dataIndex; i++){ 
        MSM9202_cls();
        itoa(dataOverflows[i], tmp, 10);
        MSM9202_print(3, tmp);
        _delay_ms(700);
      }
}

/*
comment #1: (1 << PD2) generates the binary 00000100. The operation "~" flips all the digits, i.e., the binary now is 11111011. 
Finally the &= applies the logic "AND" between DDRD and 11111011 and the result is placed again in DDRD memory. Note: What the 
operator "AND" does is for each bit in the DDRD memory, it compares with the binary number above. If the bit in DDRD is 0 and 
the bit in the binary at the same bite position is 1, then the resulting bit is 0, if the DDRD is 1 and the bit in the binary
 is 1, the resulting bit is 1, and if the bit in the DDRD is 1 or 0 and the bit in the binary is 0 then the resulting bit is 
 always 0. In summary, the command DDRD &= ~(1 << PD2) changes only the bit PD2 to zero and leave the other ones (zeros or ones)
  untouched. It seems a little bit complicated, but after you get used to it, it is the best way of changing a bit in a bite 
  without changing the other bits.
comment #2: (1 << PD2) generates the binary 00000100. Using the same logic "AND" described in comment #1, the command 
"PIND & 0000100" checks only if the PIND2 (our input pin where the push button is connected to) is set to high or not. 
All the other pins will be FALSE since the binary bits are set to 0, and since the binary bit #2 is set to 1, the IF statement
 will be TRUE only if the PD2 input is set to high or FALSE if the PD2 input is set to low.
comment #3: Following the logic explained in comment #1, this command sets output pin PINB2 in port PORTB to high voltage. 
If your LED is correct connected to this pin port with a resistor of ~300 ohms, and that resistor is connected to the ground, 
the LED should turn on.
comment #4: The LED should turn off for the same reasons explained in the previous comments.

//looking for bit zero of port D to be a one:
// if(PIND & 0x01){take_some_action();} 
  
//looking for bit 5 of port B to be a zero:
// if(~PIND & 0x20){take_action();} 

*/



void setup() {
  // put your setup code here, to run once:
  //initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
 
  /*CS12  CS11 CS10 DESCRIPTION
  0        0     0  Timer/Counter1 Disabled 
  0        0     1  No Prescaling
  0        1     0  Clock / 8
  0        1     1  Clock / 64
  1        0     0  Clock / 256
  1        0     1  Clock / 1024
  1        1     0  External clock source on T1 pin, Clock on Falling edge
  1        1     1  External clock source on T1 pin, Clock on rising edge
 */
  
//Set PORT
DDRD = 0xFF;  // IMPORTANT: from pin 0 to 7 is port D, from pin 8 to 13 is port B
PORTD=0x00;
DDRB =0xFF;
PORTB =0x00;

MSM9202_init();
}

void loop(){ 
  cmd_with_stb(0b01010111);  // set light
  delay(10);
  cmd_with_stb(0b01110011); //All on
  delay(1000);
  cmd_with_stb(0b01110001); //All off
  delay(500); 
  cmd_with_stb(0b01110011); //All on
  delay(1000);
  cmd_with_stb(0b01110001); //All off
  delay(500);
  cmd_with_stb(0b01110000); //Normal display
  
       while(1){
                // This cycle FOR is only to debug and use pin 10
                for (int i =0; i < 4; i++){
                digitalWrite(VFD_test, HIGH);
                delay(400);
                digitalWrite(VFD_test, LOW);
                delay(400);
                }
        
         // Zone of test to pins GPO
          cmd_with_stb(0b01000011); //Set P1 & P2 on
          delay(50);
          cmd_with_stb(0b01000000); //Set P1 & P2 off
          delay(50);
          cmd_with_stb(0b01000011); //Set P1 & P2 on
          delay(50);
          cmd_with_stb(0b01000000); //Set P1 & P2 off
          delay(50);
          cmd_with_stb(0b01000011); //Set P1 & P2 on
          delay(50);
          cmd_with_stb(0b01000000); //Set P1 & P2 off
          delay(50);
          
          MSM9202_cls();
          clear_VFD_ADRAM();  // Only to clear the 2 additional bits of segments. 
                              //See: (ADRAM DATA WRITE COMMAND ) pay attention to C0&C1 (pins: AD1, AD2)
          //
          strcpy(data, " Hi Folks!");   // Fill the string
          strrevert1(data);             // Do the string reverting
          MSM9202_print(1,data);        // write a grid number 1
          delay(1000);                  // Give time to see the message on VFD
          strcpy(data, "I'm driver");
          strrevert1(data);
          MSM9202_print(1,data);
          delay(1000);
          strcpy(data, "MSM9202-05");
          strrevert1(data);
          MSM9202_print(1,data);
          delay(1000);
          strcpy(data, " from OKI ");
          strrevert1(data);
          MSM9202_print(1,data);
          delay(1000);
          strcpy(data, "abcdefghij");
          strrevert1(data);
          MSM9202_print(1,data);
          delay(1000);
          strcpy(data, "klmnopqrst");
          strrevert1(data);
          MSM9202_print(1,data);
          delay(1000);
          strcpy(data, "uvxyzw....");
          strrevert1(data);
          MSM9202_print(1,data);
          delay(1000);
          //
           clear_VFD();
            //
            // this cycle for is only to test the pin 10 and help on Debug
                  for (int i=0; i < 20; i++){
                                      digitalWrite(VFD_test, HIGH);
                                      delay(30);
                                      digitalWrite(VFD_test, LOW);
                                      delay(30);      
                       }  
           }
}
