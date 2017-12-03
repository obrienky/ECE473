//  lab4_skel.c
//  William Davis
//  November 5th, 2017

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>

#include "hd44780.h"

//Mode for button Multiplication
#define Mode0  0b110 //8
#define Mode1  0b000 //0
#define Mode2  0b010 //2
#define Mode4  0b100 //4

#define Mode8 0b1000 //8

//volatile uint8_t alarmArm = 0;

// Holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5]; 

//Seven Segment Codes 0,1,2,3,4,5,6,7,8,9 and logic "0" is On
uint8_t dec_to_7seg[12] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x98, 0x01};

// Select digit array
uint8_t digitSelect[8];

// Holds value of buttons from last check
volatile uint8_t buttonState;

// Holds state of encoders
volatile uint8_t encoderState;

// Number displayed to 7seg
volatile uint16_t time = 0;

// Number displayed to bargraph
volatile uint8_t barNum = 0;

// Initialize
uint8_t checkDirection(uint8_t, uint8_t);
//void increment();
//void decrement();

void incR();
void incL();
void decR();
void decL();

uint16_t clockMSec = 0;
uint8_t clockSec = 0;

//uint8_t clockSecOld = 0;

//uint8_t colonMode = 0;
uint8_t colon = 0;

uint8_t button1 = 0;
uint8_t button2 = 0;
uint8_t button3 = 0;
uint8_t button4 = 0;

//******************************************************************************
//    -- chk_buttons --                                     
//  Checks the state of the button number passed to it. It shifts in ones till   
//  the button is pushed. Function returns a 1 only once per debounced button    
//  push so a debounce and toggle function can be implemented at the same time.  
//  Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//  Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//  external loop delay times 12.
//******************************************************************************
uint8_t chk_button(uint8_t button) {
  // Static array is initialied once at compile time
  static uint16_t State[8] = {0};   
  
  State[button] = (State[button]<<1) | !bit_is_clear(PINA, button) | 0xE000;
  if (State[button] == 0xFF00) return TRUE;
  return FALSE;
} //chk_button


//******************************************************************************
//    -- segment_sum --                                   
//  takes a 16-bit binary input value and places the appropriate equivalent 4
//  digit BCD segment code in the array segment_data for display.                       
//  Array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
//*****************************************************************************
void segsum(uint16_t sum)
{
  //determine how many digits there are 
  uint8_t i=0;    
  //uint8_t zero = TRUE;
  
  //break up decimal sum into 4 digit-segments
  segment_data[0] = sum % 10;        
  segment_data[1] = sum/10 % 10;        
  segment_data[2] = 0b00011011;//0x27;//0x00;         // keep colon off; dig10 is mapped to 0xFF        
  segment_data[3] = sum/100 % 10;        
  segment_data[4] = sum/1000 % 10;        

  // Covert dec to BCD, ignoring colon and blanking leading zeros
  //blank out leading zero digits 
  for (i=4; i > 0; --i)
  {
      if(i == 2)
      {
        if(colon == 1)
        {
         segment_data[i] = 0b00011011; //on
         }
         else if(colon == 0)
         {
           segment_data[i] = 0xFF; //off
         }
      }
//    if (zero && (segment_data[i]==0))
      //if(clockSecOld != clockSec && i == 2)
//      segment_data[i] = 0xFF; //off
//    else
//    {
//      if (i!=2) zero = FALSE;
      segment_data[i] = dec_to_7seg[segment_data[i]];
//    }//if
  }//for

  segment_data[0] = dec_to_7seg[segment_data[i]];

  return;
}//segment_sum


//******************************************************************************
//    Checks Which Buttons are Pressed
//******************************************************************************
void get_button() {

  //make PORTA an input port with pullups 
  DDRA = 0x00;    // 0 is input, 1 is output
  PORTA = 0xFF;   // 0 is float, 1 is pull-up

  //enable tristate buffer for pushbutton switches
  PORTB &= !(0x7 << PB4);
  PORTB |= (0x7 << PB4);

  int i;
  //int button_Num;
  //now check each button and increment the time as needed
   
  for (i=0; i<8; i++)
  {
    if (chk_button(i))
    {    
      buttonState ^= 1<<i;
      //switch(chk_button(i)){
      switch(buttonState){ //ERRORS HERE FOR LCD
      case 0b00:
        //Activate Alarm
        button1++;
        if(button1 % 2 == 1)
        {
          clear_display();
          string2lcd("ALARM ON");
        }else{
          //clear the display!
          clear_display();
        }
        break;
      
      case 0b01:
        //Deactivate Alarm
        button2++;
        if(button2 % 2 == 1)
        {
          clear_display();
          string2lcd("Snooze");
        }else{
          //clear the display!
          clear_display();
        }

       }
     }
   }

/**
    if (chk_button(i))
      buttonState ^= 1<<i;
  }//for

     

  if(chk_button(0))
  {
    //Activate Alarm
    clear_display();
    string2lcd("ALARM ON");
  }  
  if(chk_button(1))
  {
    //Activate Alarm
    clear_display();
    string2lcd("ALARM OFF");
  }     
    


}else{
      clear_display();
      string2lcd("Snooze");
    }
  }
**/

  //disable tristate buffer for pushbutton switches
  PORTB &= !(0x7 << PB4);

  // Reset A as output
  DDRA = 0xFF;
}


//******************************************************************************
//    Get SPI data for encoder
//******************************************************************************
void get_enc() {

  // Toggle Encoder Shift/Load
  PORTE &= ~(1<<PE6);
  PORTE |= (1<<PE6);

  // Send data to bargraph
  SPDR = barNum;

  // Wait for 8 clock cycles
  while(bit_is_clear(SPSR, SPIF)) {}

  // Save the most recent serial reading into global variable
  encoderState = SPDR;

  // Toggle Bargraph Register Clock
  PORTB |= (1<<PB0);
  PORTB &= ~(1<<PB0);
}


//******************************************************************************
//    -- Performs Logic to Test Direction of Encoder Movement --
//******************************************************************************
void exe_enc(){
  
  //Initialize
  uint8_t curr=0;
  uint8_t prev=0;
  
  volatile static uint8_t encR_0b0001 = 0; //R1
  volatile static uint8_t encR_0b0010 = 0; //R2

  volatile static uint8_t encL_0b0001 = 0; //L1
  volatile static uint8_t encL_0b0010 = 0; //L2
  
  volatile static uint8_t encStatusReg=0;

  // Encoder states
  curr = (encoderState & 0x0F);
  prev = (encStatusReg & 0x0F);

  // Right Encoder Changed State
  if ((curr & 0b0011) != (prev & 0b0011)) {

    // Shift registers to keep track of turning speed
    switch(checkDirection((curr & 0b0011),(prev & 0b0011))) {
      case 0b01:
        encR_0b0001 = (encR_0b0001<<1)|1;
        encR_0b0010 =  encR_0b0010>>1;
        break;     
      case 0b10:
        encR_0b0001 =  encR_0b0001>>1;
        encR_0b0010 = (encR_0b0010<<1)|1;
        break;
      default:
        encR_0b0001 = encR_0b0001>>1;
        encR_0b0010 = encR_0b0010>>1;
        break;
    }

    // When at notch, reset turning speed
    if ((curr & 0b0011) == 0b0011) {
      encR_0b0001 = 0;
      encR_0b0010 = 0;
    }

    // Check right encoder
    if (encStatusReg & (1<<0b0100)) {

      if (encR_0b0001 >= 0b11) {

        // Extra increments to compensate for missed bits
        if (encR_0b0001 >= 0b11111) {
          if (encR_0b0001 >= 0b111111) {
            incR();
          }
          incR();
        }

        incR();
        encStatusReg &= ~(1<<0b0100);
        encR_0b0001 = 0;
        encR_0b0010 = 0;

      } else if (encR_0b0010 >= 0b11) {

        // Extra decrements to compensate for missed bits
        if (encR_0b0010 >= 0b11111) {
          if (encR_0b0010 >= 0b111111) {
            decR();
          }
          decR();
        }

        decR();
        encStatusReg &= ~(1<<0b0100);
        encR_0b0001 = 0;
        encR_0b0010 = 0;
      }
    }

    // When at halfway point, enable state change
    //  This prevents a floating state next to notch triggering an event
    if ((curr & 0b0011) == 0x00) {
      encStatusReg |= (1<<0b0100);
    }

    encStatusReg &= ~0b0011;
    encStatusReg |= (encoderState & 0b0011);
  }


  // Left Encoder Changed State
  if ((curr & 0b1100) != (prev & 0b1100)) {

    // Shift registers to keep track of turning speed
    switch(checkDirection(((curr & 0b1100)>>2),((prev & 0b1100)>>2))) {
      case 0b01:
        encL_0b0001 = (encL_0b0001<<1)|1;
        encL_0b0010 =  encL_0b0010>>1;
        break;     
      case 0b10:
        encL_0b0001 =  encL_0b0001>>1;
        encL_0b0010 = (encL_0b0010<<1)|1;
        break;
      default:
        encL_0b0001 = encL_0b0001>>1;
        encL_0b0010 = encL_0b0010>>1;
        break;
    }

    // When at notch, reset turning speed
    if ((curr & 0b1100) == 0b1100) {
      encL_0b0001 = 0;
      encL_0b0010 = 0;
    }

    // Check right encoder
    if (encStatusReg & (1<<0b0101)) {

      if (encL_0b0001 >= 0b11) {

        // Extra increments to compensate for missed bits
        if (encL_0b0001 >= 0b11111) {
          if (encL_0b0001 >= 0b111111) {
            incL();
          }
          incL();
        }

        incL();
        encStatusReg &= ~(1<<0b0101);
        encL_0b0001 = 0;
        encL_0b0010 = 0;

      } else if (encL_0b0010 >= 0b11) {

        // Extra decrements to compensate for missed bits
        if (encL_0b0010 >= 0b11111) {
          if (encL_0b0010 >= 0b111111) {
            decL();
          }
          decL();
        }

        decL();
        encStatusReg &= ~(1<<0b0101);
        encL_0b0001 = 0;
        encL_0b0010 = 0;
      }
    }

    // When at halfway point, enable state change
    //  This prevents a floating state next to notch triggering an event
    if ((curr & 0b1100) == 0x00) {
      encStatusReg |= (1<<0b0101);
    }

    encStatusReg &= ~0b1100;
    encStatusReg |= (encoderState & 0b1100);
  }
}


//******************************************************************************
//    -- Encoder Checker
//    Return Value
//    bit1    bit0
//    0       1   Clockwise
//    1       0   Counter Clockwise
//******************************************************************************
uint8_t checkDirection(uint8_t curr, uint8_t prev) {

  curr &= 0b11; 	
  prev &= 0b11; 	
  
  switch (curr) {
    case 0b01: 		//1
      switch (prev){
        case 0b11:	
          return 0b0001;
        case 0b00:	
          return 0b0010;
      }
      break;
    case 0b00:		//0
      switch (prev){
        case 0b01:
          return 0b0001;
        case 0b10:
          return 0b0010;
      }
      break;
    case 0b10:		//2
      switch (prev){
        case 0b00:
          return 0b0001;
        case 0b11:
          return 0b0010;
      }
      break;
    case 0b11:		//3
      switch (prev){
        case 0b10:
          return 0b0001;
        case 0b01:
          return 0b0010;
      }
      break;

  }//switch

  return 0;
}

//******************************************************************************
//    -- Timer 0 Compare Interrupt --
//******************************************************************************
ISR(TIMER0_COMP_vect){
  
  //Get data from SPI
  get_enc();

  //Process data
  exe_enc();
  
  //Button to Bargraph
  get_button();



  



}//ISR TIM0_COMP_vect


//******************************************************************************
//    -- Serial Peripheral Interface Initialization --
//******************************************************************************
void init_all() {
  
  //*************************************************
  //Seven Seg Initialization
  //*************************************************
  // select pins for DEMUX in array form
  digitSelect[0] = (0x0 << PB4);
  digitSelect[1] = (0x1 << PB4);
  digitSelect[2] = (0x2 << PB4);
  //digitSelect[2] = (0x7 << PB4);
  digitSelect[3] = (0x3 << PB4);
  digitSelect[4] = (0x4 << PB4);

  //COLON DATA, dont use here
  //PORTB = 0x21;
  //PORTA = 0x04;



  // 0 is input, 1 is output
  DDRB = (1<<PB4)|(1<<PB5)|(1<<PB6);
  //
  DDRF = (1<<PF3);
  PORTF &= ~(1<<PF3);

  //*************************************************
  //Timer Initialization
  //*************************************************
  // Timer counter 0 setup, running off i/o clock

  // Asynchronous Status Register, pg107
  //    Run off of external clock
  ASSR  |= (1<<AS0);

  // Timer/Counter Interrupt Mask, pg109
  //    enable compare interrupt
  TIMSK |= (1<<OCIE0);

  // Timer/Counter Control Register
  //    CTC mode, no prescale
  TCCR0 = ((1<<WGM01)|(0<<WGM00)|(0<<COM01)|(0<<COM00)|(0<<CS02)|(0<<CS01)|(1<<CS00));
  

  // Output Compare Register
  //    Set button&encoder check time with this
  OCR0 = 0x1F;

  //*************************************************
  //SPI Initialization
  //*************************************************
  // Direction Registers
  DDRB |= (1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB7);
  DDRE |= (1<<PE6);

  // SPI Control Register
  SPCR |= (1<<SPE) | (1<<MSTR) | (0<<SPR1)|(1<<SPR0);

  // SPI Status Register
  SPSR |= (1<<SPI2X);

  // SPI Data Register
  PORTB &= ~(1<<PB7);
}


//******************************************************************************
//    -- Conditionally Increment Based on State
//******************************************************************************
void incR() {
  
  time++;		//increment

  if (time % 100 >= 60) {  //if minutes >= 60
    time -= 60;
    time += 100;
    
    if(time >= 2500 || time < 0)
    {
      time = 0;
    }
    //if(time > 2459){
    //  time = 0;
    }
}



//******************************************************************************
//    -- Conditionally Decrement Based on State
//******************************************************************************
void decR() {
  time--; 			//decrement
  //int tempTime = time / 100; 	//get hours
  
  //regular
  if (time % 100 <= 0) {	//if minutes == 0
    time += 59;
    time -= 100;
    if(time < 0 || time > 2459)
    {
      time = 2459;
    }
  }
/**
  else if(time > 2459)
  {
     time = 2459;
   }

  //error correcting
  else if(time % 100 <= 99 && time % 100 >= 60)
  {
     time -= 40;
  }
  else if(time < 0)
  {
    time = 2459;
  }
  else
  {
    time--;
  }**/
}


//******************************************************************************
//    -- Conditionally Increment Based on State
//******************************************************************************
void incL() {
  int tempMin;
  tempMin = time % 100;

  time += 100;

  if (time >= 2500) {
    //time -= 2500;
    time = 0;
    time += tempMin;
  }
}



//******************************************************************************
//    -- Conditionally Decrement Based on State
//******************************************************************************
void decL() {
  int tempMin;
  tempMin = time % 100;

  time -= 100;

  //error correcting
  //if(time / 100 <= 60 && time / 100 >= 25)
  if(time >= 2459)
  {
     time = 0;
     time += tempMin;
  }
  //error correcting backup
  if(time <= 59 && time >= 0)
  {
     time = 2400;
     time += tempMin;
  }

  if (time <= 0) {
    time = 2400;
    time += tempMin;
  }
}



//-----------------------------------------------------------------------------
//                          strobe_lcd
//Strobes the "E" pin on the LCD module. How this is done depends on the interface
//style. The 4-bit mode is presently kludged with nop statements to make a suitable
//pulse width for a 4 Mhz clock.
//TODO: make number of nops executed dependent on F_CPU, not hardcoded
//
void strobe_lcd(void){
#if SPI_MODE==1
 PORTF |=  0x08; PORTF &= ~0x08; //toggle port F bit 3, LCD strobe trigger
#else
//4-bit mode below
 LCD_PORT |= (1<<LCD_STROBE_BIT);           //set strobe bit
 asm("nop"); asm("nop"); asm("nop"); asm("nop"); //4 cycle wait
 LCD_PORT &= ~(1<<LCD_STROBE_BIT);          //clear strobe bit
 asm("nop"); asm("nop"); asm("nop"); asm("nop"); //4 cycle wait
#endif
}

//-----------------------------------------------------------------------------
//                               send_lcd
//
// Sends a command or character data to the lcd. First argument of 0x00 indicates
// a command transfer while 0x01 indicates data transfer.  The next byte is the
// command or character byte.
//
// This is a low-level function usually called by the other functions but may
// be called directly to provide more control. Most commonly controlled
// commands require 37us to complete. Thus, this routine should be called no more
// often than every 37us.
//
// Commnads that require more time have delays built in for them.
//
void send_lcd(uint8_t cmd_or_char, uint8_t byte){

#if SPI_MODE==1
  SPDR = (cmd_or_char)? 0x01 : 0x00;  //send the proper value for intent
  while (bit_is_clear(SPSR,SPIF)){}   //wait till byte is sent out
  SPDR = byte;                        //send payload
  while (bit_is_clear(SPSR,SPIF)){}   //wait till byte is sent out
  strobe_lcd();                       //strobe the LCD enable pin
#else //4-bit mode
  if(cmd_or_char==0x01){LCD_PORT |=  (1<<LCD_CMD_DATA_BIT);}
  else                 {LCD_PORT &= ~(1<<LCD_CMD_DATA_BIT);}
  uint8_t temp = LCD_PORT & 0x0F;             //perserve lower nibble, clear top nibble
  LCD_PORT   = temp | (byte & 0xF0);  //output upper nibble first
  strobe_lcd();                       //send to LCD
  LCD_PORT   = temp | (byte << 4);    //output lower nibble second
  strobe_lcd();                       //send to LCD
#endif
}


//-----------------------------------------------------------------------------
//                          clear_display
//
//Clears entire display and sets DDRAM address 0 in address counter. Requires
//1.8ms for execution. Use only if you can withstand the big delay.
//
void clear_display(void){
  send_lcd(CMD_BYTE, CLEAR_DISPLAY);
  _delay_us(1800);   //1.8ms wait for LCD execution
}

//----------------------------------------------------------------------------
//                            lcd_init
//
//Initalize the LCD
//
void lcd_init(void){
  _delay_ms(16);      //power up delay
#if SPI_MODE==1       //assumption is that the SPI port is intialized
  //TODO: kludge alert! setting of DDRF should not be here, but is probably harmless.
  DDRF=0x08;          //port F bit 3 is enable for LCD in SPI mode
  send_lcd(CMD_BYTE, 0x30); _delay_ms(7); //send cmd sequence 3 times
  send_lcd(CMD_BYTE, 0x30); _delay_ms(7);
  send_lcd(CMD_BYTE, 0x30); _delay_ms(7);
  send_lcd(CMD_BYTE, 0x38); _delay_ms(5);
  send_lcd(CMD_BYTE, 0x08); _delay_ms(5);
  send_lcd(CMD_BYTE, 0x01); _delay_ms(5);
  send_lcd(CMD_BYTE, 0x06); _delay_ms(5);
  send_lcd(CMD_BYTE, 0x0C + (CURSOR_VISIBLE<<1) + CURSOR_BLINK); _delay_ms(5);
#else //4-bit mode
  LCD_PORT_DDR = 0xF0                    | //initalize data pins
                 ((1<<LCD_CMD_DATA_BIT)  | //initalize control pins
                  (1<<LCD_STROBE_BIT  )  |
                  (1<<LCD_RDWR_BIT)    );
  //do first four writes in 8-bit mode assuming reset by instruction
  //command and write are asserted as they are initalized to zero
  LCD_PORT = 0x30; strobe_lcd(); _delay_ms(8);  //function set,   write lcd, delay > 4.1ms
  LCD_PORT = 0x30; strobe_lcd(); _delay_us(200);//function set,   write lcd, delay > 100us
  LCD_PORT = 0x30; strobe_lcd(); _delay_us(80); //function set,   write lcd, delay > 37us
  LCD_PORT = 0x20; strobe_lcd(); _delay_us(80); //set 4-bit mode, write lcd, delay > 37us
  //continue initalizing the LCD, but in 4-bit mode
  send_lcd(CMD_BYTE, 0x28); _delay_ms(7); //function set: 4-bit, 2 lines, 5x8 font
  //send_lcd(CMD_BYTE, 0x08, 5000);
  send_lcd(CMD_BYTE, 0x01); _delay_ms(7)  //clear display
  send_lcd(CMD_BYTE, 0x06);  _delay_ms(5) //cursor moves to right, don't shift display
  send_lcd(CMD_BYTE, 0x0C | (CURSOR_VISIBLE<<1) | CURSOR_BLINK); _delay_ms(5);
#endif
}

//----------------------------------------------------------------------------
//                            string2lcd
//                            
//Send a ascii string to the LCD.
void string2lcd(char *lcd_str){ 
  uint8_t i;
  for (i=0; i<=(strlen(lcd_str)-1); i++){send_lcd(CHAR_BYTE, lcd_str[i]);
  _delay_us(40);  //execution takes 37us per character
  }                  
} 









//******************************************************************************
//                                  Main
//******************************************************************************
int main(void) {

  uint8_t i = 0;

  //Initialize Everything
  init_all();
  sei();
  
  lcd_init();
  string2lcd("Initialize");
  
  while(1)
  {
    //clear_display();
    //string2lcd("IT WORKS");

    //Get code
    segsum(time);
    
    //Turn on bargraph
    barNum = buttonState;

    //make PORTA an output
    DDRA = 0xFF;

    //bound a counter (0-4) to keep track of digit to display 
    for (i=0; i<5; i++)
    {
      // Clear digit select
      PORTB &= !(0x7 << PB4);

      //update digit to display
      PORTB |= digitSelect[i];

      //send 7 segment code to LED segments
      PORTA = segment_data[i];

      _delay_us(100);
    }//for
  
  clockMSec++;
  if(clockMSec >= 1900)
  {
    clockSec += 1;
    clockMSec -= 1900;

    if(clockSec >= 60)
    {
      time += 1;
      clockSec -= 60;
      if(time % 100 >= 60)
      {
        time -= 60;
        time += 100;
      }
    }
  }

   //blink colon every sec
   if (clockMSec % 800 == 0) 
   {
      colon = TRUE;
   }
   if (clockMSec % 1900 == 0) 
   {
     colon = FALSE;
   }

  }//while

  return 0;

}//main
