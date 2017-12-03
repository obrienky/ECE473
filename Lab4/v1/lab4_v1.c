//lab3_code
//Trevor Love
//10.25.2015
//

#define F_CPU 16000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define TRUE 1
#define FALSE 0

volatile uint8_t gcEncLeftPast = 0;
volatile uint8_t gcEncRightPast = 0;

volatile int16_t gsDisplayData = 0;
volatile uint8_t gcStatus = 0;
volatile uint8_t gcDataMult = 1;

uint8_t gbLeftDone = FALSE;
uint8_t gbLeftInc = FALSE;
uint8_t gbLeftDec = FALSE;
uint8_t gbRightDone = FALSE;
uint8_t gbRightInc = FALSE;
uint8_t gbRightDec = FALSE;

uint8_t gcDigit = 0;

uint8_t gcDigitCount = 0;
uint8_t gcaSegData[5];
uint8_t gcaLEDencode[12] = { 0b11000000, 0b11111001, 0b10100100,  0b10110000, 0b10011001, 
0b10010010, 0b10000010, 0b11111000, 0b10000000, 0b10010000, 0xFF, 0x00};
	

void port_init(void)
{
 DDRB = 0xFF;
 //DDRE = (1<<PIN1); //PORTE all outputs
 DDRE = 0xFF;
 DDRC = 0xF0;
}

/***********************************************************************/
//                            spi_init                               
/***********************************************************************/
void spi_init(void){
  SPCR  |=   (1<<SPE)|(1<<MSTR)|(1<<SPR0); //set up SPI mode
  SPSR  |=   (1<<SPI2X);             // double speed operation
 }//spi_init

/***********************************************************************/
//                              tcnt0_init                             
//
void tcnt0_init(void){
  ASSR   |=  1<<AS0;  //ext osc TOSC
  TIMSK  |=  1<<OCIE0;//enable timer/counter0 overflow interrupt
  TCCR0  |=  1<<WGM01 | 1<<CS00; //CTC mode, no prescale
  OCR0 = 32;	//timer compare register value
}
/***********************************************************************/



/***********************************************************************/
//                            readEncoder
/***********************************************************************/
uint8_t readEncoder(void){
  uint8_t data;
  PORTE &= ~(1<<PE6);                    //load in || inputs
  PORTE |= (1<<PE6);                     //switch to shift
  SPDR = 0xFF;                           //start clock
  while(!(SPSR & (1<<SPIF))){};          //wait for data
  data = SPDR;                           //read encoder
  return(data);
  //return(SPDR)
}//readEncoder


/***********************************************************************/
//                           writeBarGraph
/***********************************************************************/
void writeBarGraph(uint8_t data){
  SPDR = data;                 //write to bar graph
  while(!(SPSR & (1<<SPIF))){};     //wait for data
  PORTB |=   0x01;                  //strobe output data reg in HC165 - rising edge
  PORTB &= ~(0x01);                 //falling edge
}//writeBarGraph


//***********************************************************************
//                            debounceButton                              
//**********************************************************************
uint8_t debounceButton(uint8_t cButton){
    static uint16_t state[8] = {0};
    state[cButton] = (state[cButton]<<1)|(!bit_is_clear(PINA,cButton))|0xE000;
    if(state[cButton] == 0xF000){return 1;}
    return 0;
}//debounceButton


//***********************************************************************
//                              readButtons                              
//**********************************************************************
uint8_t readButtons(void){ 
    uint8_t i;
    uint8_t value = 0;
   
    uint8_t cCstate = PORTC;
    uint8_t cAstate = PORTA;
    
    DDRA  =  0x00;              //set PORTA for input
    PORTA =  0xFF;              //set port A pull ups
    PORTC |= 0x70;              //turn on tristate buffer 

    for(i=0; i<8; i++){
        if(debounceButton(i)) {value |= 1<<i;}
    }//for

    DDRA  =  0xFF;              //set PORTA for output
    PORTA =  cAstate;
    PORTC =  cCstate;              //turn off tristate buffer

    return(value);
}//readButtons


//***********************************************************************
//                                selectMult                              
//**********************************************************************
void selectMult(/*uint8_t gcStatus*/void){
    switch(gcStatus & 0x03){
        case 0:
            gcDataMult = 1;
            break;
        case 1:
            gcDataMult = 2;
            break;
        case 2:
            gcDataMult = 4;
            break;
        case 3:
            gcDataMult = 0;
            break;
    }
}


//*******************************************************************************
//                                   createSegs                                    
//*******************************************************************************
void createSegs(uint16_t sum){

  gcaSegData[0] = sum%10;		    //set 1's
  gcaSegData[1] = sum/10%10;		//set 10's
  gcaSegData[2] = 10;			    //colon always off
  gcaSegData[3] = sum/100%10;		//set 100's
  gcaSegData[4] = sum/1000%10;		//set 1000's
	
  /*
  gcDigitCount = 0;
  if(gcDigitCount<5 &&  gcaSegData[gcDigitCount]!=0){
    gcDigitCount++;
    //gcDigitCount = 4;
  }//if
  */


  //set size of data for leading zeros
  if(gcaSegData[4] == 0){
    if(gcaSegData[3] == 0){
      if(gcaSegData[1] == 0){ gcDigitCount = 0; }
      else gcDigitCount = 1;	}
    else gcDigitCount = 3;	 }
  else gcDigitCount = 4;
}


/***********************************************************************/
//                           checkSpin
/***********************************************************************/
void checkSpin(uint8_t cEncLeft, uint8_t cEncRight){
  if(cEncLeft != 3){                                //if left not at home
     gbLeftDone = FALSE;                            //dial moving
     if((cEncLeft==1 && gcEncLeftPast ==3)||(cEncLeft ==0 && gcEncLeftPast==1)||(
         cEncLeft==2 && gcEncLeftPast ==0)){gbLeftDec = TRUE;}
     if((cEncLeft==2 && gcEncLeftPast ==3)||(cEncLeft ==0 && gcEncLeftPast==2)||(
         cEncLeft==1 && gcEncLeftPast ==0)){gbLeftInc = TRUE;}
  }//if
  else {gbLeftDone = TRUE;}

  if(cEncRight != 3){
     gbRightDone = FALSE;
     if((cEncRight==1 && gcEncRightPast ==3)||(cEncRight ==0 && gcEncRightPast==1)||(
         cEncRight==2 && gcEncRightPast ==0)){gbRightDec = TRUE;}
     if((cEncRight==2 && gcEncRightPast ==3)||(cEncRight ==0 && gcEncRightPast==2)||(
         cEncRight==1 && gcEncRightPast ==0)){gbRightInc = TRUE;}
  }//if
  else {gbRightDone = TRUE;}

}//checkSpin


/***********************************************************************/
//                           updateTime
/***********************************************************************/
/*updateTime(){
    //check right encoder     
    if(gbRightDone == TRUE){
        if(gbRightInc == TRUE){
            gsDisplayData += 1;
            if(gsDisplayData > 1023) {gsDisplayData = 0;}
            gbRightInc = FALSE;
        }
        if(gbRightDec == TRUE){
            gsDisplayData -= 1; 
            if(gsDisplayData < 0) {gsDisplayData = 1023;}
            gbRightDec = FALSE;
        }
    }//if

    //check left encoder     
    if(gbLeftDone == TRUE){
        if(gbLeftInc == TRUE){
            gsDisplayData += 2;
            if(gsDisplayData > 1023) {gsDisplayData = 0;}
            gbLeftInc = FALSE;
        }
        if(gbLeftDec == TRUE){
            gsDisplayData -= 2; 
            if(gsDisplayData < 0) {gsDisplayData = 1023;}
            gbLeftDec = FALSE;
        }
    }//if
}
*/

updateTime(uint16_t* data){
    //check right encoder     
    if(gbRightDone == TRUE){
        if(gbRightInc == TRUE){
            data[0] += 1;
            if(data[0] > 1023) {data[0] = 0;}
            gbRightInc = FALSE;
        }
        if(gbRightDec == TRUE){
            data[0] -= 1; 
            if(data[0] < 0) {data[0] = 1023;}
            gbRightDec = FALSE;
        }
    }//if

    //check left encoder     
    if(gbLeftDone == TRUE){
        if(gbLeftInc == TRUE){
            data[0] += 2;
            if(data[0] > 1023) {data[0] = 0;}
            gbLeftInc = FALSE;
        }
        if(gbLeftDec == TRUE){
            data[0] -= 2; 
            if(data[0] < 0) {data[0] = 1023;}
            gbLeftDec = FALSE;
        }
    }//if
}

/*************************************************************************/
//                           timer/counter0 ISR                          
//When the TCNT0 overflow interrupt occurs, the count_7ms variable is    
//incremented.  Every 7680 interrupts the minutes counter is incremented.
//tcnt0 interrupts come at 7.8125ms internals.
// 1/32768         = 30.517578uS
//(1/32768)*256    = 7.8125ms
//(1/32768)*256*64 = 500mS
/*************************************************************************/
ISR(TIMER0_COMP_vect){
  //static uint8_t encoder = 0;


     //turn on leds during isr
    PORTA = gcaLEDencode[gcaSegData[gcDigit]]; 	//encode the seg data and send to LED
    PORTC = gcDigit<<4;                              //select digit to display

  //read encoder data here so ISR dont fuck it up
  uint8_t cEncoderData = readEncoder();                    //read encoder into global?
  //uint8_t cEncoderData = readEncoder();
  uint8_t cEncLeft = cEncoderData & 0x03;          //seperate left data
  uint8_t cEncRight = (cEncoderData & 0x0C)>>2;    //seperate right data

 
  checkSpin(cEncLeft,cEncRight);  

  gcEncLeftPast = cEncLeft;
  gcEncRightPast = cEncRight;



  //read the button board
  gcStatus ^= readButtons();
 

  //write to any outputs 
  writeBarGraph(gcDataMult);
  //writeBarGraph(gcStatus);
}
     

/*************************************************************************/
//                              main
/*************************************************************************/
int main(){
  


  port_init();
  tcnt0_init();
  spi_init();
  sei();

  while(1){
    
    //update gcDataMult for increment  
    selectMult();

	updateTime(&gsDisplayData);
    
    //calulate decimal segments in gcaSegData
    createSegs(gsDisplayData);
    //gcDigitCount = 4;

    //write segments:
    DDRA = 0xFF;                                    //set PORTA for output
    gcDigit++;                                       //inc cDigit
    if(gcDigit > gcDigitCount) {gcDigit = 0;}         //bound digit count
    //PORTA = 0x00;                                   //set port a for output
    PORTA = gcaLEDencode[gcaSegData[gcDigit]]; 	//encode the seg data and send to LED
    PORTC = gcDigit<<4;                              //select digit to display
    _delay_ms(2);                                   //delay for 2ms

  


      
 }//while
}
