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

#define SNZCNT		3		//3 snoozes	
#define SNZMIN 		0		//0 min snooze
#define SNZSEC		10		//10 sec snooze

#define SECNUM		992		//count number for a second
#define HALFNUM		496		//count number for half second

volatile uint8_t gcStatus = 0;		//statuses of buttons
//1:setClock 0:setAlarm


int8_t gcaClockTime[3] = {0};	//time for clock
int8_t gcaAlarmTime[3] = {0};	//time for alarm
// (hr, min, sec)

int8_t gcaSnoozeTime[3] = {0};
// (cnt, min, sec)

static volatile uint16_t gsSecCount = 0;	//counter for seconds
//count of 1023 equals one second

volatile uint8_t gcEncLeftPast = 0;		//saved left encoder value
volatile uint8_t gcEncRightPast = 0;	//saved right encoder value
uint8_t gbLeftDone = FALSE;				//left encoder at home
uint8_t gbLeftInc = FALSE;				//left encoder was incremented
uint8_t gbLeftDec = FALSE;				//left encoder was decremented
uint8_t gbRightDone = FALSE;			//right encoder at home
uint8_t gbRightInc = FALSE;				//right encoder was incremented
uint8_t gbRightDec = FALSE;				//right encoder was decremented

uint8_t gcDigit = 0;					//digit for LED display
uint8_t gcaSegData[5];					//data for LED display

uint8_t gbSnooze = FALSE;				//snooze timer on
uint8_t gbAlarm = FALSE;				//alarm on
uint8_t gbAlarmACK = FALSE;				//alarm acknowledge


//0-9 are digits, 10=OFF, 11=ON, 12=colon
uint8_t gcaLEDencode[13] = { 0b11000000, 0b11111001, 0b10100100,  0b10110000, 0b10011001, 
0b10010010, 0b10000010, 0b11111000, 0b10000000, 0b10010000, 0xFF, 0x00, 0xFC};
	

/***********************************************************************/
//                           port_init                               
/***********************************************************************/
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
  SPCR  |=   (1<<SPE)|(1<<MSTR)|(1<<SPR0); 	//set up SPI mode
  SPSR  |=   (1<<SPI2X);             		//double speed operation
 }//spi_init

/***********************************************************************/
//                              tcnt0_init                             
// (32/32768) = 30.51ns
/***********************************************************************/
void tcnt0_init(void){
  ASSR   |=  1<<AS0;  				//ext osc TOSC
  TIMSK  |=  1<<OCIE0;				//enable timer/counter0 overflow interrupt
  TCCR0  |=  1<<WGM01 | 1<<CS00; 	//CTC mode, no prescale
  OCR0 = 32;						//timer compare register value
}
/***********************************************************************/

/***********************************************************************/
//                            spiTransceiver
/***********************************************************************/
uint8_t spiTransceiver(uint8_t dataOut){
	SPDR = dataOut;					//send data	out
	while(!(SPSR & (1<<SPIF))){};   //wait for data
  	uint8_t dataIn = SPDR;          //read data in
	return(dataIn);
}

/***********************************************************************/
//                            readEncoder
/***********************************************************************/
uint8_t readEncoder(void){
  PORTE &= ~(1<<PE6);      				//load in || inputs
  PORTE |= (1<<PE6);            		//switch to shift
  uint8_t data = spiTransceiver(0x00);	//read encoder
  
  //SPDR = 0xFF;					//send data	out
  //while(!(SPSR & (1<<SPIF))){};   //wait for data
  //uint8_t data = SPDR;          //read data in

  return(data);
}//readEncoder


/***********************************************************************/
//                           writeBarGraph
/***********************************************************************/
void writeBarGraph(uint8_t data){
  //SPDR = data;					//send data	out
  //while(!(SPSR & (1<<SPIF))){};   //wait for data

  spiTransceiver(data);		//send data to bar graph
  PORTB |=   0x01;          //strobe output data reg in HC165 - rising edge
  PORTB &= ~(0x01);         //falling edge
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

    DDRA  =  0xFF;             	//set PORTA for output
    PORTA =  cAstate;
    PORTC =  cCstate;          	//turn off tristate buffer

    return(value);
}//readButtons



/***********************************************************************/
//                           checkSpin
/***********************************************************************/
void checkSpin(uint8_t cEncLeft, uint8_t cEncRight){
  if(cEncLeft != 3){                    //if left not at home
     gbLeftDone = FALSE;           		//dial moving
     if((cEncLeft==1 && gcEncLeftPast ==3)||(cEncLeft ==0 && gcEncLeftPast==1)||(
         cEncLeft==2 && gcEncLeftPast ==0)){gbLeftDec = TRUE;}	//logic for dec
     if((cEncLeft==2 && gcEncLeftPast ==3)||(cEncLeft ==0 && gcEncLeftPast==2)||(
         cEncLeft==1 && gcEncLeftPast ==0)){gbLeftInc = TRUE;}	//logic for inc
  }//if
  else {gbLeftDone = TRUE;}				//if left at home

  if(cEncRight != 3){					//if right not at home
     gbRightDone = FALSE;				//dial moving
     if((cEncRight==1 && gcEncRightPast ==3)||(cEncRight ==0 && gcEncRightPast==1)||(
         cEncRight==2 && gcEncRightPast ==0)){gbRightDec = TRUE;}  //logic for dec
     if((cEncRight==2 && gcEncRightPast ==3)||(cEncRight ==0 && gcEncRightPast==2)||(
         cEncRight==1 && gcEncRightPast ==0)){gbRightInc = TRUE;}  //logic for inc
  }//if
  else {gbRightDone = TRUE;}			//if right at home

}//checkSpin



//*******************************************************************************
//                  		time2segs
//*******************************************************************************
void time2segs(int8_t* time){
  gcaSegData[0] = time[1]%10;		//set min 1's
  gcaSegData[1] = time[1]/10%10;	//set min 10's
  gcaSegData[3] = time[0]%10;		//set hr 1's
  gcaSegData[4] = time[0]/10%10;	//set hr 10's
  if(gsSecCount >= HALFNUM){		//if upper half sec
	gcaSegData[2] = 10;	}			//colon off
  else{								//if lower half sec
 	gcaSegData[2] = 12;	}			//colon on
}



/***********************************************************************/
//                           changeTime
/***********************************************************************/
void changeTime(int8_t* data){
    //check right encoder     
    if(gbRightDone == TRUE){
        if(gbRightInc == TRUE){
            data[1]++;
            if(data[1] > 59) {data[1] = 0;}
            gbRightInc = FALSE; }
        if(gbRightDec == TRUE){
            data[1]--; 
            if(data[1] < 0) {data[1] = 59;}
            gbRightDec = FALSE; }
    }//if

    //check left encoder     
    if(gbLeftDone == TRUE){
        if(gbLeftInc == TRUE){
			data[0]++;
            if(data[0] > 23) {data[0] = 0;}
            gbLeftInc = FALSE; }
        if(gbLeftDec == TRUE){
            data[0]--; 
            if(data[0] < 0) {data[0] = 23;}
            gbLeftDec = FALSE; }
    }//if
}


/***********************************************************************/
//                           updateTime
/***********************************************************************/
void updateTime(int8_t* data){
//increment each second:
	//testing gound 992 to be more accurate
	if(gsSecCount >= SECNUM){					//1024 count is 1 sec
		//gsSecCount %= 1023;					//reset counter
		gsSecCount = 0;		
		gcaClockTime[2]++;					//inc sec
		if(gbSnooze == TRUE){gcaSnoozeTime[2]++;}
		if(gcaClockTime[2] == 60){			//if sec at 60
			gcaClockTime[2] = 0;			//reset seconds
			gcaClockTime[1]++;				//inc min
			if(gcaClockTime[1] == 60){		//if min at 60
				gcaClockTime[1] = 0;		//reset min
				gcaClockTime[0]++;			//inc hr
				if(gcaClockTime[0] == 24){	//if hr at 24
					gcaClockTime[0] = 0;	//reset hr
				}
			}
		}
	}


/*
	//increment min each second:
	if(gsSecCount >= 1023){		//1024 count is 1 sec
		gsSecCount %= 1023;		//reset counter
		gcaClockTime[1]++;				//inc min
		if(gbSnooze == TRUE){gcaSnoozeTime[2]++;}
		if(gcaClockTime[1] == 60){		//if min at 60
			gcaClockTime[1] = 0;		//reset min
			gcaClockTime[0]++;			//inc hr
			if(gcaClockTime[0] == 24){	//if hr at 24
				gcaClockTime[0] = 0;	//reset hr
			}
		}
	}
*/

	//update snooze counter if needed
	if(gbSnooze == TRUE){
		if(gcaSnoozeTime[2] == 60){		//if sec at 60
			gcaSnoozeTime[2] = 0;		//reset seconds
			gcaSnoozeTime[1]++;			//inc min		
		}
	}


}



/***********************************************************************/
//                           checkAlarm
/***********************************************************************/
uint8_t checkAlarm(){
	if(gcaClockTime[0] >= gcaAlarmTime[0]){			//if clock past hr
		if(gcaClockTime[1] >= gcaAlarmTime[1]){		//if clock past min
			return 1; }								//alarm done
	}
	return 0;
}


/***********************************************************************/
//                           checkSnooze
/***********************************************************************/
uint8_t checkSnooze(){
	if(gcaSnoozeTime[1] >= SNZMIN){			//if snooze past min
		if(gcaSnoozeTime[2] >= SNZSEC){		//if snooze past sec
			return 1; }						//snooze over
	}
	return 0;
}


/*************************************************************************/
//                           timer/counter0 ISR                          
// 1/32768         	= 30.517578uS
//(1/32768)*32    	= 9.7656ms
//(1/32768)*32*1024 = 1000mS
/*************************************************************************/
ISR(TIMER0_COMP_vect){
  //turn on leds during isr
  PORTA = gcaLEDencode[gcaSegData[gcDigit]]; 	   //encode the seg data and send to LED
  PORTC = gcDigit<<4;                              //select digit to display

  //read encoder data
  uint8_t cEncoderData = readEncoder();            //read encoder
  uint8_t cEncLeft = cEncoderData & 0x03;          //seperate left data
  uint8_t cEncRight = (cEncoderData & 0x0C)>>2;    //seperate right data

  //check for movement and save state
  checkSpin(cEncLeft,cEncRight);  
  gcEncLeftPast = cEncLeft;
  gcEncRightPast = cEncRight;

  //read button board
  if(gbAlarm == TRUE) { 
	if(gbAlarmACK == FALSE){
		gbAlarmACK = readButtons(); }
	}
  else { gcStatus ^= readButtons(); }

  //gcStatus ^= readButtons();

  //increment counter for seconds
  gsSecCount++;
 
}
     

/*************************************************************************/
//                              main
/*************************************************************************/
int main(){
  
  uint8_t cAlarmState = 10;

  port_init();
  tcnt0_init();
  spi_init();
  sei();

  while(1){
   
	//check status and update clocks
    switch(gcStatus & 0x03){
        case 0:							//regular operation
            updateTime(gcaClockTime);	//update clock
			time2segs(gcaClockTime);	//output clock to LED
			gbRightInc = FALSE;			//turn off encoder logic
			gbRightDec = FALSE;
			gbLeftInc = FALSE;
			gbLeftDec = FALSE;
            break;
        
		case 1:							//alarm set mode
            changeTime(gcaAlarmTime);	//change alarm
			time2segs(gcaAlarmTime);	//output alarm time
			updateTime(gcaClockTime);	//update clock
            break;

        case 2:							//clock set mode
            changeTime(gcaClockTime);	//update time based on encoders
			time2segs(gcaClockTime);	//output clock to LED
			gsSecCount = 0;				//reset ms count
			gcaClockTime[2] = 0;		//reset seconds
            break;

        case 3:
           	//do nothing
            break;
	}


	//alarm control states
	switch(cAlarmState){
		case 10:	//alarm standby
			if((gcStatus & 0x04) == 4){ //if alarm mode on:
				cAlarmState = 20; }		// go to alarm wait 
			break;

		case 20:	//alarm wait
			if(checkAlarm()){ 			//if alarm done
				cAlarmState = 30; }		//go to alarm on
			if((gcStatus & 0x04) != 4){ //if alarm mode off
				cAlarmState = 10; }		//go to alarm standby
			break;

		case 30:	//alarm on
			gbAlarm = TRUE;				//turn alarm on
			if(gbAlarmACK != 0){		//if button pushed
				gcaSnoozeTime[2] = 0;	//reset snooze sec
				gcaSnoozeTime[1] = 0;	//reset snooze min
				gcaSnoozeTime[0]++;		//inc snooze count
				gbAlarm = FALSE;		//turn alarm off
				gbAlarmACK = FALSE;		//turn off ack
				cAlarmState = 40;		//go to snooze
			}//if
			break;

		case 40:	//snooze on
			gbSnooze = TRUE;			//in snooze mode
			if(gcaSnoozeTime[0] > SNZCNT){	//if past snooze count
				gbSnooze = FALSE;
				cAlarmState = 50; }		//go to alarm done
			if((gcStatus & 0x04) != 4){ //if alarm mode off
				gbSnooze = FALSE;
				cAlarmState = 10; }		//go to alarm standby
			if(checkSnooze()){			//if snooze done
				gbSnooze = FALSE;
				cAlarmState = 30; }		//go to alarm on
			break;

		case 50:	//alarm done
			if(gcaClockTime[0] == 0 && gcaClockTime[1] == 0){ 	//if midnight
				gcaSnoozeTime[2] = 0;	//reset snooze sec
				gcaSnoozeTime[1] = 0;	//reset snooze min
				gcaSnoozeTime[0] = 0;	//reset snooze count
				cAlarmState = 10;		//go to standby
			}			
	}

	//turn on alarm if required
	if(gbAlarm == TRUE){
		PORTB |= 0xF0; }
	else{
		PORTB &= ~0xF0; }
	


	//writeBarGraph(gcStatus);
	writeBarGraph( (1<<(cAlarmState/10 - 1)) );


    //write segments:
    DDRA = 0xFF;                                //set PORTA for output
    gcDigit++;                                  //inc gcDigit
    if(gcDigit > 4) {gcDigit = 0;}         		//bound digit count
    PORTA = gcaLEDencode[gcaSegData[gcDigit]]; 	//encode the seg data and send to LED
    PORTC = gcDigit<<4;                         //select digit to display

    _delay_ms(2);                                   //delay for 2ms

      
 }//while
}//main
