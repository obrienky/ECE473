//lab3_code
//Trevor Love
//10.25.2015
//

#define F_CPU 16000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "hd44780.h"


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

uint8_t gbAlarmSet = FALSE;
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
 DDRB = 0xFF; //all out
 DDRC = 0xFF;
 DDRD = 0xFF;
 DDRE = 0xFF;
 DDRF = 0x00; //all input
 DDRG = 0xFF;
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
//                              tcnt1_init                             
/***********************************************************************/
void tcnt1_init(void){
  TIMSK  |=  (1<<OCIE1A);			//enable timer/counter1 overflow interrupt
  TCCR1A |=  0x00; 								//normal operation
  TCCR1B |=  (1<<WGM12)|(1<<CS11)|(1<<CS10);	//CTC, OCR1A = top
  TCCR1C |=  0x00;								//no force compare
}


/***********************************************************************/
//                              tcnt2_init                             
/***********************************************************************/
void tcnt2_init(void){
  TCCR2  |=  (1<<WGM21)|(1<<WGM20)|(1<<COM21)|(1<<COM20)|(1<<CS20);
  //Fast-PWM, inverting mode, no prescaler
  OCR2    =  255;
}


/***********************************************************************/
//                              tcnt3_init                             
/***********************************************************************/
void tcnt3_init(void){
  TCCR3A |=  (1<<WGM30)|(1<<COM3A1);
  TCCR3B |=  (1<<WGM32)|(1<<CS32);
  OCR3A   =  70;
}


/***********************************************************************/
//                               ADC_init                             
/***********************************************************************/
void ADC_init(void){
  ADCSRA |= (1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2); 	//128 prescale
  ADMUX  |= (1<<REFS0)|(1<<ADLAR);				//ext. Vref, left adjusted
  ADCSRA |= (1<<ADEN); 							//do the first conversion
}


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
  
  //SPDR = 0xFF;						//send data	out
  //while(!(SPSR & (1<<SPIF))){};   	//wait for data
  //uint8_t data = SPDR;          		//read data in

  return(data);
}//readEncoder


/***********************************************************************/
//                           writeBarGraph
/***********************************************************************/
void writeBarGraph(uint8_t data){
  //SPDR = data;					//send data	out
  //while(!(SPSR & (1<<SPIF))){};   //wait for data

  spiTransceiver(data);				//send data to bar graph
  PORTB |=   0x01;          		//strobe output data reg in HC165 - rising edge
  PORTB &= ~(0x01);        	 		//falling edge
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
    PORTC =  cCstate;          	//turn off tristate buffer
    PORTA =  cAstate;


    return(value);
}//readButtons



/***********************************************************************/
//                           checkSpin
// takes in current encoder state and uses global variables for past
// encoder states;  writes to global variables used for incrementing and 
// decrementing values based on spin direction
// NLin:  gbEncLeftPast, gbEncRightPast
// NLout: gbLeftDone, gbLeftDec, gbLeftInc, gbRightDone, gbRightInc, gbRightDec
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
// write given time data to global segment variable for output
// in:    time: array of data for time: time[0]=hr, time[1]=min
// NLout: gcaSegData: array of decimal data for LED display
//*******************************************************************************
void time2segs(int8_t* time){
  gcaSegData[0] = time[1]%10;		//set min 1's
  gcaSegData[1] = time[1]/10%10;	//set min 10's
  
  uint8_t dispHr;

  if((gcStatus&(1<<4)) == (1<<4) ) { 
	dispHr = time[0]; } //24 hr
  else{
	if(dispHr > 12) { dispHr = time[0] - 12; }
    else { dispHr = time[0]; }
  }

  gcaSegData[3] = dispHr%10;		//set hr 1's

  if(dispHr >= 10){
  	gcaSegData[4] = dispHr/10%10;}	//set hr 10's
  else { gcaSegData[4] = 10; }

  if(gsSecCount >= HALFNUM){		//if upper half sec
	gcaSegData[2] = 10;	}			//colon off
  else{								//if lower half sec
	if(gbAlarmSet){ gcaSegData[2] = 11;} //all on if alarm on						
 	else { gcaSegData[2] = 12;	}	//colon on
  }			
}//time2segs



/***********************************************************************/
//                           changeTime
// uses global variables to determine encoder logic and changes time
// variable according
// NLio: gbLeftDone, gbLeftDec, gbLeftInc, gbRightDone, gbRightInc, gbRightDec
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
}//changeTime


/***********************************************************************/
//                           updateTime
/***********************************************************************/
void updateTime(int8_t* data){
//increment each second:
	//testing found 992 to be more accurate
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
uint8_t checkAlarm(void){
	if(gcaClockTime[0] >= gcaAlarmTime[0]){			//if clock past hr
		if(gcaClockTime[1] >= gcaAlarmTime[1]){		//if clock past min
			return 1; }								//alarm done
	}
	return 0;
}


/***********************************************************************/
//                           checkSnooze
/***********************************************************************/
uint8_t checkSnooze(void){
	if(gcaSnoozeTime[1] >= SNZMIN){			//if snooze past min
		if(gcaSnoozeTime[2] >= SNZSEC){		//if snooze past sec
			return 1; }						//snooze over
	}
	return 0;
}


/***********************************************************************/
//                           readADC0
/***********************************************************************/
uint8_t readADC0(void){
	ADCSRA |= (1<<ADSC);
	while(bit_is_clear(ADCSRA,ADIF)){};
	ADCSRA |= (1<<ADIF);
	uint8_t adc = ADCH;
	return(adc);
}


/***********************************************************************/
//                           setDimmer
/***********************************************************************/
void setDimmer(void){
	uint8_t light = readADC0();

	if(light>200)
		OCR2 = 50;
	else if(light>180)
		OCR2 = 65;
	else if(light>160)
		OCR2 = 80;
	else if(light>140)
		OCR2 = 105;
	else if(light>120)
		OCR2 = 135;
	else if(light>100)
		OCR2 = 175;
	else if(light>80)
		OCR2 = 205;
	else if(light>60)
		OCR2 = 230;
	else
		OCR2 = 250;

}


void changeVolume(void){

  if(gcStatus & (1<<7)){
  	if(OCR3A >= 60) { OCR3A -= 10; }
    gcStatus &= ~(1<<7);
  }

  if(gcStatus & (1<<6)){
  	if(OCR3A <= 150) { OCR3A += 10; }
    gcStatus &= ~(1<<6);
  }
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

  //increment counter for seconds
  gsSecCount++;
}


/*****************************************************************************/
//                           		Timer Interrupt 1 
/*****************************************************************************/
ISR(TIMER1_COMPA_vect){
  if(gbAlarm){ PORTD ^= (1<<PD1); }
  else{ PORTD &= ~(1<<PD1); }
}//ISR



/*************************************************************************/
//                              main
/*************************************************************************/
int main(){
  
  uint8_t cAlarmState = 1;

  port_init();
  tcnt0_init();
  tcnt1_init();
  tcnt2_init();
  tcnt3_init();
  ADC_init();
  spi_init();
  lcd_init();
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
           	gcStatus &= ~ 0x03;
            break;
	}


	//alarm control states
	switch(cAlarmState){
		case 1: 	//initialization
			clear_display();
			string2lcd("Alarm Off");
			cAlarmState = 10;

		case 10:	//alarm standby
			gbAlarmSet = FALSE;
			if((gcStatus & 0x04) == 4){ //if alarm mode on:
				clear_display();
				string2lcd("Alarm On");
				gbAlarmSet = TRUE;
				cAlarmState = 20; }		// go to alarm wait
			//	clear_display(); 
			break;

		case 20:	//alarm wait
			if(checkAlarm()){ 			//if alarm done
				clear_display();		//clear LCD
				string2lcd("WAKE UP!");
				cAlarmState = 30; }		//go to alarm on
			if((gcStatus & 0x04) != 4){ //if alarm mode off
				clear_display();		//clear LCD
				string2lcd("Alarm Off");
				cAlarmState = 10; }		//go to alarm standby
			break;

		case 30:	//alarm on
			gbAlarm = TRUE;				//turn alarm on
			if(gbAlarmACK != 0){		//if button pushed
				clear_display();
				gcaSnoozeTime[2] = 0;	//reset snooze sec
				gcaSnoozeTime[1] = 0;	//reset snooze min
				gcaSnoozeTime[0]++;		//inc snooze count
				gbAlarm = FALSE;		//turn alarm off
				gbAlarmACK = FALSE;		//turn off ack
				clear_display();		//clear LCD
				string2lcd("Snoozing");
				cAlarmState = 40;		//go to snooze
			}//if
			break;

		case 40:	//snooze on
			gbSnooze = TRUE;			//in snooze mode
			if(gcaSnoozeTime[0] > SNZCNT){	//if past snooze count
				clear_display();		//clear LCD
				string2lcd("Alarm Done");
				gbSnooze = FALSE;
				cAlarmState = 50; }		//go to alarm done
			if((gcStatus & 0x04) != 4){ //if alarm mode off
				clear_display();		//clear LCD
				string2lcd("Alarm Off");
				gbSnooze = FALSE;
				cAlarmState = 10; }		//go to alarm standby
			if(checkSnooze()){			//if snooze done
				clear_display();		//clear LCD
				string2lcd("WAKE UP!");
				gbSnooze = FALSE;
				cAlarmState = 30; }		//go to alarm on
			break;

		case 50:	//alarm done
			//string2lcd("Alarm Done");
			if(gcaClockTime[0] == 0 && gcaClockTime[1] == 0){ 	//if midnight
				gcaSnoozeTime[2] = 0;	//reset snooze sec
				gcaSnoozeTime[1] = 0;	//reset snooze min
				gcaSnoozeTime[0] = 0;	//reset snooze count
			//	clear_display();		//clear LCD
				cAlarmState = 10;		//go to standby
			}			
	}

	//turn on alarm if required
	if(gbAlarm == TRUE){ 		//if alarm on
		PORTG = 0xFF; }			//turn up volume
	else{						//if alarm off
		PORTG = ~0x00; }		//turn down volume
	


	//writeBarGraph(gcStatus);
	writeBarGraph( (1<<(cAlarmState/10 - 1)) );

	//change dim based on adc value
	setDimmer();

	//change volume based on input
	changeVolume();
    
    //write segments:
	cli();
    DDRA = 0xFF;                                //set PORTA for output
    gcDigit++;                                  //inc gcDigit
    if(gcDigit > 4) {gcDigit = 0;}         		//bound digit count
    PORTA = gcaLEDencode[gcaSegData[gcDigit]]; 	//encode the seg data and send to LED
    PORTC = gcDigit<<4;                         //select digit to display
    _delay_ms(2);                               //delay for 2ms
	sei();

      
 }//while
}//main
