#define F_CPU 16000000UL
#define BAUDRATE 9600
#define BAUD_PRESC ((F_CPU/(16UL*BAUDRATE))-1)
#define vout 52

#include <avr/io.h>
#include <avr/interrupt.h>

#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)

unsigned char dataArray[2];
unsigned char dataPointer = 0x00;
bool newData = false;
ISR(USART_RX_vect){
	dataArray[dataPointer] = UDR0;
	dataPointer++;
	if(dataPointer == 2) newData = true;	//Wait for second transfer
	dataPointer %= 2;	//Roll-over
}

//ISR(USART_TX_vect){}

unsigned char ADCVal;
unsigned char value;
ISR(ADC_vect){
	ADCVal = ADCH;
	//serialPrintLN(ADCH);
	if(ADCVal > vout-1){
		decPWM();
	}
	if(ADCVal < vout+1){
		incPWM();
	}
}


ISR(WDT_vect){
	PORTB = PORTB ^(1<<5);	
}

void serialPrint(unsigned char val){
	int i = 2;
	unsigned char dec[3];
	dec[2] = (val/100) + 48;
	dec[1] = ((val/10)%10) + 48;
	dec[0] = ((val%10)) + 48;
	do{
		while(!(UCSR0A & (1<<UDRE0)));
		UDR0 = dec[i];
		i--;
	} while(i>-1);
}

void serialPrintLN(unsigned char val){
	serialPrint(val);
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = '\n';
}

void delayMicroS(long i){
	//Remember Kenneth, the compiler knows if you do something just to waste cycles. It will just remove your code from the compiled code.
	//Use the assembly _NOP to faste cycles
	//At 16MHz 1µS is 16 cycles
	for(long j=0; j<(i*16); j++){
		_NOP();
		//asm volatile ("nop");
	}
}

void initSerial(){
	//Enable USART on power reduction register PRR
	PRR &= ~(1<<PRUSART0);
	
	//Disable digital input logic on comp pins
	DIDR1 &= ~(1<<AIN1D);
	DIDR1 &= ~(1<<AIN0D);
	//Setup
	UCSR0A = 0b00000000;
	UCSR0B = 0b10011000;  //RX-interrupt, RX-enable, TX-enable
	//UCSR0B = 0b11011000;  //RX-interrupt, TX-interrupt, RX-enable, TX-enable
	UCSR0C = 0b00000110;  //8 databits, 1 stopbit
	UBRR0H = (unsigned char)(BAUD_PRESC >> 8);
	UBRR0L = (unsigned char)(BAUD_PRESC);

}

void initWDT(){
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	WDTCSR = 0b01000101;
}

void initTimer0(){
	//Enable timer0 on power reduction register PRR
	PRR &= ~(1<<PRTIM0);  //Enable timer0 - enable=0

	//PWM setup
	TCCR0A |= (1<<COM0B1) | (1<<COM0B0);  //Set OCR0B on compare match counting up, reset at bottom (interting PWM)
	TCCR0A |= (1<<WGM00) | (1<<WGM01);  //Set fast-PWM (Top = 0xFF -> 256)
	//TCCR0A |= (1<<WGM00);  //Set phase-correct-PWM (Top = 0xFF -> 256)
	
	//TCCR0B |= (1<<CS01);  //Prescaler = 8 (7812.5Hz)
	TCCR0B |= (1<<CS00);  //Prescaler = 0 (62500Hz)

	//OCR0A & OCR0B - compare registers ( reset = 0xFF )
	OCR0A = 0xFF; 	//Use this for interrupt if needed - cannot be an output since it is the positive comparator pins
	OCR0B = 0xFF;	//Use this for PWM 

}
void initTimer1(){
	//Enable timer0 on power reduction register PRR
	PRR &= ~(1<<PRTIM1);  //Enable timer0 - enable=0

	//PWM setup
	TCCR1A |= (1<<COM1A1) | (1<<COM1A0);  //Set OCR0B on compare match counting up, reset at bottom (interting PWM)
	TCCR1A |= (1<<WGM10);  //Set fast-PWM (Top = 0xFF -> 256)
	TCCR1B |= (1<<WGM12) | (1<<CS10);	
	

	//OCR0A & OCR0B - compare registers ( reset = 0xFF )
	OCR1AH = 0x00;
	OCR1AL = 0xFF; 	//Use this for interrupt if needed - cannot be an output since it is the positive comparator pins
	OCR1BH = 0x00;
	OCR1BL = 0xFF;	//Use this for PWM 

}

void initADC(){
	PRR &= ~(1<<PRADC);
	////ADMUX  = 0b01100110;	//AVcc + ext. cap, left shift, channel 7
	ADMUX |= (1<<REFS1) | (1<<REFS0) | (1<<ADLAR) | (1<<MUX2) | (1<<MUX1);
	//ADMUX |= (1<<REFS0) | (1<<ADLAR) | (1<<MUX2) | (1<<MUX1);
	ADCSRA = 0b10101111;	//ADC enable, ADC auto trigger enable, ADC interrupt enable, 128 presc. 

	ADCSRA |= (1<<ADSC);	//Start the ADC
}

void portSetup(){
	DDRB |= (1<<5);
	PORTB |= (1<<5);	//Led on 13
	//DDRB |= (1<<1);	//T1 pwm out
	DDRD |= (1<<5);		//PWM output
}

char readADC(unsigned char ADCChannel){
	ADMUX = (ADMUX & 0xF0) | (ADCChannel & 0x0F); //Select channel

	ADCSRA |= (1<<ADSC);  //Start ADC
	while(ADCSRA & (1<<ADSC));
		//Wait for ADC to complete a read 
	return ADCH;
}

void setPWM(unsigned char pwm){
	OCR0B = (255-pwm);
}

void incPWM(){
	value = OCR0B -1;
	if(value < 1){
		value = 1;
	}
	OCR0B = value;
}
void decPWM(){
	value = OCR0B +1;
	if(value > 254){
		value = 254;
	}
	OCR0B = value;
}

int main(){	
	cli();  //Clear global interrupt
	initSerial();
	portSetup();
	initTimer0();
	//initTimer1();
	initWDT();
	initADC();
	sei();  //Set global interrupt flag
	
	//setPWM(2);
	while(1){
		//serialPrintLN(48);
		//delayMicroS(100000);
	}

	return 0;
}
