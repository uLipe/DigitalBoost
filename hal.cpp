/*
 * hal.cpp
 *
 * Created: 2/16/2015 5:51:37 PM
 *  Author: felipeneves
 */ 

//Include stuff here:
#include "Arduino.h"	//Arduino base hardware plus avr base registers.
#include "hal.h"		//classes interface.

//
//	Analog in class implementations:
//

uint16_t analog_in::analog_address;

typedef struct  
{
	uint8_t _ADCL;
	uint8_t _ADCH;
	uint8_t _ADCSRA;
	uint8_t _ADCSRB;
	uint8_t _ADMUX;
	uint8_t _DIDR2;
	uint8_t _DIDR0;
	uint8_t _DIDR1;
}analog_t;

/*
 *	analog_in()
 */
analog_in::analog_in(uint16_t adc_channel_base)
{
	//maps the base address of analog to digital registers
	analog_address = (uint16_t)adc_channel_base;
}

/*
 * AnalogInit()
 */
bool analog_in::AnalogInit()
{
	//take A/D base adress
	volatile analog_t *ptr = (analog_t*)analog_address;
	
	//if its a null pointer:
	if(ptr == nullptr)return(false);
	
	//since we are using Arduino, so all pins of analog
	//section are used ONLY as analog input:
	DDRF = 0x00;
	DDRK = 0x00;
	
	//the adc configuration used is very basic,
	//so its polled,with conversion rate about 70ksps @ 8bits
	
	ptr->_ADMUX = 0x60; //base initial mux state
	ptr->_ADCSRB = 0x00; //base initial ADC config
	ptr->_ADCSRA = 0x84; //adc in single conversion enabled @ 1MHz clsk
	ptr->_DIDR0  = 0xFF; //Put all ddrf / ddrk as analog inputs
	ptr->_DIDR1  = 0x00;
	ptr->_DIDR2  = 0xFF;	
	
	//run the first conversion to stabilize the vref:
	ptr->_ADCSRA |= (1 << ADSC);
	
	//wait:
	while(ptr->_ADCSRA & ( 1 << ADSC));	

	return(true);
}

/*
 * AnalogGet()
 */

int16_t analog_in::AnalogGet(uint8_t channel)
{
	//take A/D base adress
	volatile analog_t *ptr = (analog_t*)analog_address;
	uint8_t i = 0;
	int16_t result = (-1);

	//check for invalid channel:
	if(channel > 15)return(result);
	
	//check channel offset:
	if(channel <= 7)
	{
		//clears high mux bit
		ptr->_ADCSRB &= ~(1 << MUX5);	
	}
	else
	{
		//clears high mux bit:
		ptr->_ADCSRB |= (1 << MUX5);
		
		//maps the higher ADC channels:
		channel = 8 - channel;
	}
	
	//assign the adc channel:
	ptr->_ADMUX |= channel;
		
	//wait to sample/hold ckt:
	for(i = 0; i < 0xFF; i++);
		
	//start the conversion:
	ptr->_ADCSRA |= (1 << ADSC);
		
	//wait to complete:
	while(ptr->_ADCSRA & (1 << ADSC));
		
	//conversion over, take result:
	result = ptr->_ADCH;
		
	//reset channel:
	ptr->_ADMUX &= ~(0x1F);		
	
	
	//returns the converted value:
	return(result);
}

//
//	PWM class implementation
//

//Declare privates members of pwm class:
uint16_t pwm_out::pwm_address;
uint32_t pwm_out::cpu_base;

//HW Pwm structure definition:
typedef struct  
{
	uint8_t _TCCRA;			//Ttimer configuration registers
	uint8_t _TCCRB;
	uint8_t _TCCRC;
	uint8_t reserved;
	uint8_t  _TCNTL;		//Timer period registers
	uint8_t  _TCNTH;
	uint16_t _ICR;			//Timer input capture value register
	uint16_t _OCRA;			//Timer output capture value registers
	uint16_t _OCRB;
	uint16_t _OCRC;	
}pwm_t;


/*
 *	pwm_out() -- constructor
 */
pwm_out::pwm_out(uint16_t ocr_channel_base, uint32_t cpu_clk)
{
	//set hardware PWM base Adreess:
	pwm_address = (uint16_t)ocr_channel_base;

	//sets the cpu clock frequency:
	cpu_base = cpu_clk;
}

/*
 *	InitPwm()
 */
bool pwm_out::InitPwm(uint32_t frequency_base)
{
	uint32_t reloadVal;
	//Acesse pwm base registers
	volatile pwm_t* ptr = (pwm_t *)pwm_address;
	
	if(ptr == nullptr)return(false);	
	
	//calculates the timer reload value:
	reloadVal = (cpu_base / frequency_base) + 1;
	
	//Setup all pwms pins direcion:
	MCUCR &= ~( 1 << PUD);
	DDRB |= 0xf0;
	DDRE |= 0x38;
	DDRG |= 0x20;
	DDRH |= 0x78;
	DDRL |= 0x38;
	
	
	//Setup pwm hardware:
	ptr->_TCCRB = 0x18;
	ptr->_TCCRA = 0xAA;
	ptr->_TCCRC = 0x00; //Per pwm object we have 3 channels,initially disabled
	
	//Assign pwm frequency:
	ptr->_ICR = (uint16_t)reloadVal; 
	
	//Assign pwm initial dutycicle:
	ptr->_OCRA = 0x0000;
	ptr->_OCRB = 0x0000;
	ptr->_OCRC = 0x0000;
	
	//Enable pwm counting:
	ptr->_TCCRB |= 0x01;	

	return(true);
}

/*
 * SetPWM
 */
bool pwm_out::SetPwm(uint16_t dc, uint8_t channel)
{
	//Access pwm physical block
	volatile pwm_t* ptr = (pwm_t *)pwm_address;
	
	//check for invalid ptr:
	if(ptr == nullptr)return(false);
	
	//wrap dutycicle, if necessary:
	if(dc > ptr->_ICR) dc = ptr->_ICR;
	
	//select channel:
	switch(channel)
	{
		case PWM_CH_A: ptr->_OCRA = dc;
		break;
		
		case PWM_CH_B: ptr->_OCRB = dc;
		break;
		
		case PWM_CH_C: ptr->_OCRC = dc;
		break;
		
		//invalid channel, don't execute action:
		default: return(false);				
	}
	
	return(true);	
}

//
//	1 ms Tick timer hardware abstraction layer:
//

void (*timerVectorArray[2])(void) = {0, 0};	
volatile uint32_t ticktimer::tickCounter;
uint16_t ticktimer::timer_address;	

//Tick timer is based on timer0 or 2 structure, as follows:
typedef struct  
{
	uint8_t _TCCRA;
	uint8_t _TCCRB;
	uint8_t _TCNT;
	uint8_t _OCRA;
	uint8_t _OCRB;		
}timer_t;



/*
 * ticktimer() -- constructor
 */

ticktimer::ticktimer(uint16_t timer_channel_base)
{
	timer_address = (uint16_t)timer_channel_base;
	tickCounter = 0;	
}
/*
 * SetTimerTick:
 */
bool ticktimer::SetTimerTick(uint32_t processorClock, uint32_t tickFreq)
{
	//access hardware timer block
	volatile timer_t* ptr = (timer_t *)timer_address;
	
	uint32_t reloadVal;
	
	//check for invalid pointer:
	if(ptr == nullptr)return(false);

	reloadVal = (processorClock / tickFreq);	
	if(reloadVal > 0xFF) reloadVal = 0xFF;
		
	cli();	
		
	//now, configure tick timer for periodic interrupt:
	ptr->_TCCRB = 0x00;
	ptr->_TCCRA = 0x02;
	ptr->_OCRA  = (uint8_t)reloadVal;
	
	ptr->_OCRB = (uint8_t)reloadVal;
	
	//configure its interrupt:
	switch(timer_address)
	{
		//Enable tick interrupt:
		case TICK_TIMER0_BASE:
			TIMSK0 = (1 << OCIE0A);
			timerVectorArray[0] = &TimerCallback;
		break;
		
		
		case TICK_TIMER1_BASE:
			TIMSK2 = (1 << OCIE0A);
			timerVectorArray[1] = &TimerCallback;
		break;
	}
	
	//start timer and assert timer prescaler:
	ptr->_TCCRB |= 0x02;
	
	//enable global interrupts:
	sei();
	return(true);
}

/*
 *	GetMilis()
 */
uint32_t ticktimer::GetTicks()
{
	uint32_t tmp;
	
	cli();
	tmp = tickCounter;
	sei();
	
	return(tmp);
}

/*
 *	WaitMilisecond()
 */
bool ticktimer::WaitTicks(uint32_t ticks)
{
	uint32_t cnt = 0;
	
	cli();
	cnt = tickCounter;
	sei();
	
	//wait for timer requested:
	for(;;)
	{
		if(tickCounter - cnt >= ticks)break;
	}
	
	return(true);	
}
/*
 * TimerCallBack()
 */
void ticktimer::TimerCallback()
{
	tickCounter++;
}


//
//Tick timer interrupt run-time code:
//

//General timer processsing routinte:
void ProcessTimers(void)
{
	void( *ptrCallBack )(void );	
	uint8_t timerMask;
	
	//read interrupts status:
	timerMask  = (TIFR0 & (1 << OCF0A)) >> 1;
	timerMask |= (TIFR2 & (1 << OCF2A));
	
	timerMask = (~timerMask & 0x03);
	
	//select the vectors will be executed:
	if(timerMask & 0x01)
	{
		
		ptrCallBack = timerVectorArray[0];
		if(ptrCallBack != nullptr) ptrCallBack();		
	}
	
	if(timerMask & 0x02)
	{
		ptrCallBack = timerVectorArray[1];
		if(ptrCallBack != nullptr) ptrCallBack();		
	}
	
}

ISR(TIMER0_COMPA_vect)
{
	//Jump to timer tick processing handler
	ProcessTimers();
}


ISR(TIMER2_COMPA_vect)
{
	//Jump to timer tick processing handler
	ProcessTimers();	
}	