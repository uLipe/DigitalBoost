/*
 * hal.h
 *
 * Created: 2/16/2015 5:51:56 PM
 *  Author: felipeneves
 */ 

#ifndef HAL_H_
#define HAL_H_
#include <stdint.h>

/*
 *	Macro for hardware addressing:
 */


#define nullptr (void *)0x0000

/*
 * Analog base addresses:
 */

//Analog to digital converter base address
#define ADC0_BASE_ADRESS 0x78 
 

//Analog input class:
class analog_in
{
 public:
	analog_in(uint16_t adc_channel_base);
	static bool	AnalogInit();
	static int16_t AnalogGet(uint8_t channel); 
	
 private:
	static uint16_t analog_address;
			
};


/*
 *	 Atmega2560 pwm hardware abstraction layer:
 */


//These are the pwm objects base physical adress, other values may result in unpredictable behavior:
#define PWM0_BASE_ADDRESS 0x80
#define PWM1_BASE_ADDRESS 0x90
#define PWM2_BASE_ADDRESS 0xA0
#define PWM3_BASE_ADDRESS 0x120


//Channels in each pwm object:
#define PWM_CH_A 0x00
#define PWM_CH_B 0x01
#define PWM_CH_C 0x02

class pwm_out
{
public:
	pwm_out(uint16_t ocr_channel_base, uint32_t cpu_clk);
	static bool InitPwm(uint32_t frequency_base);
	static bool SetPwm(uint16_t dc, uint8_t channel);	
private:
	static uint32_t cpu_base;
	static uint16_t pwm_address;	
};


/*
 *	 Atmega2560 tick 1 ms timer hardware abstraction layer:
 */

#define TICK_TIMER0_BASE 0x44
#define TICK_TIMER1_BASE 0xB0

class ticktimer
{
public:
	ticktimer(uint16_t timer_channel_base);
	static bool SetTimerTick(uint32_t processorClock, uint32_t tickFreq);
	static bool WaitTicks(uint32_t ticks);
	static uint32_t GetTicks();	
	static void TimerCallback(void);

private:
	static volatile  uint32_t tickCounter;	
	static uint16_t timer_address;	
};

#endif /* HAL_H_ */