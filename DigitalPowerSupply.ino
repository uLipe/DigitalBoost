
/*
 *						EMBARCADOS
 *
 *		Fonte chaveada Digital baseada em conversor boost
 *		@file   DigitalPowerSupply.ino
 *		@brief  aplicacao principal 
 *		@author FSN
 *
 */



#include "digitalboost.h"



//creates pwm and sensors object:
pwm_out boostPwm(PWM0_BASE_ADDRESS, 16000000);
analog_in boostAnalog(ADC0_BASE_ADRESS);
//Create a tick timer generator:
ticktimer loopTimer(TICK_TIMER0_BASE);
//And instantiate a PID voltage controller:
voltageloop boostController;



void setup()
{
	/* add setup code here */	
	Serial.begin(115200);
	
	//Setup hardware:
	boostPwm.InitPwm(50000);
	boostAnalog.AnalogInit();
	
	//Setup tick timer to tick each 50KHz
	loopTimer.SetTimerTick(16000000, 50000); 	

	//Set PWM base stuff:
	boostPwm.SetPwm(0, PWM_CH_C);
}

void loop()
{
	uint32_t pidtimer = 0;
	uint32_t updatetimer =0;
	uint32_t printtimer = 0;
	
	float targetVoltage = 0.0f;
	int16_t fix_voltage;
	
	//Initialize PID controller:
	boostController.kd = 0.1f;
	boostController.ki = 1.0f;
	boostController.kp = 10.0f;
	boostController.LoopInit();
	
	//Read first voltage:
	targetVoltage = (float)boostAnalog.AnalogGet(BOOST_ANALOG_TARGET_CH) * 210.938e-3f ;
	
	//saturate voltage:
	if(targetVoltage > 27.00f) targetVoltage = 27.00f;
	if(targetVoltage < 5.00f)  targetVoltage = 5.0f;
	
	
	//Read sensor voltage:
	boostController.feedback = boostAnalog.AnalogGet(BOOST_FEED_CH) * 54;
	
	//calculate target voltage in Q7.8:
	fix_voltage = (int16_t)(targetVoltage * 256.0f);
	
	//Pass setpoint:
	boostController.setpoint = fix_voltage;//boostAnalog.AnalogGet(BOOST_ANALOG_TARGET_CH) >> 1;
	
	
	//update timers:
	pidtimer = loopTimer.GetTicks();
	updatetimer = pidtimer;
    printtimer  = updatetimer;
	//run forever:
	for(;;)
	{
		//check for pid loop calculation:
		if(loopTimer.GetTicks() - pidtimer >= 1)
		{
			int16_t result = 0;
			
			//Read sensor voltage:
			boostController.feedback = boostAnalog.AnalogGet(BOOST_FEED_CH)  * 54;
			
			//Run pid loop:
			result = boostController.LoopRun() >> 16;
			
			//saturate value:
			if(result > 160) result =  160;
			if(result < 0)   result =  0;
			
			//scale and manage boost controller:
			boostPwm.SetPwm(result, PWM_CH_C);
			
			//Print each voltage:
			//Serial.print(result >> 8);
			
			//updade timer:
			pidtimer = loopTimer.GetTicks();			
		}
		
		
		//Check for target voltage update each 5ms:
		else if(loopTimer.GetTicks() - updatetimer >= 250)		
		{
			//read target voltage:
			targetVoltage = (float)boostAnalog.AnalogGet(BOOST_ANALOG_TARGET_CH) * 210.938e-3f ;			
			
			//saturate voltage:
			if(targetVoltage > 27.00f) targetVoltage = 27.00f;
			if(targetVoltage < 5.00f)  targetVoltage = 5.0f;			
			
			//calculate setpoint:
			fix_voltage = (int16_t)(targetVoltage * 256.0f);
	
			//Pass setpoint:
			boostController.setpoint = fix_voltage;//boostAnalog.AnalogGet(BOOST_ANALOG_TARGET_CH)  * 54;
			
			//update timer:
			updatetimer = loopTimer.GetTicks();			
		}
		
		
		//print target and setpoint voltages each 1s:
		else if(loopTimer.GetTicks() - printtimer >= 10000)
		{
			float feedVoltage = 0.0f;
			
			feedVoltage = (float)boostAnalog.AnalogGet(BOOST_FEED_CH) * 210.938e-3f;
			
			//Print voltages:
			Serial.print(targetVoltage);
			Serial.println();
			Serial.print(feedVoltage);
			Serial.println();
			
			printtimer = loopTimer.GetTicks(); 	
		}
		
	}
}
