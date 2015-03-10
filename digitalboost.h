/*
 * digitalboost.h
 *
 * Created: 2/16/2015 5:50:46 PM
 *  Author: felipeneves
 */ 


#ifndef DIGITALBOOST_H_
#define DIGITALBOOST_H_

#include "hal.h"


//Class for voltage loop compensator block:
class voltageloop
{
public:
	//Voltage coeficient values:
	static float kp;				//Kp floating point value (will scaled to a Qformat)
	static float ki;				//Ki floating point value (will scaled to a Qformat)
	static float kd;				//Kd floating point value (will scaled to a Qformat)	
	
	static int16_t setpoint;		//input desired value
	static int16_t feedback;		//process variable measured value
	
	//Methods
	voltageloop();			//constructor
	static void LoopInit(); //method to reset voltage loop coeficients
	static int32_t LoopRun(); //method to run the compesation loop 
private:
	//Private stuff:
	static int16_t loopCoeff[3];
	static int16_t integrator;
	static int16_t error[2];
	static int32_t accumulator;		//accumulator to use multiplya-and-accumulate instructions
		
};


//class to implement a voltage boost converter:

#define BOOST_ANALOG_TARGET_CH 0 //AD converter channel used to capture voltage:
#define BOOST_FEED_CH		   1 //AD converter channel used to capture feedback sensor


class boostconverter
{
public:
	//voltage boost converter methods:
	boostconverter();
	static void BoostInit(float inputVoltage);
	static uint8_t SetTargetVoltageFromDigital(float v);
	static void SetTargetVoltageFromAnalog();
	static uint16_t MapCtlLaw(int16_t feedback);
	
private:
	static voltageloop *boostController;
	static float inputVoltage;
};


#endif /* DIGITALBOOST_H_ */