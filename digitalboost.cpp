/*
 * digitalboost.cpp
 *
 * Created: 2/16/2015 5:51:06 PM
 *  Author: felipeneves
 */ 

#include "digitalboost.h"

//
//boost converter voltage loop controller:
//

#define KP_COEF 0
#define KI_COEF 1
#define KD_COEF 2

	//Voltage coeficient values:
float voltageloop::kp;				//Kp floating point value (will scaled to a Qformat)
float voltageloop::ki;				//Ki floating point value (will scaled to a Qformat)
float voltageloop::kd;				//Kd floating point value (will scaled to a Qformat)
	
int16_t voltageloop::setpoint;		//input desired value
int16_t voltageloop::feedback;		//process variable measured value


int16_t voltageloop::loopCoeff[3];
int16_t voltageloop::integrator;
int16_t voltageloop::error[2];
int32_t voltageloop::accumulator;		

//
// Constructor, voltageloop
//
voltageloop::voltageloop()
{
	//Load as a proportional only controller as default:
	voltageloop::kp = 1.0f;
	voltageloop::ki = 0.0f;
	voltageloop::kd = 0.0f;		
}

//
//	LoopInit()
//
void voltageloop::LoopInit()
{
	int16_t i = 0;
	
	//clears all coefficients
	for(i = 0; i < 3; i++) voltageloop::loopCoeff[i] = 0;
	
	//clear integrator history:
	voltageloop::integrator = 0;
	
	//clear error history:
	voltageloop::error[0] = 0;
	voltageloop::error[1] = 0;
	
	//compute the pid coeeficients sets:
	voltageloop::loopCoeff[KP_COEF] = (int16_t)(voltageloop::kp * 256.0f);
	voltageloop::loopCoeff[KI_COEF] = (int16_t)(voltageloop::ki * 256.0f);
	voltageloop::loopCoeff[KD_COEF] = (int16_t)(voltageloop::kd * 256.0f);

}

//
//	LoopRun()
//
int32_t voltageloop::LoopRun()
{
	int16_t delta = 0;
	
	//clears accumulator:
	voltageloop::accumulator = 0;
	
	//compute error:
	voltageloop::error[0] = voltageloop::setpoint - voltageloop::feedback;
	
	//Compute pid:
	voltageloop::integrator += voltageloop::error[0];
	if(voltageloop::integrator > 16384)voltageloop::integrator = 16384;
	if(voltageloop::integrator < -16384)voltageloop::integrator= -16384;
	
	delta = voltageloop::error[0] - voltageloop::error[1];
	
	
	//accumulate actions (fractional multiply-accumulate):
	voltageloop::accumulator = ((int32_t)voltageloop::error[0] * voltageloop::loopCoeff[KP_COEF]) << 1;
	voltageloop::accumulator += ((int32_t)voltageloop::integrator * voltageloop::loopCoeff[KP_COEF]) << 1; 
	voltageloop::accumulator += ((int32_t)voltageloop::integrator * voltageloop::loopCoeff[KP_COEF]) << 1;
		
	//update history:
	voltageloop::error[1] = voltageloop::error[0];
	
	return(voltageloop::accumulator);	
}

//
// Boost dc-dc converter impplementation:
//


//private variable stuff:
voltageloop *boostconverter::boostController = new voltageloop();	//Boost voltage loop ctl
float boostconverter::inputVoltage;

//
// Constructor: boostConverter()
//
boostconverter::boostconverter()
{	
	//Load default controller coefficients:
	boostController->kd = 0.0f;
	boostController->ki = 0.1f;
	boostController->kp = 10.0f;
	
	//Inits:
	boostController->LoopInit();	
		
	boostconverter::inputVoltage = 1.0f;
}
//
// BoostInit()
//
void boostconverter::BoostInit(float inputVoltage)
{
	//check input voltage:
	if(inputVoltage <= 0.0f) 
	{
		inputVoltage = 1.0f;	
		
	}
	
	//maps the our input voltage:
	boostconverter::inputVoltage = inputVoltage;
	
}
//
//	SetTargetVoltage:
//
uint8_t boostconverter::SetTargetVoltageFromDigital(float v)
{
	float d = 0.0f;
	uint8_t d_fix = 0;
	//calculate the duty cicle ratio based on boost transfer function:
	//
	// m(d) = vout/vin = 1/(1 - d)
	// d = 1 - vin/vout
	//
	d = 1.0f - (boostconverter::inputVoltage / v);
	
	//convert to Q8.0 format:
	d_fix = (uint8_t)(d * 256.0f);
	
	//Maps the new setpoint in Q7.8:
	boostController->setpoint = (int16_t)(v * 256.0f);

	//get initial duty cicle:
	return(d_fix);
}
//
// SetTargetVoltageFromAnalog
//
void boostconverter::SetTargetVoltageFromAnalog()
{
	//TODO	
}
//
//	MapCtlLaw()
//
uint16_t boostconverter::MapCtlLaw(int16_t feedback)
{
	int16_t d_fix;
	
	//Read current voltage:
	boostController->feedback = feedback;
	
	//Runs the boost voltage loop:
	d_fix = (int16_t)((boostController->LoopRun()) >> 16) ;
	
	return((uint16_t)d_fix);
}

	