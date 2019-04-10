/**********************************************************************************************
 * Basic PID Library - Version 0.1
 * by Jay Wilhelm <jwilhelm@ohio.edu>
 * original by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#include "Arduino.h"


#include "BasicPIDLibraryClass.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
BasicPIDLibrary::BasicPIDLibrary(double Kp, double Ki, double Kd)
{
    BasicPIDLibrary::SetOutputLimits(0, 255);				//default output limit corresponds to
												//the arduino pwm limits

    mSampleTime = 50;							//default Controller Sample Time is 0.1 seconds

    BasicPIDLibrary::SetTunings(Kp, Ki, Kd);

    mLastTime = millis()-mSampleTime;
	
	EnableController();
}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   BasicPIDLibrary Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool BasicPIDLibrary::Compute(DECIMAL iSetpoint,DECIMAL iInput,DECIMAL &pOutput)
{
	if(!mEnabled)
		return false;
	unsigned long now = millis();
	unsigned long timeChange = (now - mLastTime);
	if(timeChange >= mSampleTime)
	{	
		DECIMAL error = iSetpoint - iInput;
		DECIMAL dInput = (error - mLastInput);
		mOutputSum += error;

		DECIMAL output;
		output = (kp * error) +  (mOutputSum * ki) + (dInput * kd); //dt assumed in ki and kd

		if(output > mOutMax) 
			pOutput = mOutMax;
		else if(output < mOutMin) 
			pOutput = mOutMin;
		else{
			pOutput = output;					//pointer to output when output is within limits
		}

		//Remember some variables for next time 
		mLastInput = error;
		mLastTime = now;
		return true;
	}
	else 
		return false;
}


/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void BasicPIDLibrary::SetTunings(DECIMAL iKp, DECIMAL iKi, DECIMAL iKd){
   kp = iKp;
   ki = iKi * (DECIMAL)mSampleTime/1000.0;
   kd = iKd / (DECIMAL)mSampleTime/1000.0;
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void BasicPIDLibrary::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      //need ratio to keep relative to seconds?
	  DECIMAL ratio  = (DECIMAL)NewSampleTime
                      / (DECIMAL)mSampleTime;
      ki *= ratio;
      kd /= ratio;
      mSampleTime = (unsigned long)NewSampleTime;
   }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void BasicPIDLibrary::SetOutputLimits(DECIMAL Min, DECIMAL Max)
{
   if(Min >= Max) return;
   mOutMin = Min;
   mOutMax = Max;
}

void BasicPIDLibrary::DisableController(void)
{
	mEnabled = false;
}
void BasicPIDLibrary::EnableController(void)
{
	mEnabled = true;
	mOutputSum = 0.0;
	mLastInput = 0.0;
	mLastTime = 0;
}
void BasicPIDLibrary::setIntegral(DECIMAL dnewSum){
	mOutputSum = dnewSum;
	}
	
DECIMAL BasicPIDLibrary::getIntegral(){
	return mOutputSum;
	}

