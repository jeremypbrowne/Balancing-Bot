#ifndef BASICPIDLIBRARY_h
#define BASICPIDLIBRARY_h

#define DECIMAL double
//#define DECIMAL float
class BasicPIDLibrary
{

  public:


	//commonly used functions **************************************************************************
	BasicPIDLibrary(DECIMAL Kp, DECIMAL Ki, DECIMAL Kd);

	
    bool Compute(DECIMAL, DECIMAL,DECIMAL&);                    // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. 

    void SetOutputLimits(DECIMAL, DECIMAL); // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application
	


	//available but not commonly used functions ********************************************************
	void setIntegral(DECIMAL dnewSum);
	
	DECIMAL getIntegral();
	
    void SetTunings(DECIMAL, DECIMAL,       // * While most users will set the tunings once in the 
                    DECIMAL);         	    //   constructor, this function gives the user the option
         	  

	void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
    void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
										  
										  
										  
	//Display functions ****************************************************************
	DECIMAL GetKp(){return kp;};						  // These functions query the pid for interal values.
	DECIMAL GetKi(){return ki;};						  //  they were created mainly for the pid front-end,
	DECIMAL GetKd(){return kd;};						  // where it's important to know what is actually 
	void DisableController(void);
	void EnableController(void);
  private:
	void Initialize();
	
	bool mEnabled;
	DECIMAL kp;                  // * (P)roportional Tuning Parameter
    DECIMAL ki;                  // * (I)ntegral Tuning Parameter
    DECIMAL kd;                  // * (D)erivative Tuning Parameter

			  
	unsigned long mLastTime;
	DECIMAL mOutputSum;
	DECIMAL	mLastInput;

	unsigned long mSampleTime;
	DECIMAL mOutMax;
	DECIMAL	mOutMin;
};
#endif

