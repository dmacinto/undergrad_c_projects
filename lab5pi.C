// ========================================================================
// Name					: Rabbit Control Skeleton
// Description 	: This files gives you the basic structure to control analog devices with the rabbit microprocessor.
// 						You need Dynamic C to compile it.
//
// ========================================================================
#use "ES308_SBC.LIB"																// Library with ES308 SBC routines
#use "ROVER_LIB.LIB"																// Library with some additional functions

// ===================================================
// Timing Parameters
// ===================================================
#define XTAL_FREQ    (9.216)										// Crystal frequency (MHz) internal clock rate
																			// read from silver raised oval
#define CONTROL_FREQ (100.0)									// Sampling frequency (Hz):  	This tells the rabbit how frequently to
																		// call the control code
#define LOG_FREQ     (100.0)                            // Data log frequency (Hz)


// ===================================================
// Sensor Coefficients
// ===================================================
#define pi 				(3.1415926)												   // pi


nodebug root interrupt void ControlRoutine(void);		   // Don't mess with

// ===================================================
// Declare Global Variables
// ===================================================
float ControlIteration;
float Time;
float SamplePeriod;
float Theta;
float Tstop;

float Ts;
int LogIndex;
int LogIndexFinal;
float LogIteration;
float LogTime[999];
int j;
// List all the variables you want to log here, note you can make up the array length as 999
float LogTheta[999];
float Ktach;
float Kenc;
float Omega;
float LogOmega[999];
float Ko;
float DesOmega;
float Error[2];
float MotorVolts[2];
float LogError[999];
float LogMotorVolts[999];
float DesTheta;




// ====================================================================================================================
// Main()
// ====================================================================================================================
void main(void)
{
	// ===================================================
	// Initialize global variables
	// ===================================================


  Tstop  = 3.0;  // set this for however long you want experiment to run

  ControlIteration = 0.0;   // don't mess with
  Time             = 0.0;
  SamplePeriod     = 1.0/CONTROL_FREQ;
  Ts = SamplePeriod;
  LogIndex      = 0;
  LogIndexFinal = (Tstop)*LOG_FREQ-1;
  LogIteration  = 0.0;
  Ko = .7289;
  // initialize your variables
  j = 0;
  Theta  = 0;
  Kenc = (2*pi)/4096;
  Ktach = 1.07;
   // ===================================================
	// Initialize Rabbit SBC
	// ===================================================
	ES308_Init();															       		    // Initialize Rabbit SBC
	EncoderQuadSet(1,4);											       		    // Set quadrature multiplier on Encoder 1 to 4
	DIO_SendByte(0);																		// reset Interrupt
	Send_DAC_Volts(0.0,0.0);														/// Set all output voltages to zero
   Error[0] = 0;
   MotorVolts[0]=0;
   Error[1]=0;
   MotorVolts[1]=0;

	// ===================================================
	// Start Contol code (regulated by "Timer B")
	// ===================================================
	TimerBInit(CONTROL_FREQ);	// Start Timer
	// it calls control routine every 1/CONTROL_FREQ seconds

	// ===================================================
	// Wait Until Tstop
	// ===================================================
	while(Time<=Tstop)
	{
	//remember control code being executed by timerB
	}

	// ===================================================
	// Stop Control code and set all voltages to zero
	// ===================================================
	TimerBUninit();
	DIO_SendByte(0);



   Send_DAC_Volts(0.0,0.0);

	//=====================================================
	// Now print logged data to a file
	//=====================================================
  for(LogIndex=0; LogIndex<LogIndexFinal; LogIndex++)
  {

     printf("%8.3f  %10.3f  %10.3f  %10.3f \n",  LogTime[LogIndex] , LogOmega[LogIndex], LogError[LogIndex], LogMotorVolts[LogIndex]);
  }



}
//========== END MAIN ===========


// ====================================================================================================================
// ControlRoutine()   THIS IS WHERE THE MAGIC HAPPENS
// ====================================================================================================================
nodebug root interrupt void ControlRoutine(void)
{
	int j,k;
	// don't change unless you like pain-------------------------------------------
	RdPortI(TBCSR);					 	// Reset interrupt
	WrPortI(TBL1R,NULL,0);  	// Reload LSB of compare register
	WrPortI(TBM1R,NULL,0);  	// Reload MSB of compare register
	ipres();								// Allow for additional interrupts
   // -------------------------------------------------------



	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  	// put your control calculations here
  	Theta = Kenc * EncoderReadData24(1);
   Omega = Ktach * Get_AD_Volts(0);



  	//send voltages to motor
   if (Time < 1.0) {
				DesOmega = 0.0;
			}  else{
  				DesOmega = 5;
  			};

      Error[0]= DesOmega - Omega;
      MotorVolts[0] = MotorVolts[1]+.9112*Error[0] - 0.5467*Error[1];  //This is your control law!
		Send_DAC_Volts( MotorVolts[0], 0.0 );  // Nothing will happen until you send to motor
      MotorVolts[1] = MotorVolts[0];
		Error[1]= Error[0];

;

  	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


	// LOG data
	if((Time<Tstop)&&(LogIndex<LogIndexFinal))
   {
    if(LogIteration >= CONTROL_FREQ/LOG_FREQ)
    {
		//  list any variables to log but always include time
		LogTime[LogIndex] = Time;
		LogTheta[LogIndex] = Theta;
      LogOmega[LogIndex] = Omega;
		LogIndex++;
		LogIteration = 0;
    }
    LogIteration++;
  }
LogError[LogIndex] = Error[0];
LogMotorVolts [LogIndex] = MotorVolts[0];

	ControlIteration ++;
		if (ControlIteration >= CONTROL_FREQ/2.0)
	{
		ControlIteration = 0.0;								// Start Counting Over
	}

	  Time = Time+SamplePeriod;
}  // ========= END CONTROL ROUTINE ===============