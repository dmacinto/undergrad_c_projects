#define COUTBUFSIZE 127
#define CINBUFSIZE  127

#define SPI_SER_B                              // Choose serial port B for SPI bus
#define SPI_CLK_DIVISOR 5                      // Minimal clock divisor
#define mag_deviation   0.0                    // (-10.5) Maryland magnetic deviation (deg)

//#define PI 3.141592653589793

#use "spi.lib"                                 // Contains the SPI functions

// --------------------------------------------------------------------
// Global definitions
// --------------------------------------------------------------------
int   LED_flag;

char Xmeas[1];                                 // Single character arrays for MM3 SPI communication
char Ymeas[1];
char Zmeas[1];

typedef union                                  // Union for MM3 measurements
{
  int intval;
  char bytes[2];
}MM3;

MM3 MMx, MMy, MMz;                             // MM3 measurements

// --------------------------------------
// Functions
// --------------------------------------
xmem nodebug void MsDelay(int MS);
xmem nodebug void InitMicroMag3(void);
float MeasureHeading(void);

// ---------------------------------------------------------------
// main()
// ---------------------------------------------------------------

void main()
{
  int  i, initPass, dc;
  char TxSentence[200];
  float heading, psi;
  float t1, t2, deltaT, psiPrev, psiDot;
  float psiD;
  float ep, epDot;
  float kp, kd, ki;
  float uF, uT, uFmax, uTmax;

  float time;

  // Initialize variables
  initPass = 1;

  Xmeas[0] = 0x51; 	// MM3 command word
  Ymeas[0] = 0x52;
  Zmeas[0] = 0x53;

  for(i=0;i<200;i++)
    TxSentence[i] = '\0';

  heading = 0.0;
  psi     = 0.0;
  time    = 0.0;

  // Initialize hardware
   LED_flag = 0;

  // Serial Port C is Xbee module
  serCopen(9600);
  serCparity(PARAM_NOPARITY);
  MsDelay(100);
  serCputs("BD1\r\n");
  serCputs("BD1 SV1 M128\r\n");
  serCputs("BD1 SV2 M128\r\n");

  //Serial Peripherial Interface
  SPIinit();

  // Initialize MicroMag3 compass
  InitMicroMag3();                             // Initialize the MicroMag3 Compass

  // ON/Status Light (LED) PE7
  BitWrPortI(PEDR, &PEDRShadow,1,7);	         // Start with PE7 (LED) High - OFF
  BitWrPortI(PEFR, &PEFRShadow,0,7);	         // Make PE7 Function as Normal I/O
  BitWrPortI(PEDDR,&PEDDRShadow,1,7);          // Make PE7 an OUTPUT for LED

  // CS2 Line for MPC3208 A2D2 converter PG1
  BitWrPortI(PGDR,&PGDRShadow,1,1);		         // Start with PG1 (CS2 of MCP3208) High
  BitWrPortI(PGFR, &PGFRShadow, 0, 1);	       // Make PG1 Function as Normal I/O
  BitWrPortI(PGDDR, &PGDDRShadow, 1, 1);       // Make PG1 an OUTPUT for CS2 Pin

  // CS1 Line for MPC3208 A2D1 converter PG0
  BitWrPortI(PGDR, &PGDRShadow,1,0);		       // Start with PG0 (CS1 of MCP3208) High
  BitWrPortI(PGFR, &PGFRShadow,0,0);	         // Make PG0 Function as Normal I/O
  BitWrPortI(PGDDR,&PGDDRShadow,1,0);          // Make PG0 an OUTPUT for CS1 Pin

  while(1)
  {
    // Measure heading
    heading = MeasureHeading();
    psi     = heading*PI/180.0;

    if(initPass)
    {
      psiPrev = psi;
      psiDot  = 0.0;
      t1      = MS_TIMER;
    }
    else
    {
      t2      = MS_TIMER;
      deltaT  = (t2-t1)/1000.0;
      time    = time+deltaT;

      if(psi-psiPrev>2.0*PI)
        {psiPrev = psiPrev+2.0*PI;}

      if(psi-psiPrev<-2.0*PI)
        {psiPrev = psiPrev-2.0*PI;}

      // Backwards difference calculation of yaw rate
      psiDot  = (psi-psiPrev)/deltaT;

      // Age variables
      psiPrev = psi;
      t1      = t2;
    }

    // Desired heading
    psiD = 270.0*PI/180.0; // Due East

    // Heading tracking error
    ep    = psiD-psi;

    // Account for heading wrap around
    if(ep>PI)
      {ep = -2.0*PI+ep;}
    if(ep<-PI)
      {ep =  2.0*PI+ep;}

    // Yaw rate tracking error
    epDot = 0.0-psiDot;

    // Thrust commands
    uF = 2.0;                                  // Surge thrust command

    kp = 10.0;
    kd = 5.0;
    ki = 0.0;

    uT = kp*ep+kd*epDot;                      // Yaw thrust command

    // Saturate thrust commands
    uFmax = 20.0;
    uTmax = 20.0;

    if(uF>uFmax)
      {uF = uFmax;}
    if(uF<-uFmax)
      {uF = -uFmax;}

    if(uT>uTmax)
      {uT = uTmax;}
    if(uT<-uTmax)
      {uT = -uTmax;}

    // Generate sentence to be sent to SV203
    dc = 128-uF/uFmax*128;
    sprintf(TxSentence,"BD1 SV1 M%d\n\r",dc);
    serCputs(TxSentence);
    //printf("uF = %s",TxSentence);

    dc = 128-uT/uTmax*128;
    sprintf(TxSentence,"BD1 SV2 M%d\n\r",dc);
    serCputs(TxSentence);
    //printf("uT = %s",TxSentence);

    // End of control routine
    initPass = 0;

    // printf commands
    printf("heading = %.1f (deg), ep = %.1f (deg).\n",heading,ep*180.0/PI);
    //printf("time    = %.2f (sec)\n",time);

    //if(time>1.0)
    //  printf("deltaT = %.3f (sec)",deltaT);

    // -----------------------------------------------
    // LED status indicator costatement
    // -----------------------------------------------
    costate
    {
      if(LED_flag)
      {
        BitWrPortI(PEDR,&PEDRShadow,1,7);    	 // Turn Off LED
        waitfor(DelayMs(500));
        LED_flag = 0;
      }
      else
      {
        BitWrPortI(PEDR,&PEDRShadow,0,7);    	 // Turn On LED
        waitfor(DelayMs(500));
        LED_flag = 1;
      }
    } // End of LED costatement
    // MsDelay(500);
  }
}

// ---------------------------------------
// MsDelay
// ---------------------------------------
xmem nodebug void MsDelay(int MS)   			  // Millisecond delay
{
  long SavTimer, TimerDiff;
  TimerDiff = 0;
  SavTimer  = MS_TIMER;
  while(TimerDiff < MS) {TimerDiff = MS_TIMER - SavTimer;}
}

// ---------------------------------------
// InitMircoMag3
// ---------------------------------------
xmem nodebug void InitMicroMag3(void)
{
  BitWrPortI(PBDDR,&PBDDRShadow, 0, 6);   // Make PB6 an INPUT for DRDY Pin

  BitWrPortI(PBDR,&PBDRShadow,1,5);       // Start with PB5 (SSNOT of MicroMag3) High
  BitWrPortI(PBDDR,&PBDDRShadow, 1, 5);   // Make PB5 an OUTPUT for SSNOT Pin

  BitWrPortI(PBDR,&PBDRShadow,0,7);       // Start with PB7 (RESET of MicroMag3) LOW
  BitWrPortI(PBDDR,&PBDDRShadow, 1, 7);   // Make PB7 an OUTPUT for RESET Pin
}

// ---------------------------------------
// MeasureHeading
// ---------------------------------------

float MeasureHeading(void)
{
  char MM3result[2];
  float MMxOut, MMyOut;
  float headingM;

  // Initialize variables
  MMxOut   = 0.0;
  MMyOut   = 0.0;

  // Read the X axis of the MicroMag3
  BitWrPortI(PBDR,&PBDRShadow,0,5);          // Take SSNOT Pin LOW

  BitWrPortI(PBDR,&PBDRShadow,0,7);          // Take PB7 (RESET of MicroMag3) LOW
  BitWrPortI(PBDR,&PBDRShadow,1,7);          // Take PB7 (RESET of MicroMag3) HIGH
  BitWrPortI(PBDR,&PBDRShadow,0,7);          // Take PB7 (RESET of MicroMag3) LOW again

  SPIWrite(Xmeas, 1);                        // 1 byte transfer to MicroMag3

  BitWrPortI(PBDR,&PBDRShadow,1,5);          // Take SSNOT Pin HIGH

  // waitfor(BitRdPortI(PBDR, 6));          // Wait for DRDY to go HIGH
  while(!BitRdPortI(PBDR,6));

  BitWrPortI(PBDR,&PBDRShadow,0,5);          // Take SSNOT Pin LOW

  SPIRead(MM3result, 2);                     // 1 byte transfer from MicroMag3
  BitWrPortI(PBDR,&PBDRShadow,1,5);          // Take SSNOT Pin HIGH

  MMx.bytes[1] = MM3result[0];
  MMx.bytes[0] = MM3result[1];

  // Read the Y axis of the MicroMag3
  BitWrPortI(PBDR,&PBDRShadow,0,5);          // Take SSNOT Pin LOW

  BitWrPortI(PBDR,&PBDRShadow,1,7);          // Take PB7 (RESET of MicroMag3) HIGH
  BitWrPortI(PBDR,&PBDRShadow,0,7);          // Take PB7 (RESET of MicroMag3) LOW again

  SPIWrite(Ymeas, 1);                        // 1 byte transfer to MicroMag3

  BitWrPortI(PBDR,&PBDRShadow,1,5);          // Take SSNOT Pin HIGH

  // waitfor(BitRdPortI(PBDR, 6));           // Wait for DRDY to go HIGH
  while(!BitRdPortI(PBDR,6));

  BitWrPortI(PBDR,&PBDRShadow,0,5);          // Take SSNOT Pin LOW

  SPIRead(MM3result, 2);                     // 1 byte transfer from MicroMag3
  BitWrPortI(PBDR,&PBDRShadow,1,5);          // Take SSNOT Pin HIGH

  MMy.bytes[1] = MM3result[0];
  MMy.bytes[0] = MM3result[1];

  // Read the Z axis of the MicroMag3
  BitWrPortI(PBDR,&PBDRShadow,0,5);          // Take SSNOT Pin LOW

  BitWrPortI(PBDR,&PBDRShadow,1,7);          // Take PB7 (RESET of MicroMag3) HIGH
  BitWrPortI(PBDR,&PBDRShadow,0,7);          // Take PB7 (RESET of MicroMag3) LOW again

  SPIWrite(Zmeas, 1);                        // 1 byte transfer to MicroMag3

  BitWrPortI(PBDR,&PBDRShadow,1,5);          // Take SSNOT Pin HIGH

  // waitfor(BitRdPortI(PBDR, 6));           // Wait for DRDY to go HIGH
  while(!BitRdPortI(PBDR,6));

  BitWrPortI(PBDR,&PBDRShadow,0,5);          // Take SSNOT Pin LOW

  SPIRead(MM3result, 2);                     // 1 byte transfer from MicroMag3
  BitWrPortI(PBDR,&PBDRShadow,1,5);          // Take SSNOT Pin HIGH

  MMz.bytes[1] = MM3result[0];
  MMz.bytes[0] = MM3result[1];

  // Calculate heading from magnetic field measurements
  MMxOut = (float)MMx.intval-25;
  MMyOut = (float)MMy.intval-11;

  headingM = atan(MMyOut/MMxOut);

  if((MMxOut < 0.0) && (MMyOut < 0.0))  // 2 PI ambiguity
    headingM -= PI;
  if((MMxOut < 0.0) && (MMyOut >= 0.0))
    headingM += PI;

  headingM *= (180.0/PI); //Change radians to degrees

  headingM = 180.0-headingM;

  if(headingM < 0.0)
    headingM += 360.00;

  // printf("%.3f,%.3f\n",MMxOut,MMyOut);
  return headingM;
}