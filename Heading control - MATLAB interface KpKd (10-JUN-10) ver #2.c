#define COUTBUFSIZE 127
#define CINBUFSIZE  127

#define FOUTBUFSIZE 127
#define FINBUFSIZE  127

#define SPI_SER_B                              // Choose serial port B for SPI bus
#define SPI_CLK_DIVISOR 5                      // Minimal clock divisor
#define mag_deviation   0.0                    // (-10.5) Maryland magnetic deviation (deg)

//#define PI 3.141592653589793

#use "spi.lib"                                 // Contains the SPI functions

// =====================================================================================================
// Global definitions
// =====================================================================================================
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

// =====================================================================================================
// Functions
// =====================================================================================================
xmem nodebug void MsDelay(int MS);
xmem nodebug void InitMicroMag3(void);
float MeasureHeading(void);

// =====================================================================================================
// main()
// =====================================================================================================
void main()
{
  int  i, dc;
  int  timeInit;

  char RxSentence[30];
  char TxSentence[30];
  char desHeadingStr[4];
  char uFstr[4];
  char enableStr[2];
  char KpStr[4];
  char KdStr[4];
  char initPassStr[2];

  float heading, psi;
  float t1, t2, deltaT, psiPrev, psiDot;
  float psiD;
  float ep, epDot;
  float Kp, Kd, Ki;
  float uF, uT, uFmax, uTmax;
  float desHeading, enable;

  float psiDuf[2], psiDf[2];

  float initPass;

  float time;

  float depInt[2];
  float epInt;
  float uKiMax;

  // Initialize variables
  Xmeas[0] = 0x51; 	// MM3 command word                                                   b
  Ymeas[0] = 0x52;
  Zmeas[0] = 0x53;

  timeInit = 1;

  // Clear transmit/receive sentence
  for(i=0;i<30;i++)
  {
    RxSentence[i] = '\0';
    TxSentence[i] = '\0';
  }

  for(i=0;i<4;i++)
  {
    desHeadingStr[i] = '\0';
    uFstr[i] = '\0';
    KpStr[i] = '\0';
    KdStr[i] = '\0';
  }

  enableStr[0] = '\0';
  enableStr[1] = '\0';

  initPassStr[0] = '\0';
  initPassStr[1] = '\0';

  // Initialize other variables
  heading = 0.0;
  psi     = 0.0;
  time    = 0.0;
  enable  = 0.0;

  psiDuf[1] = 0.0;
  psiDf[1]  = 0.0;

  initPass  = 1.0;

  depInt[0] = 0.0;
  depInt[1] = 0.0;
  epInt     = 0.0;


  // Initial control gain selection
  Kp = 1.0;
  Kd = 2.0;
  Ki = 1.0;

  desHeading = 0.0;

  // Initialize hardware
   LED_flag = 0;

  // Serial Port C is SV203 board
  serCopen(9600);
  serCparity(PARAM_NOPARITY);
  MsDelay(100);

  // Initialize board and zero out thrusters
  serCputs("BD1\r\n");
  serCputs("BD1 SV1 M128\r\n");
  serCputs("BD1 SV2 M128\r\n");

   // Serial Port F communicates with laptop
  serFopen(9600);
  serFparity(PARAM_NOPARITY);
  MsDelay(100);

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

    // Acquire desired heading, surge thrust command, and enable signal from laptop
    costate
    {
      wfd cof_serFgets(RxSentence, 30, 200);
      // printf(RxSentence);

      // hxxx,xxxxf,xe,xxxxp,xxxxd - format of the string from MATLAB
      // the 1st three numbers are the desired heading in (deg) 0<desHeading<360.0
      // the 2nd three numbers are the open-loop surge thrust command. Always less than 10 (N)
      // the 3rd number is the enable. Either 0 or 1.
     // printf(RxSentence);
     // printf("\n");

     if(RxSentence[0]=='h' && RxSentence[4]==',' && RxSentence[9]=='f' && RxSentence[12]=='e')
     {
                      //  RxSentence[0] = 'h'
       desHeadingStr[0] = RxSentence[1];
       desHeadingStr[1] = RxSentence[2];
       desHeadingStr[2] = RxSentence[3];
                      //  RxSentence[4] = ','
       uFstr[0]         = RxSentence[5];
       uFstr[1]         = RxSentence[6];
       uFstr[2]         = RxSentence[7];
       uFstr[3]         = RxSentence[8];
                     //   RxSentence[9] = 'f'
                     //   RxSentence[10] = ','
       enableStr[0]     = RxSentence[11];
                     //   RxSentence[12] = 'e'
                     //   RxSentence[13] = ','
       KpStr[0]         = RxSentence[14];
       KpStr[1]         = RxSentence[15];
       KpStr[2]         = RxSentence[16];
       KpStr[3]         = RxSentence[17];
                     //   RxSentence[18] = 'p'
                     //   RxSentence[19] = ','
       KdStr[0]         = RxSentence[20];
       KdStr[1]         = RxSentence[21];
       KdStr[2]         = RxSentence[22];
       KdStr[3]         = RxSentence[23];
                     //   RxSentence[24] = 'd'
                     //   RxSentence[25] = ','
       initPassStr[0]   = RxSentence[26];
                     //   RxSentence[27] = 'i'

       // Convert to floats
       desHeading = atof(desHeadingStr);
       uF         = atof(uFstr);
       enable     = atof(enableStr);
       Kp         = atof(KpStr);
       Kd         = atof(KdStr);
       initPass   = atof(initPassStr);

       // Clear strings again
       for(i=0;i<4;i++)
       {
         desHeadingStr[i] = '\0';
         uFstr[i] = '\0';
         KpStr[i] = '\0';
         KdStr[i] = '\0';
       }

       enableStr[0] = '\0';

       for(i=0;i<15;i++)
         RxSentence[i] = '\0';
     }
     else
     {
        serFputs("Invalid command.\r\n");
        enable = 0.0;

        for(i=0;i<15;i++)
          RxSentence[i] = '\0';
     }

      // Flush read buffer
      serFrdFlush;
    }

    // =====================================================================
    // Start control calculations here.
    // =====================================================================

    if(initPass)
    {
      psiPrev = psi;
      psiDot  = 0.0;
      t1      = MS_TIMER;

      psiD     = desHeading*PI/180.0;
      psiDuf[0] = psiD;
      psiDuf[1] = psiD;
      psiDf[0]  = psiD;
      psiDf[1]  = psiD;

      // printf("I am here.\n");
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
    psiD = desHeading*PI/180.0;

    // Digital filter on the desired heading.
    // 2 sec. settling time at approx. Ts = 0.032 (sec)
    psiDuf[0] = psiD;
    psiDf[0]  = 0.9773*psiDf[1]+0.011369*(psiDuf[0]+psiDuf[1]);

    // Age variables
    psiDuf[1] = psiDuf[0];
    psiDf[1]  = psiDf[0];

    // Heading tracking error
    ep    = psiD-psi;


    // Account for heading wrap around
    if(ep>PI)
      {ep = -2.0*PI+ep;}
    if(ep<-PI)
      {ep =  2.0*PI+ep;}

    epInt     = epInt+0.5*deltaT*(ep+depInt[1]);
    depInt[1] = ep;

    uKiMax = 5.0;

    if(Ki*epInt>uKiMax)
      epInt = uKiMax/Ki;
    if(Ki*epInt<-uKiMax)
      epInt = -uKiMax/Ki;



    // Yaw rate tracking error
    epDot = 0.0-psiDot;

    // Thrust commands

    uT = Kp*ep+Kd*epDot+Ki*epInt;              // Yaw thrust command

    // Saturate thrust commands
    uFmax = 8.0;
    uTmax = 8.0;

    if(uF>uFmax)
      {uF = uFmax;}
    if(uF<-uFmax)
      {uF = -uFmax;}

    if(uT>uTmax)
      {uT = uTmax;}
    if(uT<-uTmax)
      {uT = -uTmax;}

    // Generate sentence to be sent to SV203
    dc = 125-uF/20.0*128;
    //dc = 125;
    sprintf(TxSentence,"BD1 SV1 M%d\n\r",dc);
    serCputs(TxSentence);
    //printf("uF = %s",TxSentence);

    dc = 128-uT/20.0*128;
    //dc = 128;
    sprintf(TxSentence,"BD1 SV2 M%d\n\r",dc);
    serCputs(TxSentence);
    //printf("uT = %s",TxSentence);

    // printf commands
    printf("desHeading = %.1f (deg), heading = %.1f (deg), initPass = %.1f.\n",desHeading,heading,initPass);
    // printf("uF = %.1f (N), uT = %.1f (N), psiDot = %f.1 (rad/sec).\n",uF,uT,psiDot);
    //printf("Kp = %.1f. Kd = %.1f \n",Kp,Kd);

    //printf("time    = %.2f (sec)\n",time);

    // if(time>1.0 && timeInit)
    //{
    //  printf("deltaT = %.3f (sec)",deltaT);
    //  timeInit = 0;
    //}


    // End of control routine
    initPass = 0;

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
  MMxOut = (float)MMx.intval+200;
  MMyOut = (float)MMy.intval-100;

  headingM = atan(MMyOut/MMxOut);

  if((MMxOut < 0.0) && (MMyOut < 0.0))  // 2 PI ambiguity
    headingM -= PI;
  if((MMxOut < 0.0) && (MMyOut >= 0.0))
    headingM += PI;

  headingM *= (180.0/PI); //Change radians to degrees

  // headingM = 180.0-headingM;
  // printf("heading = %.1f \n",headingM);

  if(headingM < 0.0)
    headingM += 360.00;

  // printf("%.3f,%.3f\n",MMxOut,MMyOut);
  return headingM;
}