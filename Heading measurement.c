#define DOUTBUFSIZE 127
#define DINBUFSIZE  127
#define SPI_SER_B                              // Choose serial port B for SPI bus
#define SPI_CLK_DIVISOR 5                      // Minimal clock divisor
#define mag_deviation 0.0                      // (-10.5) Maryland magnetic deviation (deg)

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

float heading; // (deg) - magnetic heading measurement
float uSurge;  // (N)   - surge force command
float uTurn;   // (Nm)  - yaw torque command

// --------------------------------------
// Functions
// --------------------------------------
xmem nodebug void MsDelay(int MS);
xmem nodebug void InitMicroMag3(void);

// ---------------------------------------------------------------
// main()
// ---------------------------------------------------------------

void main()
{
  int  i;

  char MM3result[2];
  char TX_sentence[200];

  float MMxOut, MMyOut;

  // Initialize variables
  Xmeas[0] = 0x51; 	// MM3 command word
  Ymeas[0] = 0x52;
  Zmeas[0] = 0x53;

  MMxOut   = 0.0;
  MMyOut   = 0.0;

  for(i=0;i<200;i++)
    TX_sentence[i] = '\0';

  // Initialize hardware
   LED_flag = 0;

  // Serial Port D is Xbee module
  serDopen(4800);
  serDparity(PARAM_NOPARITY);
  MsDelay(100);

  //Serial Peripherial Interface (Syscronous serial bus)
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

    // ----------------------------------------------------------------------
    // Calculate heading
    // ----------------------------------------------------------------------

    //Read the X axis of the MicroMag3
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

    BitWrPortI(PBDR,&PBDRShadow,1,5);    			 // Take SSNOT Pin HIGH

    // waitfor(BitRdPortI(PBDR, 6));     			 // Wait for DRDY to go HIGH
    while(!BitRdPortI(PBDR,6));

    BitWrPortI(PBDR,&PBDRShadow,0,5);    			 // Take SSNOT Pin LOW

    SPIRead(MM3result, 2);               			 // 1 byte transfer from MicroMag3
    BitWrPortI(PBDR,&PBDRShadow,1,5);    			 // Take SSNOT Pin HIGH

    MMz.bytes[1] = MM3result[0];
    MMz.bytes[0] = MM3result[1];

    // Calculate heading from magnetic field measurements
    MMxOut = (float)MMx.intval;
    MMyOut = (float)MMy.intval;

		heading = atan((float)MMy.intval / (float)MMx.intval);

    if(((float)MMx.intval < 0.0) && ((float)MMy.intval < 0.0))  // 2 PI ambiguity
      heading -= PI;
    if(((float)MMx.intval < 0.0) && ((float)MMy.intval >= 0.0))
      heading += PI;

    heading *= (180.0/PI); //Change radians to degrees

    heading = heading-180.0;

    if(heading < 0.0)
      heading += 360.00;

     printf("Heading = %f (deg). \n",heading);

    // Create output string to send to serial port D
    // sprintf(TX_sentence, "%.1f,%.1f\r",MMxOut,MMyOut);
    // serDputs(TX_sentence);

    // Printing to screen for debugging
    // printf("Heading = %.1f (deg). MMx = %.1f. MMy = %.1f\n",heading,MMxOut,MMyOut);

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
    MsDelay(500);
  }   // while(1)
}     // main()

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