#define FOUTBUFSIZE 127
#define FINBUFSIZE  127

void main(void)
{
  int   i;
  char  RecStr[100];
  char  desHeadingStr[4];
  char  kpStr[4];
  char  kdStr[4];
  char  kiStr[4];
  float desHeading, kp, kd, ki;

  serFopen(4800);            //Set up Rabbit serial Port for TSIP 9600 8O1
  serFwrFlush;
  serFrdFlush;
  serFparity(PARAM_NOPARITY);
  MsDelay(100);

  // Initialize variables
  desHeading = 0.0;
  kp         = 0.0;
  kd         = 0.0;
  ki         = 0.0;

  for(i=0;i<4;i++)
  {
    desHeadingStr[i] = '';
    kpStr[i]         = '';
    kdStr[i]         = '';
    kiStr[i]         = '';
  }

  costate
  {
    wfd cof_serEgets(RecStr, 100, 200);

    // Agreed upon protocol
    // desiredHeading,kp,kd,ki
    // xxxx,xxxx,xxxx,xxxx

    if(strlen(RecStr)==15 && RecStr[4]==',' && RecStr[9]==','' && RecStr[14]==',')
      desHeadingStr[0] = RecStr[0];
      desHeadingStr[1] = RecStr[1];
      desHeadingStr[2] = RecStr[2];
      desHeadingStr[3] = RecStr[3];

    	// RecStr[4] is a ','

	    kpStr[0] = RecStr[5];
	    kpStr[1] = RecStr[6];
	    kpStr[2] = RecStr[7];
	    kpStr[3] = RecStr[8];

	    // RecStr[9] is a ','

	    kdStr[0] = RecStr[10];
	    kdStr[1] = RecStr[11];
	    kdStr[2] = RecStr[12];
	    kdStr[3] = RecStr[13];

	    // RecStr[14] is a ','

	    kiStr[0] = RecStr[15];
	    kiStr[1] = RecStr[16];
	    kiStr[2] = RecStr[17];
	    kiStr[3] = RecStr[18];

      // Convert to floating point values
      desHeadig = atof(desHeadingStr);
      kp        = atof(kpStr);
      kd        = atof(kdStr);
      ki        = atof(kiStr);

      // Flush read buffer
      serFrdFlush;
    }
    else
    {
      serFputs('Invalid command.\r\n');

      // Flush write buffer
      serFwrFlush;
    }

    printf("desHeading = %f (deg), kp = %f, kd = %f, ki = %f.\n",desHeading,kp,kd,ki);
  }


} // end of main