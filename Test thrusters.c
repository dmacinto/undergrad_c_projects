#define COUTBUFSIZE 127
#define CINBUFSIZE  127

void servo_Init(void)
{
	// set up PWM outputs
	pwm_set(0, 0, 0);                            // Start with no pulse (0.0/18.0)*1024
	pwm_set(1, 0, 0);
	pwm_set(2, 0, 0);
	pwm_set(3, 0, 0);
}

void MsDelay(int MS)   // Millisecond delay
{
    long SavTimer, TimerDiff;
    TimerDiff = 0;
    SavTimer = MS_TIMER;
    while(TimerDiff < MS) {TimerDiff = MS_TIMER - SavTimer;}
}

void main(void)
{
  int  i, dc;
  char TxSentence[200];

  serCopen(9600);                              // Set up Rabbit serial Port for TSIP 9600 8O1
  serCwrFlush;
  serCrdFlush;
  serCparity(PARAM_NOPARITY);
  MsDelay(100);

  // If you are using the PWM ports to generate RC signals
  servo_Init();
  pwm_init(56);	                               // 56 (Hz) fo Hobby servo (will select the closest possible value),

  // 1024 counter steps for 18 msec.
  // 1.0 msec = 57  counter steps.
  // 1.5 msec = 85  counter steps.
  // 2.0 msec = 114 counter steps.

  pwm_set(3,85,0);

  // Clear out TxSentence
  for(i=0;i<200;i++)
    TxSentence[i] = 0;

  // Initialize SV203 board
  serCputs("BD1\r\n");
  MsDelay(1000);

  // Cycle thruster in forward direction
  /*for(i=0;i<10;i++)
  {
    dc = 128+i;
    sprintf(TxSentence,"BD1 SV6 M%d\r\n",dc);
    printf("TxSentence = ",TxSentence,"\r\n");

    serCputs(TxSentence);

    MsDelay(500);
  }
  */

  while(1)
  {
  sprintf(TxSentence,"BD1 SV2 M128\n\r");
  printf(TxSentence);
  serCputs(TxSentence);
  MsDelay(1000);

  //sprintf(TxSentence,"BD1 SV6 M120\n\r");
  //serCputs(TxSentence);
  //printf(TxSentence);
  //MsDelay(1000);

  }


} // end of main