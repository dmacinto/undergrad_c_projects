#define FOUTBUFSIZE 127
#define FINBUFSIZE  127
void MsDelay(int MS)   // Millisecond delay
{
    long SavTimer, TimerDiff;
    TimerDiff = 0;
    SavTimer = MS_TIMER;
    while(TimerDiff < MS) {TimerDiff = MS_TIMER - SavTimer;}
}

void main(void)
{
  int   i;
  char  RecStr[100];

  // Open serial ports
  serFopen(9600);            //Set up Rabbit serial Port for TSIP 9600 8O1
  serFwrFlush;
  serFrdFlush;
  serFparity(PARAM_NOPARITY);
  MsDelay(100);

  while(1)
  {
    // serFputs("H\n");
    // serFgets(RecStr,10,200)


    MsDelay(1000);
    costate
    {
      wfd cof_serFgets(RecStr, 100, 200);
      printf(RecStr);

      // Flush read buffer
      serFrdFlush;
    }
  }
}