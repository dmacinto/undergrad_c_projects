#use "ES308_SBC.LIB"


float x;

main()
{
ES308_Init();


while(1)
	{
   x=Get_AD_Volts(3);
   printf("Volts: %5.1f\r",x);
  }


}