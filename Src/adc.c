#include "main.h"
#include "string.h"


//ADC_ChannelConfTypeDef sConfig;


ulong adc_bat_average;
ulong adc_bat_sum;
byte adc_bat_num;
uint bat_vol;



void adc_ctrl()
{
	adc_bat_average=0;
	adc_bat_sum=0;
	adc_bat_num=100;
	bat_vol=0;	
	
}



void ADC1_INT () 
{
	
        HAL_ADC_Start(&hTHERM_ADC_Handle);
	

	switch(sConfig.Channel)
	{
		case ADC_CHANNEL_10:
			if(adc_bat_num--)
			{
                                HAL_ADC_PollForConversion(&hTHERM_ADC_Handle,1000);
				adc_bat_sum+=HAL_ADC_GetValue(&hTHERM_ADC_Handle);;
			}else
			{
				adc_bat_average=adc_bat_sum/100;
				adc_bat_sum=0;
				adc_bat_num=100;
				bat_vol=(uint)(adc_bat_average*0.8057);//mV	
			}

		break;
		default:
		   	sConfig.Channel=ADC_CHANNEL_10;
		break;		
	}

	HAL_ADC_Stop(&hTHERM_ADC_Handle);
        

}