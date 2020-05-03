#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "oled.h"
#include "key.h"
#include "myiic.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "adc.h"



//ALIENTEK 探索者STM32F407开发板 实验12
//OLED显示实验-库函数版本 
//技术支持：www.openedv.com
//淘宝店铺：http://eboard.taobao.com  
//广州市星翼电子科技有限公司  
//作者：正点原子 @ALIENTEK


//void senior(void);
//void caidan(void);
//void adchj(void);




// void adchj(void)
// {
// 	u16 adcx;
// 	u8 key;           //保存键值
// 	u16 temp;
// 	OLED_Clear();
// 	while(1) 
// 	{		 
// 		key = KEY_Scan(0);
// 		if(key==KEY2_PRES)
// 		{
// 			senior();
// 		}
// 		OLED_ShowCHinese(0,0,25);
// 		OLED_ShowCHinese(16,0,26);
// 		OLED_ShowCHinese(32,0,24);
// 		OLED_ShowString(48,0,":");
// 		adcx=Get_Adc_Average(ADC_Channel_5,20);//获取通道5的转换值，20次取平均
// 		adcx=adcx/40;
// 		OLED_ShowNum(58,0,adcx,3,20);    //显示ADCC采样后的原始值
// 		OLED_ShowString(96,0,"d");
// 		OLED_ShowString(104,0,"B");
		
// 		OLED_ShowCHinese(0,2,27);
// 		OLED_ShowCHinese(16,2,28);
// 		OLED_ShowCHinese(32,2,29);
// 		OLED_ShowCHinese(48,2,30);
// 		OLED_ShowString(64,2,":");
// 		temp=Get_Adc_Average(ADC_Channel_4,20);//获取通道5的转换值，20次取平均
// 		temp=temp;
// 		OLED_ShowNum(48,4,temp,4,20);    //显示ADCC采样后的原始值
// 		OLED_ShowString(96,4,"L");
// 		OLED_ShowString(104,4,"u");
// 		OLED_ShowString(112,4,"x");
		
// 		OLED_ShowString(0,6,"key3:");
// 		OLED_ShowCHinese(44,6,11);
// 		OLED_ShowCHinese(60,6,12);
// 	}
// }

// void caidan(void)
// {
// 	OLED_Clear();
// 	OLED_ShowString(0,0,"key1:");
// 	OLED_ShowString(0,2,"6050");
// 	OLED_ShowCHinese(44,2,17);
// 	OLED_ShowCHinese(60,2,18);
// 	OLED_ShowCHinese(76,2,19);
// 	OLED_ShowCHinese(92,2,20);
// 	OLED_ShowCHinese(108,2,21);
	
// 	OLED_ShowString(0,4,"key2:");
// 	OLED_ShowCHinese(0,6,17);
// 	OLED_ShowCHinese(16,6,18);
// 	OLED_ShowCHinese(32,6,22);
// 	OLED_ShowCHinese(48,6,23);
// 	OLED_ShowCHinese(64,6,24);
// }



int main(void)
{ 

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);     //初始化延时函数

	uart_init(115200);	//初始化串口波特率为115200
	LED_Init();					//初始化LED
 	OLED_Init();				//初始化OLED
	KEY_Init();
	usart3_init(384200);  //初始化串口3波特率为115200 Gps串口通信
	usmart_dev.init(168);		//初始化USMART

	while(1)
	{
		//senior();
		OLED_ShowCHinese(92,2,20);
	}
		
}
