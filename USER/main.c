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
#include "usmart.h"
#include "usart2.h"
#include "usart3.h"
#include "gps.h"
#include "dht11.h"
#include "timer.h"
#include "ds18b20.h"
#include <string.h>

//ALIENTEK 探索者STM32F407开发板 实验12
//OLED显示实验-库函数版本
//技术支持：www.openedv.com
//淘宝店铺：http://eboard.taobao.com
//广州市星翼电子科技有限公司
//作者：正点原子 @ALIENTEK

u8 USART1_TX_BUF[USART3_MAX_RECV_LEN]; //串口1,发送缓存区
nmea_msg gpsx;						   //GPS信息
__align(4) u8 dtbuf[50];			   //打印缓存器

u8 key = 0XFF;
u16 i, rxlen;
u8 lock = 3;
u8 unlock = 2;
u8 temperature;  
u8 lonData[100]; // 经度	    
u8 latData[100]; // 纬度	 
u8 humidity; 
u8 nbiotMsg[200];
u8 binaryData[500];
u8 charData[60];
u8 binaryDataSize = 0;
u8 UDPReply[50];
u8 boxId[]="15";
u8 locked = 0;
u8 tempData[50];

void sendDataByNbiot(){
	u8 data[200];
	sprintf((char *)data,"%s#%d#%s#%s",boxId,temperature,lonData,latData);
	u2_printf((char*)data);
}

void Gps_Msg_Show(void)
{
	float tp;
	tp = gpsx.longitude;
	sprintf((char *)lonData, "%.5f%1c", tp /= 100000, gpsx.ewhemi); //得到经度字符串
	OLED_ShowString(0, 2, lonData);
	tp = gpsx.latitude;
	sprintf((char *)latData, "%.5f%1c", tp /= 100000, gpsx.nshemi); //得到纬度字符串
	OLED_ShowString(0, 4, latData);
}

void updateTemperature()
{
	short temp = DS18B20_Get_Temp();		//读取温湿度值	
	temperature = temp/10;
	OLED_ShowNum(0,0,temperature,2,16);
	OLED_ShowString(20, 0, ".");
	OLED_ShowNum(30,0,temp%10,1,16);
	if(temperature>70){
		temperature = 0;
	}
}

void updateLocation()
{
	if (USART3_RX_STA & 0X8000) //接收到一次数据了
	{
		rxlen = USART3_RX_STA & 0X7FFF; //得到数据长度
		for (i = 0; i < rxlen; i++)
			USART1_TX_BUF[i] = USART3_RX_BUF[i];
		USART3_RX_STA = 0;						  //启动下一次接收
		USART1_TX_BUF[i] = 0;					  //自动添加结束符
		GPS_Analysis(&gpsx, (u8 *)USART1_TX_BUF); //分析字符串
		Gps_Msg_Show();							  //显示信息
		//if(upload)printf("\r\n%s\r\n",USART1_TX_BUF);//发送接收到的数据到串口1
	}
}

void initGPS()
{
	if (SkyTra_Cfg_Rate(5) != 0) //设置定位信息更新速度为5Hz,顺便判断GPS模块是否在位.
	{
		OLED_ShowString(0, 0, "Start GPS.....");
		do
		{
			usart3_init(9600);						   //初始化串口3波特率为9600
			SkyTra_Cfg_Prt(3);						   //重新设置模块的波特率为38400
			usart3_init(38400);						   //初始化串口3波特率为38400
			key = SkyTra_Cfg_Tp(100000);			   //脉冲宽度为100ms
		} while (SkyTra_Cfg_Rate(5) != 0 && key != 0); //配置SkyTraF8-BD的更新速率为5Hz
		OLED_Clear(); //清除显示
		OLED_ShowString(0, 0, "GPS OK");
		delay_ms(1000);
		OLED_Clear(); //清除显示
	}
}

void initTemperature(){
	OLED_ShowString(0, 0, "Start TP.....");
	while(DS18B20_Init()){
		delay_ms(400);
	}
	delay_ms(500);
	OLED_Clear(); //清除显示
	OLED_ShowString(0, 0, "TP OK");
	delay_ms(1000);
	OLED_Clear(); //清除显示
}

u8 readUart2(){
	u16 len =0;
	u16 t = 0;
	u16 m = 0;
	if(USART2_RX_STA&0x8000)
	{					   
		len=USART2_RX_STA&0x3fff;
		USART2_RX_STA=0;
		for(t=0;t<len;t++)
		{
			if (USART2_RX_BUF[t] != '\r' && USART2_RX_BUF[t] != '\n')
			{
				nbiotMsg[m++] = USART2_RX_BUF[t];
			}
		}
		if (m==0)
		{
			return 1;
		}
		nbiotMsg[m] = '\0';
		// OLED_ShowString(0, 6, "                ");
		// OLED_ShowString(0, 6, nbiotMsg);
		return 1;
	}else{
		return 0;
	}
}

void sendUDP(char* str){
	u2_printf(str);
	i = 0;
	while (!readUart2()){
		i++;
		delay_ms(100);
		if (i>60)
		{
			i=0;
			u2_printf(str);
		}
		
	};
}

void showReply(){
	OLED_ShowString(0, 6, "                ");
	OLED_ShowString(0, 6, nbiotMsg);
}

u8 getUDPRes(char* str){
	u8 temp[50];
	sprintf((char *)temp,"%s#%s",str,boxId);
	sendUDP((char*)temp);
	if (strcmp((char*)nbiotMsg,"true")==0)
	{
		return 1;
	}else{
		return 0;
	}
}

void keyListen()
{
	key = KEY_Scan(0);
	if (key == lock)
	{
		LED1 = !LED1;
		if (getUDPRes("@LockEnable"))
		{
			OLED_ShowString(0, 6, "                ");
			OLED_ShowString(0, 6, "Wait bind");
			for (i = 15; i >0; i--)
			{
				OLED_ShowNum(110,6,i,2,16);
				delay_ms(1000);
			}
			if (getUDPRes("@CheckLock")){
				OLED_ShowString(0, 6, "                ");
				OLED_ShowString(0, 6, "Bind OK");
				locked = 1;
			}else{
				OLED_ShowString(0, 6, "                ");
				OLED_ShowString(0, 6, "No phone bind");
				locked = 0;
			}
		}else{
			OLED_ShowString(0, 6, "                ");
			OLED_ShowString(0, 6, "Has been used");
		}
		LED1 = !LED1;
	}
	if (key == unlock)
	{
		LED1 = !LED1;
		if (getUDPRes("@UnlockEnable"))
		{
			OLED_ShowString(0, 6, "                ");
			OLED_ShowString(0, 6, "Wait confirm");
			for (i = 15; i >0; i--)
			{
				OLED_ShowNum(110,6,i,2,16);
				delay_ms(1000);
			}
			if (getUDPRes("@CheckUnlock")){
				OLED_ShowString(0, 6, "                ");
				OLED_ShowString(0, 6, "Receive OK");
				locked = 1;
			}else{
				OLED_ShowString(0, 6, "                ");
				OLED_ShowString(0, 6, "No phone receive");
				locked = 0;
			}
		}else{
			OLED_ShowString(0, 6, "                ");
			OLED_ShowString(0, 6, "No package");
		}
		LED1 = !LED1;
	}
}

void initNbiot(){
	OLED_ShowString(0, 0, "Start NB.....");
	while (strcmp((char*)nbiotMsg,"Connected")!=0)
	{
		readUart2();
	}
	sprintf((char *)tempData,"!%s",boxId);
	sendUDP((char*)tempData);
	if (strcmp((char*)nbiotMsg,"true")==0){
		locked = 1;
	}else{
		locked = 0;
	}
	OLED_Clear(); //清除显示
	OLED_ShowString(0, 0, "NB OK");
	delay_ms(1000);
	OLED_Clear(); //清除显示
}

void showLockStatus(){
	OLED_ShowString(60, 0, locked?"locked":"unlocked");
}

int main(void)
{
	u32 count = 0;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置系统中断优先级分组2
	delay_init(168);								//初始化延时函数
	uart_init(115200); //初始化串口波特率为115200
	usart2_init(9600);
	LED_Init();		   //初始化LED
	OLED_Init();	   //初始化OLED
	KEY_Init();
	usart3_init(384200);  //初始化串口3波特率为115200 Gps串口通信
	usmart_dev.init(168); //初始化USMART
	initTemperature();
	initGPS();
	initNbiot();
	
	while (1)
	{
		count++;
		delay_ms(1);
		keyListen();
		showLockStatus();
		if(count>=5000){
			count =0;
			LED0 = !LED0;
			updateTemperature();
			updateLocation();
			sendDataByNbiot();
		}
	}
}
