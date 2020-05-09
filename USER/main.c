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

//ALIENTEK ̽����STM32F407������ ʵ��12
//OLED��ʾʵ��-�⺯���汾
//����֧�֣�www.openedv.com
//�Ա����̣�http://eboard.taobao.com
//������������ӿƼ����޹�˾
//���ߣ�����ԭ�� @ALIENTEK

u8 USART1_TX_BUF[USART3_MAX_RECV_LEN]; //����1,���ͻ�����
nmea_msg gpsx;						   //GPS��Ϣ
__align(4) u8 dtbuf[50];			   //��ӡ������

u8 key = 0XFF;
	u16 i, rxlen;
	u8 lock = 3;
	u8 unlock = 2;
	u8 temperature;  	    
	u8 humidity; 

void Gps_Msg_Show(void)
{
	float tp;
	tp = gpsx.longitude;
	sprintf((char *)dtbuf, "%.5f %1c   ", tp /= 100000, gpsx.ewhemi); //�õ������ַ���
	OLED_ShowString(0, 2, dtbuf);
	tp = gpsx.latitude;
	sprintf((char *)dtbuf, "%.5f %1c   ", tp /= 100000, gpsx.nshemi); //�õ�γ���ַ���
	OLED_ShowString(0, 4, dtbuf);
}

void updateTemperature()
{
	DHT11_Read_Data(&temperature,&humidity);		//��ȡ��ʪ��ֵ	
	OLED_ShowNum(0,0,temperature,2,16);
	OLED_ShowNum(30,0,humidity,4,16);
}

void keyListen()
{
	key = KEY_Scan(0);
	if (key == lock)
	{
		LED1 = !LED1;
	}
	if (key == unlock)
	{
		LED1 = !LED1;
	}
}

void updateLocation()
{
	if (USART3_RX_STA & 0X8000) //���յ�һ��������
	{
		rxlen = USART3_RX_STA & 0X7FFF; //�õ����ݳ���
		for (i = 0; i < rxlen; i++)
			USART1_TX_BUF[i] = USART3_RX_BUF[i];
		USART3_RX_STA = 0;						  //������һ�ν���
		USART1_TX_BUF[i] = 0;					  //�Զ���ӽ�����
		GPS_Analysis(&gpsx, (u8 *)USART1_TX_BUF); //�����ַ���
		Gps_Msg_Show();							  //��ʾ��Ϣ
		//if(upload)printf("\r\n%s\r\n",USART1_TX_BUF);//���ͽ��յ������ݵ�����1
	}
}

void initGPS()
{
	if (SkyTra_Cfg_Rate(5) != 0) //���ö�λ��Ϣ�����ٶ�Ϊ5Hz,˳���ж�GPSģ���Ƿ���λ.
	{
		OLED_ShowString(0, 0, "start.....");
		do
		{
			usart3_init(9600);						   //��ʼ������3������Ϊ9600
			SkyTra_Cfg_Prt(3);						   //��������ģ��Ĳ�����Ϊ38400
			usart3_init(38400);						   //��ʼ������3������Ϊ38400
			key = SkyTra_Cfg_Tp(100000);			   //������Ϊ100ms
		} while (SkyTra_Cfg_Rate(5) != 0 && key != 0); //����SkyTraF8-BD�ĸ�������Ϊ5Hz
		OLED_ShowString(0, 0, "GPS OK");
		delay_ms(1000);
		OLED_Clear(); //�����ʾ
	}
}

int main(void)
{
	u32 count = 0;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����ϵͳ�ж����ȼ�����2
	delay_init(168);								//��ʼ����ʱ����
	uart_init(115200); //��ʼ�����ڲ�����Ϊ115200
	LED_Init();		   //��ʼ��LED
	OLED_Init();	   //��ʼ��OLED
	KEY_Init();
	usart3_init(384200);  //��ʼ������3������Ϊ115200 Gps����ͨ��
	usmart_dev.init(168); //��ʼ��USMART
	while(DHT11_Init());
	initGPS();
	//usart2_init(9600);
	
	while (1)
	{
		count++;
		delay_ms(1);
		keyListen();
		if(count>=2000){
			count =0;
			//u2_printf("ddddd");
			LED0 = !LED0;
			updateTemperature();
			updateLocation();
		}
		
	}
}
