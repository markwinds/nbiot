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
u8 lonData[100]; // ����	    
u8 latData[100]; // γ��	 
u8 humidity; 
u8 nbiotMsg[200];
u8 binaryData[500];
u8 charData[60];
u8 binaryDataSize = 0;
u8 UDPReply[50];

u8 strLen(u8 *str)
{
    u8 i = 0;      
    while(str[i++]!='\0');
    return i-1;
}

void binary2Char(u8* str){
	u16 index = 0;
	while (*str!='\0'){
		u8 high = *str;
		u8 low = 0;
		str++;
		low = *str;
		str++;
		if(high>='0' && high<='9'){
			high = (high-'0')*16;
		}else{
			high = (high-'A'+10)*16;
		}
		if(low>='0' && low<='9'){
			low = (low-'0');
		}else{
			low = (high-'A'+10);
		}
		charData[index++] = high+low;
	}
	charData[index] = '\0';
}

void char2Binary(u8* str){
	u16 index = 0;
	while (*str!='\0')
	{
		u8 c = *str;
		u8 high = c/16;
		u8 low = c%16;
		if(high>9){
			binaryData[index++] = (high-10)+'A';
		}else{
			binaryData[index++] = high + '0';
		}
		if(low>9){
			binaryData[index++] = (low-10)+'A';
		}else{
			binaryData[index++] = low + '0';
		}
		str++;
	}
	binaryDataSize = index;
	binaryData[index] = '\0';
}

void sendDataByNbiot(){
	u8 data[200];
	sprintf((char *)data,"13#%d#%s#%s",temperature,lonData,latData);
	char2Binary(data);
	sprintf((char *)data,"AT+NMGS=%d,%s\r\n",binaryDataSize/2,binaryData);
	u2_printf((char*)data);
}

void Gps_Msg_Show(void)
{
	float tp;
	tp = gpsx.longitude;
	sprintf((char *)lonData, "%.5f%1c", tp /= 100000, gpsx.ewhemi); //�õ������ַ���
	OLED_ShowString(0, 2, lonData);
	tp = gpsx.latitude;
	sprintf((char *)latData, "%.5f%1c", tp /= 100000, gpsx.nshemi); //�õ�γ���ַ���
	OLED_ShowString(0, 4, latData);
}

void updateTemperature()
{
	irqCount+=DHT11_Read_Data(&temperature,&humidity);		//��ȡ��ʪ��ֵ	
	OLED_ShowNum(0,0,temperature,2,16);
	OLED_ShowNum(30,0,humidity,4,16);
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
		OLED_ShowString(0, 0, "Start GPS.....");
		do
		{
			usart3_init(9600);						   //��ʼ������3������Ϊ9600
			SkyTra_Cfg_Prt(3);						   //��������ģ��Ĳ�����Ϊ38400
			usart3_init(38400);						   //��ʼ������3������Ϊ38400
			key = SkyTra_Cfg_Tp(100000);			   //������Ϊ100ms
		} while (SkyTra_Cfg_Rate(5) != 0 && key != 0); //����SkyTraF8-BD�ĸ�������Ϊ5Hz
		OLED_Clear(); //�����ʾ
		OLED_ShowString(0, 0, "GPS OK");
		delay_ms(1000);
		OLED_Clear(); //�����ʾ
	}
}

void initTemperature(){
	OLED_ShowString(0, 0, "Start TP.....");
	while(DHT11_Init()){
		delay_ms(400);
	}
	delay_ms(500);
	OLED_Clear(); //�����ʾ
	OLED_ShowString(0, 0, "TP OK");
	delay_ms(1000);
	OLED_Clear(); //�����ʾ
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
		OLED_ShowString(0, 6, "                ");
		OLED_ShowString(0, 6, nbiotMsg);
		return 1;
	}else{
		return 0;
	}
}

void sendUDP(u8* str){
	u8 temp[50];
	u8 replyFlag = 0;
	u8 size = 0;
	u8 i = 0;
	u8 index = 0;
	char2Binary(str);
	readUart2();
	sprintf((char *)temp,"AT+NSOST=1,47.97.195.152,8800,%d,%s\r\n",binaryDataSize/2,binaryData);
	u2_printf((char*)temp);
	while (!replyFlag){
		if(readUart2()){
			if (nbiotMsg[1]=='N'){
				replyFlag = 1;
			}
		}
	}
	replyFlag = 0;
	size = nbiotMsg[strLen(nbiotMsg)-1];
	sprintf((char *)temp,"AT+NSORF=1,%c\r\n",size);
	u2_printf((char*)temp);
	replyFlag = 0;
	while (!replyFlag){
		if(readUart2()){
			if (nbiotMsg[0]=='1'){
				replyFlag = 1;
			}
		}
	}
	for(i=23;i<23+(size-'0')*2;i++){
		UDPReply[index++] = nbiotMsg[i];
	}
	UDPReply[index] = '\0';
	binary2Char(UDPReply);
	OLED_ShowString(0, 4, charData);
}

void keyListen()
{
	key = KEY_Scan(0);
	if (key == lock)
	{
		LED1 = !LED1;
		sendUDP("123");
		LED1 = !LED1;
	}
	if (key == unlock)
	{
		LED1 = !LED1;
		sendUDP("123");
		LED1 = !LED1;
	}
}

void initNbiot(){
	u2_printf("AT+NSOCR=DGRAM,17,4589,1\r\n");
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
	initTemperature();
	initGPS();
	usart2_init(9600);
	initNbiot();
	// binary2Char("303132");
	// OLED_ShowString(0, 4, charData);
	
	while (1)
	{
		// count++;
		// delay_ms(1);
		 keyListen();
		readUart2();
		// if(count>=5000){
		// 	count =0;
		// 	LED0 = !LED0;
		// 	updateTemperature();
		// 	updateLocation();
		// 	sendDataByNbiot();
		// 	OLED_ShowNum(30,6,humidity,4,16);
		// }
	}
}
