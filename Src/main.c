/*----------------------------------------------------------------------------
 * Copyright (c) <2016-2018>, <Huawei Technologies Co., Ltd>
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this list of
 * conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list
 * of conditions and the following disclaimer in the documentation and/or other materials
 * provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific prior written
 * permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *---------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------
 * Notice of Export Control Law
 * ===============================================
 * Huawei LiteOS may be subject to applicable export control laws and regulations, which might
 * include those applicable to Huawei LiteOS of U.S. and the country in which you are located.
 * Import, export and usage of Huawei LiteOS in any manner by you shall be in compliance with such
 * applicable export control laws and regulations.
 *---------------------------------------------------------------------------*/

#include "sys_init.h"

/****** chose a sensor for your application*****/

//#define MODULE_DHT11
//#define MODULE_SMOKE
//#define MODULE_GPS
//#define MODULE_BH1750
#define MODULE_MYAPP
#define PRINTALL 1

uint8_t aRxBuffer[1];

uint8_t flag=0;

int datalen=0;
int datalen_tmp=0;
int rxlen=0;
int rxlen_uart_temp=0;

uint8_t state=0;
uint8_t laststate=0;
int stateCounter=0;

#define NEUL_MAX_BUF_SIZE 3000
char Recv_Data_Buf[NEUL_MAX_BUF_SIZE] = {0};
char Recv_Uart_Buf[NEUL_MAX_BUF_SIZE] = {0};
char Recv_UDP_Buf[NEUL_MAX_BUF_SIZE] = {0};
char Recv_UDP_Buf_temp[NEUL_MAX_BUF_SIZE] = {0};
char Recv_Uart_Buf_temp[NEUL_MAX_BUF_SIZE] = {0};
char testbuf[NEUL_MAX_BUF_SIZE] = {0xF4,0xF4,0x00,0x04,0x41,0x42,0x43,0x44};
/*互斥锁句柄ID*/
static UINT32 g_Testmux01;


/* event control struct */
static EVENT_CB_S  udp_event;

/* wait event type */
#define event_recv 0x00000001

static UINT32 g_uwQueue;





void printbuf(char* buf, int len);
void resetData();
UINT32 g_TskHandle;

msg_sys_type bc95_net_data;
static char nbiot_uart[500];

#ifdef	MODULE_DHT11
DHT11_Data_TypeDef  DHT11_Data;
msg_for_DHT11       DHT11_send;
#endif

#ifdef	MODULE_SMOKE
msg_for_SMOKE       SMOKE_send;
#endif

#ifdef	MODULE_GPS
gps_msg             gpsmsg;
msg_for_GPS         GPS_send;
float Longitude;
float Latitude;
static unsigned char gps_uart[1000];
#endif

#ifdef	MODULE_BH1750
msg_for_BH1750      BH1750_send;
#endif


VOID HardWare_Init(VOID)
{
	HAL_Init();
	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	DelayInit();	
	MX_GPIO_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	printf("Welcome to td-tech, This is nb-iot Board.\r\n");
}

void user_hw_init(void)
{
	char *str = "TD-Tech good";
	OLED_Init();
	DHT11_Init();	
	OLED_Clear();
	//OLED_ShowCHinese(0,0,0);
	//OLED_ShowCHinese(18,0,1);
	//OLED_ShowCHinese(36,0,2);
	//OLED_ShowCHinese(54,0,3);
	//OLED_ShowCHinese(72,0,4);
	//OLED_ShowCHinese(90,0,5);
	OLED_ShowString(0,2,(uint8_t*)str,16);
	LOS_HwiCreate(EXTI0_IRQn, 2,0,EXTI0_IRQHandler,NULL);
	LOS_HwiCreate(EXTI1_IRQn, 3,0,EXTI1_IRQHandler,NULL);
	LOS_HwiCreate(EXTI2_IRQn, 4,0,EXTI2_IRQHandler,NULL);
	LOS_HwiCreate(EXTI3_IRQn, 5,0,EXTI3_IRQHandler,NULL);
	LOS_HwiCreate(USART1_IRQn, 6,0,USART1_IRQHandler,NULL);
	LOS_HwiCreate(USART2_IRQn, 6,0,USART2_IRQHandler,NULL);
	LOS_HwiCreate(USART3_IRQn, 7,0,USART3_IRQHandler,NULL);
	los_dev_uart_init(LOS_STM32L431_UART2, 9600, nbiot_uart, 500);
}

VOID data_collection_task(VOID)
{
	UINT32 uwRet = LOS_OK;
#ifdef	MODULE_SMOKE		
	short int Value;
	short int MaxValue=1000;
	SMOKE_send.index=1;
#endif
	
#ifdef	MODULE_GPS		

#endif

#ifdef	MODULE_BH1750
	short int Lux;
	Init_BH1750();
#endif
	user_hw_init();

	while (1)
  {
#ifdef	MODULE_DHT11
	  /****************temperature and humidity*****************/
    if(DHT11_Read_TempAndHumidity(&DHT11_Data)==SUCCESS)
    {
			printf("读取DHT11成功!-->湿度为%.1f ％RH ，温度为 %.1f℃ \n",DHT11_Data.humidity,DHT11_Data.temperature);
    }
    else
    {
      printf("读取DHT11信息失败\n");
			DHT11_Init();      
    }
		sprintf(DHT11_send.temp, "%.1f", DHT11_Data.temperature);
		sprintf(DHT11_send.hum, "%.1f", DHT11_Data.humidity);
#endif

#ifdef	MODULE_SMOKE		
		/****************smoke******************/
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 50);
		Value = HAL_ADC_GetValue(&hadc1);
		printf("\r\n******************************MQ2 Value is  %d\r\n",Value);
		sprintf(SMOKE_send.Value, "%4d", Value);
		sprintf(SMOKE_send.MaxValue, "%4d", MaxValue);
#endif

#ifdef	MODULE_GPS		
		/****************GPS******************/
	  HAL_UART_Receive_IT(&huart3,gps_uart,1000);
		NMEA_BDS_GPRMC_Analysis(&gpsmsg,(uint8_t*)gps_uart);	//分析字符串
		Longitude=(float)((float)gpsmsg.longitude_bd/100000);	
		printf("Longitude:%.5f %lc     \r\n",Longitude,gpsmsg.ewhemi_bd);
		Latitude=(float)((float)gpsmsg.latitude_bd/100000);
		printf("Latitude:%.5f %1c   \r\n",Latitude,gpsmsg.nshemi_bd);	
  
#endif

#ifdef	MODULE_BH1750
		/****************BH1750******************/
		Lux=(int)Convert_BH1750();
	  printf("\r\n******************************BH1750 Value is  %d\r\n",Lux);
    sprintf(BH1750_send.Lux, "%5d", Lux);	
#endif

		uwRet=LOS_TaskDelay(1500);
		if(uwRet !=LOS_OK)
		return;
	
  }
}
UINT32 creat_data_collection_task()
{
    UINT32 uwRet = LOS_OK;
    TSK_INIT_PARAM_S task_init_param;

    task_init_param.usTaskPrio = 0;
    task_init_param.pcName = "data_collection_task";
    task_init_param.pfnTaskEntry = (TSK_ENTRY_FUNC)data_collection_task;
    task_init_param.uwStackSize = 0x800;

    uwRet = LOS_TaskCreate(&g_TskHandle, &task_init_param);
    if(LOS_OK != uwRet)
    {
        return uwRet;
    }
    return uwRet;
}


VOID data_report_task(VOID)
{
	UINT32 uwRet = LOS_OK;
	neul_bc95_reboot();                  													 //初始化模块
	while(neul_bc95_get_netstat()<0);                              //等待连接上网络
#ifdef	MODULE_MYAPP
	printf("AT_COM_NSOCL begin:\r\n");	
	//CLUB_AT_QUERY( AT_COM_NSOCL );
	//neul_bc95_close_udpsocket(1);
	printf("AT_COM_NSOCL end\r\n");
	
	printf("AT_COM_NSOCR begin:\r\n");	
	//CLUB_AT_QUERY( AT_COM_NSOCR );
	neul_bc95_create_udpsocket(8888);
	neul_bc95_socket_config_remoteinfo(1, "120.55.96.180", 5000);
	printf("AT_COM_NSOCR end\r\n");
#endif		
	//neul_bc95_set_cdpserver("180.101.147.115");                  //连接电信平台
	//neul_bc95_set_cdpserver("218.4.33.71");                        //连接华为平台
	while(1)
	{
		printf("app task test\r\n");
#ifdef	MODULE_MYAPP
		switch(flag)
		{
			case 1:
			{
				printf("case 1\r\n");
				//CLUB_AT_Custom( AT_COM_Custom_CSQ );
				//CLUB_AT_Custom( AT_COM_Custom_TIME );					
	   			//printf("CSQ and Time\r\n");
	   			//printf("CSQ:");
	   			//printf("%s\r\n",bc95_net_data.net_csq);
	   			//printf("Time:");
	   			//printf("%s\r\n",bc95_net_data.net_time);		
				
				break;
			}
			case 2:
			{
				printf("case 2\r\n");
				break;
			}
			case 3:
			{
				printf("case 3\r\n");
				printf("AT_COM_NSOST begin:\r\n");	
				//CLUB_AT_QUERY( AT_COM_NSOST );	
				char *buf = "AABB";
				neul_bc95_udp_send(1, buf, strlen(buf));
        			printf("AT_COM_NSOST end\r\n");	
				
				//HAL_Delay(5000);
				UINT32 uwTick = LOS_MS2Tick(5000);//
				neul_bc95_sleep(uwTick);
				
				printf("AT_COM_Custom_NSORF begin:\r\n");	
			  	//CLUB_AT_Custom( AT_COM_Custom_NSORF ); 
				neul_bc95_udp_read(1,Recv_Data_Buf, NEUL_MAX_BUF_SIZE,0);
				printf("AT_COM_Custom_NSORF end\r\n");	
				printbuf(Recv_Data_Buf,strlen(Recv_Data_Buf));
				//bc95_hexstr_to_byte(bc95_net_data.net_nsorf, bc95_net_data.net_nsorf_str, 100);
				printf("rec udp:\r\n");
				printf("net_nsorf:%s\r\n",Recv_Data_Buf);	
			  	//printf("net_nsorf_str:%s\r\n",bc95_net_data.net_nsorf_str);	
				
				//CLUB_AT_Custom( AT_COM_Custom_TIME );
				//printf("Time:");
				//printf("%s\r\n",bc95_net_data.net_time);	
				break;
			}
			case 4:
			{
				printf("case 4\r\n");
				//CLUB_AT_QUERY( AT_COM_NCDP );
				//CLUB_AT_QUERY( AT_COM_NMGS );							
				break;
			}
			default:
				break;
		}		
#endif		
		
#ifdef	MODULE_DHT11
/*******************temperature and humidity*****************/
		if(neul_bc95_send_coap_paylaod((const char*)(&DHT11_send),sizeof(DHT11_send))>=0)		//发送数据到平台	
				printf("ocean_send_data OK!\n");                                                //发生成功
		else                                                                                //发送失败
			{
				printf("ocean_send_data Fail!\n"); 
			}
		uart_data_flush();
		memset(bc95_net_data.net_nmgr, 0, 5);
		neul_bc95_read_coap_msg(bc95_net_data.net_nmgr,5);
		printf("%s\n",bc95_net_data.net_nmgr);
		if(strcmp(bc95_net_data.net_nmgr,"ON")==0) //开灯
			{	
					HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);    // 输出高电平
			}
		if(strcmp(bc95_net_data.net_nmgr,"OFF")==0) //关灯
			{	
					HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);  // 输出低电平
			}
/*******************************END**********************************************/
#endif
			
#ifdef	MODULE_SMOKE	
/***************************smoke*********************************/
		SMOKE_send.CSQ=neul_bc95_get_csq();
		if(neul_bc95_send_coap_paylaod((const char*)(&SMOKE_send),sizeof(SMOKE_send))>=0)		//发送数据到平台	
			printf("ocean_send_data OK!\n");                                                  //发生成功
		else                                                                                //发送失败
			{
			printf("ocean_send_data Fail!\n"); 
			}
		uart_data_flush();
		memset(bc95_net_data.net_nmgr, 0, 5);
		neul_bc95_read_coap_msg(bc95_net_data.net_nmgr,5);
		printf("%s\n",bc95_net_data.net_nmgr);
		if(strcmp(bc95_net_data.net_nmgr,"ON")==0) //开蜂鸣器
			{	
					HAL_GPIO_WritePin(Beep_GPIO_Port,Beep_Pin,GPIO_PIN_SET);    // 输出高电平
			}
		if(strcmp(bc95_net_data.net_nmgr,"OFF")==0) //关蜂鸣器
			{	
					HAL_GPIO_WritePin(Beep_GPIO_Port,Beep_Pin,GPIO_PIN_RESET);  // 输出低电平
			}
/*******************************END**********************************************/
#endif
			
#ifdef	MODULE_GPS
/***********************************GPS*************************************/
		  if(Latitude!=0&&Longitude!=0)
		{
			memset(GPS_send.Latitude, 0, 8);
			memset(GPS_send.Longitude, 0, 9);
			sprintf(GPS_send.Latitude, "%.5f", Latitude);
			sprintf(GPS_send.Longitude, "%.5f", Longitude);		
			if(neul_bc95_send_coap_paylaod((const char*)(&GPS_send),sizeof(GPS_send))>=0)		//发送数据到平台	
				printf("ocean_send_data OK!\n");                                            //发生成功
		else                                                                            //发送失败
			{
				printf("ocean_send_data Fail!\n"); 
			}
			uart_data_flush();
		memset(bc95_net_data.net_nmgr, 0, 5);
		neul_bc95_read_coap_msg(bc95_net_data.net_nmgr,5);
		printf("%s\n",bc95_net_data.net_nmgr);
		if(strcmp(bc95_net_data.net_nmgr,"ON")==0) //开灯
			{	
					HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);    // 输出高电平
			}
		if(strcmp(bc95_net_data.net_nmgr,"OFF")==0) //关灯
			{	
					HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);  // 输出低电平
			}
		}
/*******************************END**********************************************/
#endif
			
#ifdef	MODULE_BH1750
/**********************************BH1750*************************************/
		if(neul_bc95_send_coap_paylaod((const char*)(&BH1750_send),sizeof(BH1750_send))>=0)		//发送数据到平台	
				printf("ocean_send_data OK!\n");                                                  //发生成功
		else                                                                                  //发送失败
			{
				printf("ocean_send_data Fail!\n"); 
			}
			uart_data_flush();
		memset(bc95_net_data.net_nmgr, 0, 5);
		neul_bc95_read_coap_msg(bc95_net_data.net_nmgr,5);
		printf("%s\n",bc95_net_data.net_nmgr);
		if(strcmp(bc95_net_data.net_nmgr,"ON")==0) //开灯
			{	
					HAL_GPIO_WritePin(Light_GPIO_Port,Light_Pin,GPIO_PIN_RESET);    // 输出低电平
			}
		if(strcmp(bc95_net_data.net_nmgr,"OFF")==0) //关灯
			{	
					HAL_GPIO_WritePin(Light_GPIO_Port,Light_Pin,GPIO_PIN_SET);  // 输出高电平
			}
/*******************************END**********************************************/
#endif 
	  uwRet=LOS_TaskDelay(500);
		if(uwRet !=LOS_OK)
		return;
	}
}
UINT32 creat_data_report_task()
{
    UINT32 uwRet = LOS_OK;
    TSK_INIT_PARAM_S task_init_param;

    task_init_param.usTaskPrio = 1;
    task_init_param.pcName = "data_report_task";
    task_init_param.pfnTaskEntry = (TSK_ENTRY_FUNC)data_report_task;
    task_init_param.uwStackSize = 0x400;
    uwRet = LOS_TaskCreate(&g_TskHandle, &task_init_param);
    if(LOS_OK != uwRet)
    {
        return uwRet;
    }
    return uwRet;
}

UINT32 SndRcvEvent(UINT32 uwEvents)
{
    UINT32 uwRet;


    /* write event */
    printf("write event %d.\n", uwEvents);
    uwRet = LOS_EventWrite(&udp_event, uwEvents);
    if(uwRet != LOS_OK)
    {
        printf("event write failed .\n");
        return LOS_NOK;
    }



    return LOS_OK;
}

UINT32 data_udp_init(VOID)
{
	UINT32 uwRet = LOS_OK;
	user_hw_init();
	HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, 1);	
	HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, 1);	

	neul_bc95_reboot();                  													 //初始化模块
	while(neul_bc95_get_netstat()<0);                              //等待连接上网络

	//printf("AT_COM_NSOCL begin:\r\n");	
	//CLUB_AT_QUERY( AT_COM_NSOCL );
	neul_bc95_close_udpsocket(1);
	//printf("AT_COM_NSOCL end\r\n");
	
	//printf("AT_COM_NSOCR begin:\r\n");	
	//CLUB_AT_QUERY( AT_COM_NSOCR );
	uwRet=neul_bc95_create_udpsocket(8888);
	if(uwRet==-1)
	{
		printf("neul_bc95_create_udpsocket error\r\n");
		return -1;
	}
	printf("neul_bc95_create_udpsocket ok\r\n");
	//uwRet=neul_bc95_socket_config_remoteinfo(1, "120.55.96.180", 5000);
	uwRet=neul_bc95_socket_config_remoteinfo(1, "47.95.197.61", 7000);
	//uwRet=neul_bc95_socket_config_remoteinfo(1, "115.29.240.46", 6000);
	if(uwRet!=LOS_OK)
	{
		printf("neul_bc95_socket_config_remoteinfo error\r\n");
		return -1;
	}
	printf("neul_bc95_socket_config_remoteinfo ok\r\n");
	return LOS_OK;
	//neul_bc95_socket_config_remoteinfo(1, "47.95.197.61", 58883);
	//printf("AT_COM_NSOCR end\r\n");
}

VOID data_udp_send(VOID)
{
	UINT32 uwRet = LOS_OK;


	UINT32 uwReadbuf;
	UINT32 uwMsgCount = 0;

	data_udp_init();
	//if (uwRet != LOS_OK)
	//{
	//    return LOS_NOK;
	//}	

	while (1)
	{
		/* read data from queue to uwReadbuf */
		uwRet = LOS_QueueRead(g_uwQueue, &uwReadbuf, NEUL_MAX_BUF_SIZE,  LOS_WAIT_FOREVER);
		if (uwRet != LOS_OK)
		{
		    printf("recv message failure,error:%x\n",uwRet);
		    continue;
		}

		mytick.sendTick= LOS_TickCountGet();
		uwMsgCount++;
   		int datalen_high = ((char*)uwReadbuf)[0];
		uint8_t datalen_low =((char*)uwReadbuf)[1];
				
		int datalen1 =(datalen_high<<8)|(datalen_low&0xFF);
		
		printf("----->\r\nlen:%d, recv message:%s\n",datalen1, (CHAR *)uwReadbuf);
		//printbuf((CHAR *)uwReadbuf,datalen1+2);
		//printf("\r\n");
		//printf("AT_COM_NSOST begin:\r\n");	
		//CLUB_AT_QUERY( AT_COM_NSOST );	
		//char *buf = "AABB";
		
				    /*申请互斥锁*/
    uwRet=LOS_MuxPend(g_Testmux01, LOS_WAIT_FOREVER);
		
		neul_bc95_udp_send(1, (CHAR *)uwReadbuf+2, datalen1);
		    /*释放互斥锁*/
    LOS_MuxPost(g_Testmux01);
			mytick.sendOkTick= LOS_TickCountGet();
		//printf("AT_COM_NSOST end\r\n");	

		//HAL_Delay(5000);
		//UINT32 uwTick = LOS_MS2Tick(1000);//
		//neul_bc95_sleep(uwTick);
		//SndRcvEvent(event_recv);

		//(VOID)LOS_TaskDelay(5);
	}

	/* delete queue */
	while (LOS_OK != LOS_QueueDelete(g_uwQueue))
	{
	    (VOID)LOS_TaskDelay(1);
	}

}
UINT32 creat_data_udp_send()
{
    UINT32 uwRet = LOS_OK;
    TSK_INIT_PARAM_S task_init_param;

    task_init_param.usTaskPrio = 1;
    task_init_param.pcName = "data_udp_send";
    task_init_param.pfnTaskEntry = (TSK_ENTRY_FUNC)data_udp_send;
    task_init_param.uwStackSize = 0x400;
    uwRet = LOS_TaskCreate(&g_TskHandle, &task_init_param);
    if(LOS_OK != uwRet)
    {
        return uwRet;
    }
    return uwRet;
}


VOID data_udp_recv(VOID)
{
	UINT32 uwEvent;
	UINT32 uwRet = LOS_OK;

	/*
	 * timeout, WAITMODE to read event, timeout is 100 ticks,
	 * if timeout, wake task directly
	 */
	//printf("Example_Event wait event 0x%x \n",event_recv);
	while (1)
	{
		uwEvent = LOS_EventRead(&udp_event, event_recv, LOS_WAITMODE_AND, 300);
		if(uwEvent == event_recv)
		{
			printf("udp_event,read event :0x%x, wait...\r\n",uwEvent);
			
			/* clear event flag */
			//printf("EventMask:%d\n", udp_event.uwEventID);
			LOS_EventClear(&udp_event, ~udp_event.uwEventID);
			//printf("EventMask:%d\n", udp_event.uwEventID);
		}
		else
		{
			//printf("udp_event,read event timeout :0x%x\n-------\r\n",uwEvent);
		}

		if(state>0&& state == laststate)
		{
			stateCounter++;
			if(stateCounter>10){
				printf("error state:%d\r\n",laststate);
				resetData();
			}
		}
		else 
		{
			stateCounter=0;
			laststate=state;
		}
		
		//printf("AT_COM_Custom_NSORF begin:\r\n");	
		//CLUB_AT_Custom( AT_COM_Custom_NSORF ); 
		memset(Recv_UDP_Buf, 0, NEUL_MAX_BUF_SIZE);
		memset(Recv_UDP_Buf_temp, 0, NEUL_MAX_BUF_SIZE);
		/*申请互斥锁*/
		uwRet=LOS_MuxPend(g_Testmux01, LOS_WAIT_FOREVER);

		int len = neul_bc95_udp_read(1,Recv_UDP_Buf, NEUL_MAX_BUF_SIZE,0);
		//int len = neul_bc95_udp_read_restore(1,Recv_UDP_Buf);
		/*释放互斥锁*/
		LOS_MuxPost(g_Testmux01);
		
		//int len =8;
		//memcpy(Recv_UDP_Buf,testbuf,8);
		//printf("AT_COM_Custom_NSORF end\r\n");	
		if(len>0)
		{
			mytick.recvAllTick= LOS_TickCountGet();
			UINT32 total_time = LOS_Tick2MS(mytick.recvAllTick - mytick.sendTick);
			UINT32 send_time = LOS_Tick2MS(mytick.sendOkTick- mytick.sendTick);	
			UINT32 ind_time = LOS_Tick2MS(mytick.recvIndTick - mytick.sendOkTick);	
			UINT32 read_time = LOS_Tick2MS(mytick.recvAllTick- mytick.recvIndTick);	
			printf("rec udp package, len=%d, total_time=%d,send_time=%d, ind_time=%d, read_time=%d\r\n<-----\r\n", len, total_time,send_time,ind_time,read_time);
			//printbuf(Recv_UDP_Buf,len);
			
			uint8_t *plen=(uint8_t *)&len;
			Recv_UDP_Buf_temp[0]=0xF4;
			Recv_UDP_Buf_temp[1]=plen[1];
			Recv_UDP_Buf_temp[2]=plen[0];
			memcpy(Recv_UDP_Buf_temp+3, Recv_UDP_Buf, len);
			Recv_UDP_Buf_temp[3+len]=0xF5;
			if(PRINTALL){
			printbuf(Recv_UDP_Buf_temp,len+4);
			printf("\r\nrec udp string:%s\r\n-------\r\n",Recv_UDP_Buf);	
				}
   		       HAL_UART_Transmit(OUTPUT_UART, (uint8_t *)Recv_UDP_Buf_temp, len+4, 0xFFFF);
   		       
					    
		}
	}
	return;

}

UINT32 creat_data_udp_recv()
{
    UINT32 uwRet = LOS_OK;
    TSK_INIT_PARAM_S task_init_param;

    task_init_param.usTaskPrio = 1;
    task_init_param.pcName = "data_udp_recv";
    task_init_param.pfnTaskEntry = (TSK_ENTRY_FUNC)data_udp_recv;
    task_init_param.uwStackSize = 0x400;
    uwRet = LOS_TaskCreate(&g_TskHandle, &task_init_param);
    if(LOS_OK != uwRet)
    {
        return uwRet;
    }
    return uwRet;
}
int main(void)
{
    UINT32 uwRet = LOS_OK;
	
    HardWare_Init();

    uwRet = LOS_KernelInit();
    if (uwRet != LOS_OK)
    {
        return LOS_NOK;
    }
		
    //uwRet = creat_data_collection_task();
    //if (uwRet != LOS_OK)
    //{
     //   return LOS_NOK;
    //}
		
	//uwRet = creat_data_report_task();
    //if (uwRet != LOS_OK)
    //{
    //    return LOS_NOK;
    //}

    /* event init */
    uwRet = LOS_EventInit(&udp_event);
    if(uwRet != LOS_OK)
    {
        printf("init event failed .\n");
        return LOS_NOK;
    }

	/* create queue */
	uwRet = LOS_QueueCreate("queue", 3, &g_uwQueue, 0, NEUL_MAX_BUF_SIZE);
	if (uwRet != LOS_OK)
	{
	    printf("create queue failure!,error:%x\n",uwRet);
		return LOS_NOK;
	}
/*创建互斥锁*/
    LOS_MuxCreate(&g_Testmux01);

	printf("create the queue success!\n");



	uwRet = creat_data_udp_send();
	if (uwRet != LOS_OK)
	{
	    return LOS_NOK;
	}	

	uwRet = creat_data_udp_recv();
	if (uwRet != LOS_OK)
	{
	    return LOS_NOK;
	}	

	LOS_Start();
}

#ifdef	MODULE_MYAPP
void resetData()
{
	       //printf("\r\nerror:state=%d,rxlen=%d,datalen=%d\r\n",state,rxlen,datalen);
		printbuf(Recv_Uart_Buf,rxlen);
		printf("\r\n");
		memset(Recv_Uart_Buf_temp,0,NEUL_MAX_BUF_SIZE);
		rxlen_uart_temp=0;			
		memset(Recv_Uart_Buf,0,NEUL_MAX_BUF_SIZE);
		rxlen=0;		
		state=0;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//printf("%c",aRxBuffer[0]);
//printf("0x%.2x ",aRxBuffer[0]);
//HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, 1);	
//return;
	if(huart->Instance==USART1||huart->Instance==USART3)                                              
	{
		//printf("%c",aRxBuffer[0]);
		/*
		if(aRxBuffer[0]==0x0d)
		{
			printf("%s\r",Recv_Data_Buf);
			printbuf(Recv_Data_Buf,rxlen);
			rxlen=0;
			memset(Recv_Data_Buf,0,NEUL_MAX_BUF_SIZE);
			
		}
		else
		{
			Recv_Data_Buf[rxlen++]=aRxBuffer[0];
		}
		*/
		uint8_t data=aRxBuffer[0];
		if(rxlen_uart_temp<NEUL_MAX_BUF_SIZE)
		{
			Recv_Uart_Buf_temp[rxlen_uart_temp++]=data;
		}
		
		if(state==0)
		{
			if(data==0xF4){
				state=1;
			}
			
		}
		//else if(state==1&&data==0xF4)
		//{
		//	state=2;
		//}
		else if(state==1)
		{
			state=2;
			datalen_tmp = data;
		}		
		else if(state==2)
		{
			state=3;
			
			rxlen=0;
			memset(Recv_Uart_Buf,0,NEUL_MAX_BUF_SIZE);
			Recv_Uart_Buf[rxlen++]=datalen_tmp;
			//printf("h:%d", datalen_tmp);
			Recv_Uart_Buf[rxlen++]=data;	
			//printf("\r\nl:%d", data);
			datalen =(datalen_tmp<<8)|(data&0xFF);
			//printf("\r\ntotal:%d", datalen);

		}			
		else if(state==3&&datalen >0&&datalen<NEUL_MAX_BUF_SIZE)
		{
			datalen--;
//			if(rxlen==4)
//{printf("ok");}
			Recv_Uart_Buf[rxlen++]=data;
			if(datalen==0)
			{
				//printf("%s\r",Recv_Uart_Buf);
				state=4;
			}
			if(rxlen==6) 
			{
				if((Recv_Uart_Buf[0]!=Recv_Uart_Buf[4])||(Recv_Uart_Buf[1]!=Recv_Uart_Buf[5]))
				{
				       printf("rxlen==6/r/n");
					resetData();
					//printf("error!=\r\n");
				}
			}

			

		}		
		else if(state==4&&data==0xF5)
		{
			//printbuf(Recv_Uart_Buf,rxlen);
			//printf("\r\nuart receive len:%d\r\n",rxlen);
			if(PRINTALL){
			printbuf(Recv_Uart_Buf_temp,rxlen_uart_temp);
			printf("\r\n0x%.2x ",Recv_Uart_Buf[0]);
			printf("0x%.2x ",Recv_Uart_Buf[1]);
			printf("0x%.2x ",Recv_Uart_Buf[4]);
			printf("0x%.2x ",Recv_Uart_Buf[5]);
			printf("\r\nuart receive len:%d\r\n",rxlen_uart_temp);	
			}
			/* write abuf data to queue */
			UINT32 uwRet = 0;
			uwRet = LOS_QueueWrite(g_uwQueue, Recv_Uart_Buf, rxlen, 0);
			if(uwRet != LOS_OK)
			{
				printf("send message failure,error:%x\n",uwRet);
			}
			//resetData();
			memset(Recv_Uart_Buf_temp,0,NEUL_MAX_BUF_SIZE);
			rxlen_uart_temp=0;			
			state=0;			
		}
		else
		{
			printf("else/r/n");
		       resetData();
		}
		if(huart->Instance==USART1)   
		{
			HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, 1);			
		}
		else if(huart->Instance==USART3)   
		{
			HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, 1);		
		}
	}

	
}     


void printbuf(char* buf, int len)
{
	for(int i=0;i<len;i++)
	{
		if(i%16==0)
		{
			printf("\r\n");
		}
		printf("0x%.2x ",buf[i]);

	}
}

#endif 
