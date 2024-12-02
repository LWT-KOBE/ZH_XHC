#include "application.h"
#include "usartx.h"
#include "board.h"
#include "State.h"
#include "remote.h"
CAN2_TaskStruct_t CAN2Data;



CAN2_TaskStruct_t* getCAN2_Task(){
    return &CAN2Data;
}

/* ���ó�ʼ�� */
void CAN2_TaskGlobalInit(void){
	
	IO_Init();
//	BSP_CAN_Mode_Init(&can1,CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,12,CAN_Mode_Normal,Preemption,Sub);
	driver_can1_init(CAN1,BSP_GPIOB8,BSP_GPIOB9,4,0);//405 
	Can_Send_Config(8);//53--0
	uart2_init(115200);
	uart3_init(115200);
	uart4_init(115200);
	DMA_Config();
	time2_init(99,839);
//	time2_init(99,599);
	MyADC_Init();
	EEPROM_GPIO_Init();
	EXTI_Configuration();
	Can_Send_Config(8);//53--0
//	EEPROM_Write_Byte(19, 3);
		
//	delay_ms(10);
	
	ReadE2promData();
	
	///241009
	WorkOffFlag = EEPROM_Read_Byte(20);
//	if(WorkOffFlag == 0x55)
//	{
//		WPS = 0;//д�����ر�
//		EEPROM_Write_Byte(20,0xaa);
//		WPS = 1;		
//	}
	if(WorkOffFlag == 0xaa)
	{
		TrainState = ST10;
		StepState = 7;
		InStationCount = 1;
		LD_Step = 0;
	}
	///241009
	
//	TrainHeadNum = 1; //��ȡ��ͷ��
	LedNumDisplay = TrainHeadNum;
	
	#if !DriveMode //��������
		TIM14_PWM_Init(8399,0);	//10K  ����Ƶ��PWMƵ��=84000000/8400=10Khz 
	#endif	
	 
	
	TM1620_Config();
	TM1620_init();		
	Can_Send_Config(8);//53--0
	Remote_Init();//������ճ�ʼ�� 
	//��ʼ��CAN2 ������125K
	//CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,24,CAN_Mode_Normal);
	//��ʼ��CAN2 ������250K
	//CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,12,CAN_Mode_Normal);
	//��ʼ��CAN2 ������1M
	//CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
}

void CAN2_TaskUpdateTask(void *Parameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	digitalLo(&getCAN2_Task()->dataInitFlag);
	
	while(true){
		vTaskDelayUntil(&xLastWakeTime,CAN2_Task_NORMAL_PERIOD);
        //��ֹ�ظ���ʼ��
		if(!CAN2Data.dataInitFlag){	
            //���п���ȫ����ʼ��            
			CAN2_TaskGlobalInit();																																							
			digitalHi(&getCAN2_Task()->dataInitFlag);
			
		}
		Remote_Keyvalue = Remote_Scan();
		for(u8 i = 0;i<TrainBasketMaxNum;i++)
		{
			gCheckHeartLiveCount[i]++;
			if(gCheckHeartLiveCount[i] >=800)//0.8S�������߼��ͳ�βλ�ü��
			{
				BasketLifeFlag &= ~(1<<i);
				gCheckHeartLiveCount[i] = 800;
			}
			
		}		
		
//���ϱ�Ӧ���־		
		for(u8 i =0;i < 21;i++)
		{
			DeliverResultTimer[i]++;
			if(DeliverResultTimer[i] >=1800 && DeliverResultTimer[i] <1802) 
				DeliverResultFlag &= ~(1 << i);//����ϱ�״̬//241017ע��
			else if(DeliverResultTimer[i] >=2000)
				DeliverResultTimer[i] = 2000;
		}
//���ϱ�Ӧ���־	
		
//�����ϱ���־λ����
		if(PocketCount == 1)//��ֹδ�ظ�һֱ��
		{
			DeliverResul2APPtBuff = 0;
			
		}
//�����ϱ���־λ����
		
		//////////////////
		TrainContral();//���� 
		//////////////////
		Display();
		
		KeyScan();
		
/*  �𳵹����뾯��   */		
		if(FrontCarPositionFlag == 0)
			TrainWarning |= 0x01;//ǰ��λ�ö�ʧ
		else
			TrainWarning &= ~0x01;//�������
		
		if(MBSpeed != 0)
		{
			if(gSpeedRA <30)
				TrainWarning |= 0x02;//���1�쳣
			else
				TrainWarning &= ~0x02;//������1�쳣
			
			if(gSpeedRB <30)
				TrainWarning |= 0x04;//���2�쳣
			else
				TrainWarning &= ~0x04;//������2�쳣	
			
			if(gSpeedRC <30)
				TrainWarning |= 0x08;//���3�쳣
			else
				TrainWarning &= ~0x08;//������3�쳣	
			
			if(gSpeedRC <30)
				TrainWarning |= 0x10;//���4�쳣
			else
				TrainWarning &= ~0x10;//������4�쳣				
				
		}
		CanSendOutStationDelay++;
		if(CanSendOutStationDelay >= 1000 && CanSendOutStationDelay <1002)
			CanSendOutStationFlag = 0;
		else if(CanSendOutStationDelay >=1002)
		{
			CanSendOutStationDelay = 1002;
		}
		if(gSpeedR >30)
		{
			SensorWarningDelay1++;
			SensorWarningDelay2++;
			SensorWarningDelay3++;
			SensorWarningDelay4++;
			SensorWarningDelay5++;
			SensorWarningDelay6++;
			SensorWarningDelay7++;
			SensorWarningDelay8++;
/*		SensorWarning		1			*/			
			if(Q_LD == 0)
			{
				if(SensorWarningDelay1 >=2000)
				{
					SensorWarning |= 0x01;//LD1�쳣
					SensorWarningDelay1 = 2000;
				}	
				else
					SensorWarning &= ~0x01;//���LD1�쳣
			}
			else
				SensorWarningDelay1 = 0;
			
/*		SensorWarning		2			*/			
			if(H_LD == 0)
			{
				if(SensorWarningDelay2 >=2000)
				{
					SensorWarning |= 0x02;//LD2�쳣
					SensorWarningDelay2 = 2000;
				}	
				else
					SensorWarning &= ~0x02;//���LD2�쳣
			}
			else
				SensorWarningDelay2 = 0;	

/*		SensorWarning		3			*/			
			if(Q_LD_B == 0)
			{
				if(SensorWarningDelay3 >=2000)
				{
					SensorWarning |= 0x04;//LD1B�쳣
					SensorWarningDelay3 = 2000;
				}	
				else
					SensorWarning &= ~0x04;//���LD1B�쳣
			}
			else
				SensorWarningDelay3 = 0;	

/*		SensorWarning		4			*/			
			if(H_LD_B == 0)
			{
				if(SensorWarningDelay4 >=2000)
				{
					SensorWarning |= 0x08;//LD2B�쳣
					SensorWarningDelay4 = 2000;
				}
				else
					SensorWarning &= ~0x08;//���LD1B�쳣
			}
			else
				SensorWarningDelay4 = 0;	

/*		SensorWarning		5			*/			
			if(Q_GDSW == 0)
			{
				if(SensorWarningDelay5 >=2000)
				{
					SensorWarning |= 0x10;//ST1�쳣
					SensorWarningDelay5 = 2000;
				}	
				else
					SensorWarning &= ~0x10;//���ST1�쳣
			}
			else
				SensorWarningDelay5 = 0;	

/*		SensorWarning		6			*/			
			if(H_GDSW == 0)
			{
				if(SensorWarningDelay6 >=2000)
				{
					SensorWarning |= 0x20;//ST2�쳣
					SensorWarningDelay6 = 2000;
				}
				else
					SensorWarning &= ~0x20;//���ST2�쳣
			}
			else
				SensorWarningDelay6 = 0;	

/*		SensorWarning		7			*/			
			if(Q_GDSW_B == 0)
			{
				if(SensorWarningDelay7 >=2000)
				{
					SensorWarning |= 0x40;//ST1B�쳣
					SensorWarningDelay7 = 2000;
				}
				else
					SensorWarning &= ~0x40;//���ST1B�쳣
			}
			else
				SensorWarningDelay7 = 0;	

/*		SensorWarning		8			*/			
			if(H_GDSW_B == 0)
			{
				if(SensorWarningDelay8 >=2000)
				{
					SensorWarning |= 0x80;//ST2B�쳣
					SensorWarningDelay8 = 2000;
				}
				else
					SensorWarning &= ~0x80;//���ST2B�쳣
			}
			else
				SensorWarningDelay8 = 0;			
			
		}
 
/*  �𳵹����뾯��   */	
	
		#if !DriveMode //��������
			if((TrainWarning&0x1e) == 0x1e)
				TrainStop |= 0x02;
			else
				TrainStop &= ~0x02;
		
		#else
			if(OdriveLifeFlag == 0)//Odriveʧ��
			{
				TrainStop |= 0x02;
			}
			else
				TrainStop &= ~0x02;
		#endif
/*  ͣ��ԭ����   */		
		if(MF_SW == 0)
			TrainStop |= 0x01;
		else
			TrainStop &= ~0x01;
		
		if(MBSpeed > 0 && (TrainStop&0x02) != 0x02)
			TrainStop &= ~0xfe;
		

//		if(CageNumber == 10 && LDDelayFlag == 0)
//		{
//			TrainStop |= 0x08;
//		}
		
		
/*  ͣ��ԭ����   */
		
		digitalIncreasing(&getCAN2_Task()->loops);        

	}
}





//     Frame
// nodeID | CMD
// 6 bits | 5 bits
CanRxMsg can1_rx_msg;
u32 rxbuf3;
void CAN1_RX0_IRQHandler(void){
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET){
		// ����жϱ�־�ͱ�־λ
		CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
		CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
		
		// �ӽ��� FIFO �ж�ȡ��Ϣ		
		CAN_Receive(CAN1, CAN_FIFO0, &can1_rx_msg);
		IAP_APP_CAN_ReStart(can1_rx_msg); 
		SaveData(can1_rx_msg);	
		CheckCarCanCmd();


		
		digitalIncreasing(&OdriveData.OdError.errorCount);

		/*********�������Զ��岿��**********/
		


	}
}


/*
***************************************************
��������CAN1_TX_IRQHandler
���ܣ�CAN1�����ж�
��ע��
***************************************************
*/
void CAN1_TX_IRQHandler(void){
	if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET){
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);

		/*********�������Զ��岿��**********/
    OdriveSend_RemoteCmd(CAN1,AXIS0_ID,MSG_GET_MOTOR_ERROR);
        
	}
}




void CAN2DataInit(void){
	getsupervisorData()->taskEvent[CAN2_Task] = xTaskCreate(CAN2_TaskUpdateTask,"CAN2_Task",CAN2_Task_STACK_SIZE,NULL,CAN2_Task_PRIORITY,&CAN2Data.xHandleTask);
    //usbVCP_Printf("ControlInit Successfully \r\n");
    
}
