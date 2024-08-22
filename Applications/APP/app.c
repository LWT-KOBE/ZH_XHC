#include "app.h"
#include "stdlib.h"
#include <string.h>
#include "application.h"
#include "MyADC.h"  
CanRxMsg RxMessage;
CanTxMsg TxMessage;


u32 CAN_ID = 0;
u8 RxRAM0[8];
u8 Rxflag1 = 0;

u32 PocketCount,PocketCountold = 0;//U�͵�Ƭ����
u32 Pocket_A_Count,OldPocket_A_Count,NewPocket_A_Count = 0;//������Ƭ
u16 LongLDCount = 0;
u16 Timer40msCount = 0;
u16 Timer10msCount = 0;
u16 Timer100msCount = 0; 
u16 gCheckHeartLiveCount = 0;//�������߼��
u16 gCheckTrailLiveCount = 0;
u16 gCheckFroCarLiveCount = 0; 
u16 SwitchCount1 = 0;
u32 SwitchCount2 = 0; 
u8 SwitchCount = 0; 
 
 
 
float gSpeedR = 0;
u8 ReadSpeedFlag = 0;//��������쳣1
u16 ReadSpeedTimerCount = 0;
u8 MotorDir = 0; //
u8 MotorGoFlag = 0;
u32 APPBasketNum = 0x00000000;//����B3  �����B2  Ͷ�ݸ�ں�B0\1

u32 APPSendADDRFlag = 0;//APP�·���ַ��־λ  21��λ����21�������·��ĵ�ַ
u8 APPSendADDRACKFlag = 0;//APP�·���ַ��־λ-�Զ�ģʽ
int AppSendAddr[21];//�·���ַ  ����B3  �����B2  Ͷ�ݸ�ں�B0\1
u32 AppAddrACK = 0;//Ӧ��ظ�

int LastDeliverResultNum[21];//����B1  �����B2  Ͷ�ݸ�ں�B3\4 
unsigned short int LastPoketNum[21];//��һȦ��Ƭ����

u32 BasketApplicationFlag = 0;//21��λ����21�������Ӧ�����ַ״̬    1--�����ַ��־
u32 DeliverResultFlag = 0;//21��λ����21�������Ӧ�ϱ����״̬  1--�ϱ�
u32 DeliverResultACKFlag = 0;//21��λ����21�������Ӧ�ϱ����Ӧ��״̬  1--Ӧ��

u8 BasketError[24];//������ϻ���

u32 NFCNUM,NFCNUMold,NFCNUMnew = 0;//��ͷλ��
u32 TailstockNFC = 0;

u16 CarTailNFCNum = 0;//��βλ��
u8 MoveOneFlag = 0;

u8 HeadSendBasketFlag = 0; // 0x01--�·���ַ  0x02--ֹͣ�·�  0x04--�·��ظ�Ͷ�ݽ��Ӧ���ź�  0x08--ֹͣͶ�ݽ��Ӧ���·�
u32 BasketPocketNum,BasketPocketNumold,BasketPocketNumnew,LastDeliverResult = 0;

u32 BasketReciveAddrFlag = 0;//�����յ���ַ��־λ 21bit����21������--�Զ�ģʽ

u16 BasketNumACKADDRToAPP = 0;

u16 PositionTail = 0;//��Ƭ��βλ��
u16 FrontCarPositionNFC = 0;//ǰ��λ�ã�ָ����ǰ�����һ�ڳ����λ��

u16 testflag = 0;

u16 FrontCarPositionPokec = 0;//ǰ����βλ��
u16 FrontCarPositionPokec1 = 0;//ǰ����βλ��
u16 FrontCarPositionPokec2 = 0;//ǰ����βλ��

u8 FrontCarPositionFlag = 0;//ǰ��λ�û�ȡ�ɹ���־λ
//u8 TrailPositionFlag = 0;//��βλ�����߱�־

u16 FrontCarPositionHead = 0;//ǰ����ͷλ��λ��
u16 CarPositionPocket = 0;//��Ƭ��+�ӽ���Ƭ
u16 CarDistance = 0;  //��ǰ������������
u8 SendDataFlag = 0;//�յ�ǰ���Ĺ㲥���������ݱ�־
u8 ReciveRightFlag = 0;//�������ձ�־λ
u8 TrainHeadNum = 0;   //����
u8 CarSignalNum = 0;//��ʱ�ȴ�����ʱ�����
u16 TrainBasketPosition = 0;//��βλ�ø�ں�

float gPowerValueFloat = 0;
u16 powervol = 0;
unsigned  int gCarBatPower = 0; //��Դ��ѹ
u16 carpower = 0;

u8 ST1ACKFlag = 0;//�Լ�ָ���־λ

u8 TrainApplyForExitFlag = 0;//��վ�����־λ
u8 BreakCargo = 0;
u8 CarGo = 0;//0--ǰ��
u8 CarBack = 1;//����
u8 ScranTrain = 0;//�����־--1
u8 StionStop  = 0;   		//��վͣ����Ƭ

u8 CarDirection = 0;//0-stop 1ǰ��  2����
u16 IDMaxNum = 0;//����ں�
u8 CarScranFlag = 0;
u8 CarDriveFlag = 0;

u8 ApplicationtTavel=9;	//���뷢����Ƭ
u8 ForceTravel1 =10;
u8 ForceTravel2 =11;
u8 ForceTravel3 =29;

u8 ConfigrationFlag = 0;//���ñ�־λ 1--����
u8 InStationFlag = 0;//��վ��־
u8 InStationCount = 0;
u8 InStationCount1 = 0;
u32 BasketLifeFlag = 0;//�������߱�־

u8 ReadDataFlag = 0;
u8 CanSendCoount = 0;

//u8 DriveMode = 1;//����������� 0--�������� 1--ODRIVE

////////////////////////////���ò���///////////////////////////

u16 CaseNum = 77;//�����   ADDR 0-1
u8 Pocket4=11;						//װ�ص�Ƭ ADDR 2
u8 TrainBasketMaxNum = 20;//��������������β��� ADDR 3
u8 TrainMode = 1;//ɨ��ģʽ  //0--�ֶ�ɨ��    1--�Զ�ɨ��  ADDR 4
u8 HigSpeed = 200;//ADDR 5
u8 MidSpeed = 150;//ADDR 6
u8 LowSpeed = 80;//ADDR 7

u8 OutStationQuicken = 4;//��վ���ٸ�� ADDR 8

u8 FollowHigSpeed = 20;//���������� ADDR 9
u8 FollowMidSpeed = 10;//���������� ADDR 10
u8 FollowLowSpeed = 5;//���������� ADDR 11

u8 InStationHigtoMidSpeed = 10;//����װ����ǰ���ټ����� ADDR 12
u8 InStationMidtoLowSpeed = 4;//����װ����ǰ���ټ����� ADDR 13
u8 TrainMaxNum = 3;//������    ADDR 14
u16 RecycleCaseNum = 77;//���ո��  ADDR 15-16

u8 MFContralZS = 1;//���������ں�����  ADDR 17
u8 MFContralDS = 2;//���������ںŵ���  ADDR 18

////////////////////////////���ò���///////////////////////////




void IAP_APP_Init(void)
{
	SCB->VTOR = FLASH_BASE | IAP_Bootloat_SIZE; //IAP_Bootloat_SIZE:0x2000=8K �ֽ�
//	GetLockCode(&g_Lock_Code);						//��ȡоƬΨһID
}


//void GetCarSpeed(void)
//{
//	u8 i = 0;
//	u32 buf,gSum = 0;
//	
//	//����С���ٶ�
//	for(i = 0; i < SPEED_VAULE; i++)
//	{
//		buf += GetSpeedVaule[i];
//	}
//	gSum = buf;
//	gSpeedR = (u32)(60.0 / (1.0 / 1000000 * gSum));
//	buf = 0;
//}



u32 count = 0;
u8 Icount[21];
u8 Icount1[21];
u8 Icount2[21];
u8 i = 0;
void SaveData(CanRxMsg temp_CAN_Msg)
{ 
  u8 i; 
	
	CAN_ID = temp_CAN_Msg.StdId;
	
	if(temp_CAN_Msg.IDE == CAN_Id_Standard)
	{
		for(i = 0; i < 8; i++)
		{
			RxRAM0[i] = temp_CAN_Msg.Data[i];
		}
	}	
		Rxflag1 = SUCCESS;

}

void CheckCarCanCmd(void)
{	
	if(Rxflag1 == SUCCESS)
	{		
		for(i=0;i<21;i++)
		{
			if(CAN_ID == i+1)	
			{			
				BasketError[i] = RxRAM0[3];//������������
				BasketLifeFlag |= 1 << i;
				gCheckHeartLiveCount = 0;//�������߼��ʱ������
			}
			if(TrainMode == 0)//�ֶ�ɨ��
			{
				if((RxRAM0[0] & 0x0f)== 0x01)//�����ַ
				{
					if(CAN_ID == i+1)	
					{
						Icount1[i]++;
						if(Icount1[i] == 1)
						{
							MoveOneFlag = 0x01;
							BasketApplicationFlag |= 1 << i;//�������������״̬
							IntermediateVariable2 = 0;
						}
						if(Icount1[i] >= 0xee)
							Icount1[i] = 0xee;
						
						Icount[i] = 0; 
					}
				}	
				else if((RxRAM0[0]& 0x0f) == 0x02)//�����յ���ַֹͣ�·�
				{
					if(CAN_ID == i+1)	
					{					
						Icount[i]++;
						APPSendADDRFlag &= ~(1<<i);//0808
						if(Icount[i] == 1)
						{
							APPSendADDRFlag &= ~(1<<i);//
							HeadSendBasketFlag &= ~0x03;
							HeadSendBasketFlag |= 0x02;
							BasketApplicationFlag &= ~(0x00000001 << i);//�������״̬
						}
						if(Icount[i] >= 0xee)
							Icount[i] = 0xee;
						Icount1[i] = 0;
					}
				}
			}
			else//�Զ�ɨ��
			{
				if(CAN_ID == i+1)		
				{
					if((RxRAM0[0]& 0x0f) == 0x02)//�����յ���ַֹͣ�·�
					{					
						Icount[i]++;
						APPSendADDRFlag &= ~(1<<i);
						if(Icount[i] == 1)
						{
//							APPSendADDRFlag &= ~(1<<i);
							HeadSendBasketFlag &= ~0x03;
							HeadSendBasketFlag |= 0x02;	
							BasketReciveAddrFlag |= (1<<i);//�����յ���ַ
						}
						if(Icount[i] >= 0xee)
							Icount[i] = 0xee;
					}
					else if((RxRAM0[0]& 0x0f) == 0x03)
					{
						Icount[i] = 0;
						BasketReciveAddrFlag &= ~(1<<i);
					}					
				}
			}
			
			if((RxRAM0[0] & 0xf0)== 0x10)//�յ������ϱ����
			{
				if(CAN_ID == i+1)	
				{	
					Icount2[i]++;
					if(Icount2[i] == 1)
					{					
						LastDeliverResultNum[i] = RxRAM0[1] <<8 | RxRAM0[2];  //���������Ͷ�ݽ��
						LastPoketNum[i] = RxRAM0[4] <<8 | RxRAM0[5];
						DeliverResultFlag |= 1 << i;//����������ϱ�״̬
					}
					if(Icount2[i] >= 0xee)
						Icount2[i] = 0xee;					
					
				}	
			}
			else if((RxRAM0[0]& 0xf0) == 0x20)//�����յ�Ͷ�ݽ��ȷ��Ӧ���ź�
			{
				if(CAN_ID == i+1)	
				{					
					HeadSendBasketFlag &= ~0x0c;
					HeadSendBasketFlag |= 0x08;			
					DeliverResultFlag &= ~(1 << i);//����ϱ�״̬
					Icount2[i] = 0;
				}
			}
		}
//	#if !DriveMode
//		if(CAN_ID == TrainBasketMaxNum-1)//��ȡ��βλ��
//		{
//			PositionTail = RxRAM0[4]<<8 | RxRAM0[5];
//			PositionTail = PositionTail/2+PositionTail%2;
//			TrailPositionFlag = 1;
//			gCheckTrailLiveCount = 0;
//		}	//0814
//	#else
		
//		if(CAN_ID == TrainBasketMaxNum-1)//��ȡ��βλ��
//		{
//			PositionTail = RxRAM0[4]<<8 | RxRAM0[5];
//			PositionTail = PositionTail/2+PositionTail%2;
//			TrailPositionFlag = 1;
//		gCheckTrailLiveCount = 0;
//		}		
		
//	#endif		

		
	}
}




void PowerValueLedShow(void)//
{

//	float dat=0;
	gPowerValueFloat = ((1.0 * gCarBatPower) * 3.3 * (15.0 + 2.0) / 4096.0 / 2.0);
	powervol = gPowerValueFloat *10;	
//	dat = (powervol - CARPOWER_VUALE_MIN_FLOAT)*100.0/(CARPOWER_VUALE_MAX_FLOAT-CARPOWER_VUALE_MIN_FLOAT);

//	if(dat>99.0)
//	{
//		dat = 99; 
//	}
//	
//	if(dat < 1)
//	{
//		dat = 1;
//	}
//	carpower=(u8)(dat);
	
}


void Read_vol_contral(void)
{
	uint16_t  cnt = 0;
	u8 TransmitMailbox=0;	
	{
		TxMessage.ExtId = 0x0F585944;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.IDE = CAN_ID_EXT;
		TxMessage.DLC = 8;
			
		TxMessage.Data[0] = 0x02;
		TxMessage.Data[1] = 2;
		TxMessage.Data[2] = 0xa7;//vol
		TxMessage.Data[3] = 0xa8;//cur
		TxMessage.Data[4] = 0;
		TxMessage.Data[5] = 0;
		TxMessage.Data[6] = 0;
		TxMessage.Data[7] = 0;

		//send to meter
		TransmitMailbox=CAN_Transmit(CAN1,&TxMessage);
		cnt = 0;
		while((CAN_TransmitStatus(CAN1,TransmitMailbox) != CANTXOK) && (cnt != 0xffff))
		{
			cnt++;
		}
	}	
	
}

void time2_init(u16 arr,u16 psc)//
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrct;
	NVIC_InitTypeDef NVIC_InitStrct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_TimeBaseInitStrct.TIM_Period=arr;
	TIM_TimeBaseInitStrct.TIM_Prescaler=psc;
	TIM_TimeBaseInitStrct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStrct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStrct.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStrct);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	
	TIM_Cmd(TIM2,ENABLE);//ʧ��
	
	NVIC_InitStrct.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStrct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStrct.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStrct.NVIC_IRQChannelCmd=ENABLE;
	
	NVIC_Init(&NVIC_InitStrct);	
}


void TIM2_IRQHandler(void)
{
	if(TIM_GetFlagStatus(TIM2,TIM_IT_Update)==SET)//
//	if(TIM2->SR&0X0001) //Overflow interrupt //����ж�
	{	
		if(gSpeedR >= 10 && (TrainState == ST6 || TrainState == ST1))
			LongLDCount++;
		
		Timer40msCount++;
		Timer10msCount++;
		Timer100msCount++;
	
		gCheckHeartLiveCount++;
		if(gCheckHeartLiveCount >=800)//0.8S�������߼��ͳ�βλ�ü��
		{
			BasketLifeFlag = 0;
			
		}
//		gCheckTrailLiveCount++;
//		if(gCheckTrailLiveCount >= 2000)
//		{
//			TrailPositionFlag = 0;
//		}
		gCheckFroCarLiveCount++;
		if(gCheckFroCarLiveCount >= 1000)
		{
			FrontCarPositionFlag = 0;
		}		
		


///////////////////����ѹ//////////////////////////////////////////////	

		if(Timer40msCount>=40)//
		{
			Timer40msCount = 0;			
			ADC_Value = MyADC_GetValue();
			garry_ch0[gGetAdcCounter] = ADC_Value[0]; //��Դ��ѹ
			gGetAdcCounter++;
			if(gGetAdcCounter >= 10)
			{
				gGetAdcCounter = 0;
			}		
			GetAdcAverage();//10��ADCƽ��ֵ	
			PowerValueLedShow();	//����
		}
///////////////////����ѹ//////////////////////////////////////////////
		
///////////////////////////////��Ƭ����//////////////////////////////		
		if(Q_GDSW == 0 && H_GDSW == 0)
		{
			SwitchCount1++;
			if(SwitchCount1 == 1)
			{
				PocketCount++;	
				CageNumber = PocketCount/2+PocketCount%2;//��ں�				
			}
			else if(SwitchCount1 >= 20)
			{
				SwitchCount1 = 20;
			}
		}
		else if(Q_GDSW == 1 && H_GDSW == 1)
		{
			SwitchCount1 = 0;
		}
	
///////////////////////////////��Ƭ����//////////////////////////////			



///////////////////////////////������Ƭ����//////////////////////////////		
	
	if(H_LD == 0 && Q_LD ==0)
	{
		SwitchCount++;
		if(gSpeedR >= 10 && (TrainState == ST6 || TrainState == ST1))
			SwitchCount2++;
		
		if(SwitchCount == 1)
		{
			Pocket_A_Count++;
			SwitchCount2 = 0;
			if(Pocket_A_Count == Pocket4)//0802
			{
				PocketCountold = 0;
			}			
		}
		else if(SwitchCount >= 20 )
		{
			SwitchCount = 20;
		}
		
		if(SwitchCount2 >= 200)//������վ
		{
			InStationCount1++;
			if(InStationCount1 ==1)
			{
				InStationCount++;
				Pocket_A_Count = 0;
				PocketCountold = PocketCount;
				CageNumber = 0;PocketCount = 0;//�����ڼ���
			}
			InStationFlag = 1;
			StionStop = 1;
			PocketStep = 0;
			APPSendADDRFlag = 0;
			memset(AppSendAddr, 0, 21*sizeof(int));//����·���ַ	

			if(InStationCount1 >2)	
				InStationCount1 =2;			
			if(InStationCount >2)	
				InStationCount =2;
			
			if(TrainState == ST6)
			{
				TrainState = ST3;//��վ	
				StepState = 2;
			}
			else if(TrainState == ST1)		
			{
				//if(PocketCountold == (CaseNum*2-1) && InStationCount ==2)//��Ƭ������ȷ�˳��Լ�
				if(InStationCount ==2)//��Ƭ������ȷ�˳��Լ�	
				{
					TrainState = ST2;	
					StepState = 2;	
				}
			}
		}
	}
	else if(H_LD == 1 && Q_LD ==1)
	{
		SwitchCount = 0;
		InStationCount1 = 0;
	}	
///////////////////////////////������Ƭ����//////////////////////////////	
//	TrainHeadNum = KeyNumHead; //��ȡ��ͷ��			
	TrainContral();//���� 
	gSpeedR = OdReceivedData.vel_estimate[1].float_temp *22.0;//��ȡ�ٶ�
	TIM_ClearFlag(TIM2,TIM_IT_Update);
//	TIM2->SR&=~(1<<0); //Clear the interrupt flag bit //����жϱ�־λ
	}
}




