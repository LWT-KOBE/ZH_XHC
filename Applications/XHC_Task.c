#include "application.h"
#include "usartx.h"
#include "board.h"
XHC_TaskStruct_t XHCData;



XHC_TaskStruct_t* getXHC_Task(){
    return &XHCData;
}

/* ���ó�ʼ�� */
void XHC_TaskGlobalInit(void){

//	TIM3_Int_Init(65535, 83); //����HALL�ٶ�
}

void XHC_TaskUpdateTask(void *Parameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	digitalLo(&getXHC_Task()->dataInitFlag);
	while(true){
		vTaskDelayUntil(&xLastWakeTime,XHC_Task_NORMAL_PERIOD);
        //��ֹ�ظ���ʼ��
		if(!XHCData.dataInitFlag){	
            //���п���ȫ����ʼ��            
			XHC_TaskGlobalInit();																																							
			digitalHi(&getXHC_Task()->dataInitFlag);
			
		}
//////////////		
		USART_Data_Send_Task();//���ڷ���
//////////////	
		if(TrainState == ST2 && WorkOffFlag == 0x55)
		{
			WPS = 0;//д�����ر�
			delay_ms(1);
			EEPROM_Write_Byte(20,0xaa);			
			WPS = 1;	
			delay_ms(1);
			WorkOffFlag = EEPROM_Read_Byte(20);
		}			
///////////////////����ѹ//////////////////////////////////////////////	
	//	if(Timer40msCount>=40)//
		{
		//	Timer40msCount = 0;			
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
		
		digitalIncreasing(&getXHC_Task()->loops);        

	}
}

void XHCDataInit(void){
	getsupervisorData()->taskEvent[XHC_Task] = xTaskCreate(XHC_TaskUpdateTask,"XHC_Task",XHC_Task_STACK_SIZE,NULL,XHC_Task_PRIORITY,&XHCData.xHandleTask);
    //usbVCP_Printf("ControlInit Successfully \r\n");
    
}
