#include "application.h"
#include "usartx.h"
#include "board.h"
XHC_TaskStruct_t XHCData;



XHC_TaskStruct_t* getXHC_Task(){
    return &XHCData;
}

/* ���ó�ʼ�� */
void XHC_TaskGlobalInit(void){
	IO_Init();
	uart2_init(115200);
	uart3_init(115200);
	DMA_Config();
	time2_init(99,839);
	MyADC_Init();
	EEPROM_GPIO_Init(); 
	TrainHeadNum = KeyNumHead; //��ȡ��ͷ��	
//	ReadE2promData();
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
		Can_USART_Data_Send_Task();//���ڡ�CAN����


//////////////		
		digitalIncreasing(&getXHC_Task()->loops);        

	}
}

void XHCDataInit(void){
	getsupervisorData()->taskEvent[XHC_Task] = xTaskCreate(XHC_TaskUpdateTask,"XHC_Task",XHC_Task_STACK_SIZE,NULL,XHC_Task_PRIORITY,&XHCData.xHandleTask);
    //usbVCP_Printf("ControlInit Successfully \r\n");
    
}
