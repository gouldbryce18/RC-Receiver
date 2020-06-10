/*******************************************************************************************
UART code
   Contains
       -Transmit task
       -Receive task
       -PWMPend
       -DrivePend


Bryce Gould   6/10/2020
*******************************************************************************************/

#include <uCOS/uC-CFG/app_cfg.h>
#include <uCOS/uCOS-III/os.h>
#include "MCUType.h"
#include "K22FRDM_ClkCfg.h"
#include "K22FRDM_GPIO.h"
#include "BasicIO.h"
#include "UART.h"
#include "ADC.h"
#include "PWM.h"

static OS_TCB RXTaskTCB;
static OS_TCB TXTaskTCB;
static CPU_STK RXTaskStk[APP_CFG_UART_TASK_STK_SIZE];
static CPU_STK TXTaskStk[APP_CFG_UART_TASK_STK_SIZE];





void RXTask(void);
void TXTask(void);

void UARTInit(void){
    OS_ERR os_err;


    BIOOpen(BIO_BIT_RATE_9600);

    OSTaskCreate((OS_TCB     *)&RXTaskTCB,
                   (CPU_CHAR   *)"RX Task ",
                   (OS_TASK_PTR ) RXTask,
                   (void       *) 0,
                   (OS_PRIO     ) APP_CFG_RX_TASK_PRIO,
                   (CPU_STK    *)&RXTaskStk[0],
                   (CPU_STK     )(APP_CFG_ADC_TASK_STK_SIZE / 10u),
                   (CPU_STK_SIZE) APP_CFG_ADC_TASK_STK_SIZE,
                   (OS_MSG_QTY  ) 0,
                   (OS_TICK     ) 0,
                   (void       *) 0,
                   (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                   (OS_ERR     *)&os_err);
    while(os_err != OS_ERR_NONE){           /* Error Trap                        */
    }
    OSTaskCreate((OS_TCB     *)&TXTaskTCB,
                   (CPU_CHAR   *)"TX Task ",
                   (OS_TASK_PTR ) TXTask,
                   (void       *) 0,
                   (OS_PRIO     ) APP_CFG_TX_TASK_PRIO,
                   (CPU_STK    *)&TXTaskStk[0],
                   (CPU_STK     )(APP_CFG_ADC_TASK_STK_SIZE / 10u),
                   (CPU_STK_SIZE) APP_CFG_ADC_TASK_STK_SIZE,
                   (OS_MSG_QTY  ) 0,
                   (OS_TICK     ) 0,
                   (void       *) 0,
                   (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                   (OS_ERR     *)&os_err);


    OSSemCreate(&(Drive.flag),"DriveFlag",0,&os_err);

    while(os_err != OS_ERR_NONE){           /* Error Trap                        */
    }
}

void RXTask(void){
    OS_ERR os_err;
    INT8U RXdata;

    while(1){
        OSTimeDly(300,OS_OPT_TIME_PERIODIC,&os_err);
        RXdata =BIOGetChar();
        Drive.speed = RXdata;
        //Light up corresponding LEDS
        OSSemPost(&(Drive.flag), OS_OPT_POST_1, &os_err);
    }
    while(os_err != OS_ERR_NONE){           /* Error Trap                        */
    }
}
void TXTask(void){
    OS_ERR os_err;
    INT8U TXdata;

    while(1){

        OSTimeDly(300,OS_OPT_TIME_PERIODIC,&os_err);
        TXdata = Drive.data;
        BIOWrite(TXdata);

    }
    while(os_err != OS_ERR_NONE){           /* Error Trap                        */
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
// DrivePend - abstracted semaphore that signals to PWM that drive data has been received
/////////////////////////////////////////////////////////////////////////////////////////
INT16U DrivePend(OS_TICK tout, OS_ERR *os_err_ptr){

    OSSemPend(&(Drive.flag), tout, OS_OPT_PEND_BLOCKING,(void *)0, os_err_ptr);
    return Drive.speed;
}


