/*
 * ADC.c
 *
 *  Created on: Apr 21, 2020
 *      Author: Bryce Gould
 */
#include <BasicIO.h>
#include <MCUType.h>
#include <uCOS/uC-CFG/app_cfg.h>
#include <uCOS/uCOS-III/os.h>
#include "K22FRDM_ClkCfg.h"
#include "K22FRDM_GPIO.h"
#include "ADC.h"
#include "PWM.h"
#include "UART.h"

#define SET_BIT_ONE 0x1
#define CLEAR_BIT_ONE 0xFE
#define SET_BIT_TWO 0x2
#define CLEAR_BIT_TWO 0xFD
#define SET_BIT_THREE 0x4
#define CLEAR_BIT_THREE 0xFB


static OS_TCB ADCTaskTCB;
static CPU_STK ADCTaskStk[APP_CFG_UART_TASK_STK_SIZE];


void ADCTask(void);

void ADCTask(void){
    OS_ERR os_err;

    INT16U motorheat;
    INT16U motorbatt;
    INT16U MCUbatt;
    INT16U max = 36556;
    INT16U half = max/2;
    INT8U data = 0;

    while(1){
        ADCPend(0, &os_err);
       // OSTimeDly(15,OS_OPT_TIME_PERIODIC,&os_err);
        while(os_err != OS_ERR_NONE){           /* Error Trap                        */
        }

        //////////////////////////////////////////////////////////////
        // data is 8 bits with last bit being set if motor is too hot
        // second to last bit being set if motor battery is low
        // third to last bit is set if mcu battery is low
        //////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////
        // Motorheat
        /////////////////////////////////////////////////////////////
        ADC0->SC1[0] = ADC_SC1_ADCH(6);
        while((ADC0->SC1[0] & ADC_SC1_COCO_MASK)==0){}
        motorheat = ADC0->R[0]; //Get result
        if(motorheat < half){
            data = data | SET_BIT_ONE;
            GPIOB->PSOR = GPIO_PIN(16);
        }else{
            data = data & CLEAR_BIT_ONE;
            GPIOB->PSOR = GPIO_PIN(16);
        }


        /////////////////////////////////////////////////////////////
        // motor battery
        /////////////////////////////////////////////////////////////
        ADC0->SC1[0] = ADC_SC1_ADCH(7); //Start
        while((ADC0->SC1[0] & ADC_SC1_COCO_MASK)==0){}
        motorbatt = ADC0->R[0]; //Get result
        if(motorbatt < half){
            data = data | SET_BIT_TWO;
            GPIOB->PSOR = GPIO_PIN(18);
        }else{
            data = data & CLEAR_BIT_TWO;
            GPIOB->PCOR = GPIO_PIN(18);
        }



        ////////////////////////////////////////////////////////////
        // MCU battery
        ////////////////////////////////////////////////////////////

        ADC1->SC1[0] = ADC_SC1_ADCH(4);
        while((ADC1->SC1[0] & ADC_SC1_COCO_MASK)==0){}
        MCUbatt = ADC1->R[1]; //Get result

        if(MCUbatt < half){

                data = data | SET_BIT_THREE;
                GPIOB->PSOR = GPIO_PIN(16);
            }else{
                data = data & CLEAR_BIT_THREE;
                GPIOB->PCOR = GPIO_PIN(16);

            }
        //holds the information to send to remote
        Drive.data = data;



   }
    while(os_err != OS_ERR_NONE){           /* Error Trap                        */
    }

}
void ADCInit(void){
    OS_ERR os_err;
    //Software triggered conversions
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
    SIM->SCGC6 |= SIM_SCGC6_ADC1_MASK;

    ADC0->CFG1 |= ADC_CFG1_ADIV(3)|ADC_CFG1_MODE(3)|ADC_CFG1_ADLSMP_MASK;
    ADC1->CFG1 |= ADC_CFG1_ADIV(3)|ADC_CFG1_MODE(3)|ADC_CFG1_ADLSMP_MASK;


    //Using ADC's 6b, 7b

    //J2 pin 12 for channel 6
    ADC0->SC1[0] = ADC_SC1_ADCH(6);
    //J2 pin 8 for channel 7
    //J2 pin 14 for channel 4
    ADC1->SC1[0] = ADC_SC1_ADCH(4);


    //////////////////////////////////////////////////////
    // Need initialization for analog stick default value
    //////////////////////////////////////////////////////
    OSTaskCreate((OS_TCB     *)&ADCTaskTCB,
                   (CPU_CHAR   *)"ADC Task ",
                   (OS_TASK_PTR ) ADCTask,
                   (void       *) 0,
                   (OS_PRIO     ) APP_CFG_ADC_TASK_PRIO,
                   (CPU_STK    *)&ADCTaskStk[0],
                   (CPU_STK     )(APP_CFG_ADC_TASK_STK_SIZE / 10u),
                   (CPU_STK_SIZE) APP_CFG_ADC_TASK_STK_SIZE,
                   (OS_MSG_QTY  ) 0,
                   (OS_TICK     ) 0,
                   (void       *) 0,
                   (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                   (OS_ERR     *)&os_err);


    while(os_err != OS_ERR_NONE){           /* Error Trap                        */
    }

}



