/*
 * PWM.c
 *
 *  Created on: Apr 21, 2020
 *      Author: Bryce Gould
 */
#include <uCOS/uC-CFG/app_cfg.h>
#include <uCOS/uCOS-III/os.h>
#include "MCUType.h"
#include "K22FRDM_ClkCfg.h"
#include "K22FRDM_GPIO.h"
#include "BasicIO.h"
#include "PWM.h"
#include "UART.h"
#include "SeniorProject.h"
#include "ADC.h"

#define MODs 1000;
#define BIT_ONE 0x1
#define BIT_TWO_AND_THREE 0x3
#define CLEAR_FIRST_HALF 0x0F


static OS_TCB PWMTaskTCB;
static CPU_STK PWMTaskStk[APP_CFG_UART_TASK_STK_SIZE];

static OS_SEM ADCFlag;



void PWMTask(void);

void PWMInit(void){
    OS_ERR os_err;
    //enable FTM0 and FTM0 module clock
    SIM->SCGC6 |= 0x03000000;
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

    //ties peripherals to mux address
    PORTA->PCR[4]=PORT_PCR_MUX(3)|PORT_PCR_DSE_MASK;
    PORTD->PCR[4]=PORT_PCR_MUX(4)|PORT_PCR_DSE_MASK;

    FTM0->SC=FTM_SC_CLKS(0);
    FTM0->CNTIN=FTM_CNTIN_INIT(0);
    FTM0->CNT=FTM_CNT_COUNT(0);
    FTM0->MOD=MODs;

    FTM0->SC=FTM_SC_CLKS(1) | FTM_SC_PS(4);

    //pin 10 j1
    FTM0->CONTROLS[1].CnSC=FTM_CnSC_ELSB_MASK | FTM_CnSC_MSB_MASK;
    FTM0->CONTROLS[1].CnV=0;

    //pin 6 j2
    FTM0->CONTROLS[4].CnSC=FTM_CnSC_ELSB_MASK | FTM_CnSC_MSB_MASK;
    FTM0->CONTROLS[4].CnV=0;


    OSTaskCreate((OS_TCB     *)&PWMTaskTCB,
                   (CPU_CHAR   *)"PWM Task ",
                   (OS_TASK_PTR ) PWMTask,
                   (void       *) 0,
                   (OS_PRIO     ) APP_CFG_PWM_TASK_PRIO,
                   (CPU_STK    *)&PWMTaskStk[0],
                   (CPU_STK     )(APP_CFG_ADC_TASK_STK_SIZE / 10u),
                   (CPU_STK_SIZE) APP_CFG_ADC_TASK_STK_SIZE,
                   (OS_MSG_QTY  ) 0,
                   (OS_TICK     ) 0,
                   (void       *) 0,
                   (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                   (OS_ERR     *)&os_err);

    OSSemCreate(&ADCFlag,"ADCFlag",0,&os_err);

    while(os_err != OS_ERR_NONE){           /* Error Trap                        */
    }
}



void PWMTask(void){
    OS_ERR os_err;
    INT16U speed;
    INT16U drivedata;
    INT16U steer;
    INT8U power;

    INT8U forward;
    INT8U leftorright;
    INT8U turnstrength;

    while(1){

        drivedata = DrivePend(0, &os_err);
        ////////////////////////////////////////////////////////
        // drivedata is a INT8U with the first bit being forward or reverse, second bit left or right. 3rd and 4th bit the strength of the turn
        // and the last 4 bits the speed
        // drivedata = 0  00           0         0000
        //    direction^  ^left = 01   ^turn       ^drive strength
        //    forward=0    right= 10    strength
        //    reverse=1    strght=00

        // *drive strenght doesnt work below 0x04
        //////////////////////////////////////////////////////
        forward = drivedata >> 7;
        leftorright = (drivedata >> 5) & BIT_TWO_AND_THREE;
        turnstrength = (drivedata >> 4) & BIT_ONE;
        power = drivedata & CLEAR_FIRST_HALF;
        //adjustments for drive motor UART wont receive all zeros so Transmitter has replaced all zeros with a 1.
        if(power == 1){
            power = 0;
        }else{}
        //Motor will only start to turn once a certain PWM threshold is reached. Through
        // testing I determined it was around 350/1000 duty cycle
        speed = 350 + (power)*25;
        if(power == 0){
            speed = 0;
        }else{}
        //adjustments for steering motor

        ////////////////////////////////////////////////
        //Set turning strength
        ////////////////////////////////////////////////
        if(turnstrength == 1){
            steer = 800;
        }else{
            steer = 200;
        }
        /////////////////////////////////////////////////
        //Determine forward or reverse
        ////////////////////////////////////////////////
        if(forward == 0){
            //forward
            GPIOB->PSOR = GPIO_PIN(1); //Set PORTB0 pin 2 on j24
            GPIOB->PCOR = GPIO_PIN(0); //clear PORTB1 pin 4 on j24
        }else{
            //reverse
            GPIOB->PSOR = GPIO_PIN(0);
            GPIOB->PCOR = GPIO_PIN(1);
        }
        if(leftorright == 1){
            //left
            GPIOB->PSOR = GPIO_PIN(2); //set pin 12 on j24
            GPIOB->PCOR = GPIO_PIN(3); //clear pin 11 on j24
            FTM0->CONTROLS[4].CnV = 800;
        }else{
            if(leftorright == 2){
                //right
                GPIOB->PCOR = GPIO_PIN(2);
                GPIOB->PSOR = GPIO_PIN(3);
                FTM0->CONTROLS[4].CnV = 800;
            }else{
                FTM0->CONTROLS[4].CnV = 0;
            }
        }
        ////////////////////////////////////////////////////
        //Set speed of drive motor
        //////////////////////////////////////////////

        FTM0->CONTROLS[1].CnV = speed;

        //tell ADC so sample batteries and thermistor
        OSSemPost(&ADCFlag, OS_OPT_POST_1, &os_err);

        while(os_err != OS_ERR_NONE){           /* Error Trap                        */
        }
    }

}
//////////////////////////////////////////////////////////////////////
// ADCPend - Abstracted semaphore that signals ADC's to start sampling
//////////////////////////////////////////////////////////////////////
void ADCPend(OS_TICK tout, OS_ERR *p_err){
    OSSemPend(&ADCFlag, tout, OS_OPT_PEND_BLOCKING,(void *)0, p_err);
}
