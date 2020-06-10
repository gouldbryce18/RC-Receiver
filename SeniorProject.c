/*****************************************************************************************
  Receiver code for RC car
  -Bryce Gould 6/10/2020
*****************************************************************************************/

#include <uCOS/uC-CFG/app_cfg.h>
#include <uCOS/uCOS-III/os.h>
#include "MCUType.h"
#include "K22FRDM_ClkCfg.h"
#include "K22FRDM_GPIO.h"
#include "BasicIO.h"
#include "UART.h"
#include "ADC.h"
#include "PWM.h"



/*****************************************************************************************
* Allocate task control blocks
*****************************************************************************************/
static OS_TCB AppTaskStartTCB;
//static OS_TCB ControlTaskTCB;

/*****************************************************************************************
* Allocate task stack space.
*****************************************************************************************/
static CPU_STK AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];
//static CPU_STK ControlTaskStk[APP_CFG_ADC_TASK_STK_SIZE];

/*****************************************************************************************
* Task Function Prototypes. 
*   - Private if in the same module as startup task. Otherwise public.
*****************************************************************************************/
static void AppStartTask(void *p_arg);
//static void ControlTask(void *p_arg);
static void GPIOInit(void);
/*****************************************************************************************
* main()
*****************************************************************************************/
void main(void) {

    OS_ERR  os_err;

    K22FRDM_BootClock();
    CPU_IntDis();               /* Disable all interrupts, OS will enable them  */

    OSInit(&os_err);                    /* Initialize uC/OS-III                         */
    while(os_err != OS_ERR_NONE){                   /* Error Trap                       */
    }

    OSTaskCreate(&AppTaskStartTCB,                  /* Address of TCB assigned to task */
                 "Start Task",                      /* Name you want to give the task */
                 AppStartTask,                      /* Address of the task itself */
                 (void *) 0,                        /* p_arg is not used so null ptr */
                 APP_CFG_TASK_START_PRIO,           /* Priority you assign to the task */
                 &AppTaskStartStk[0],               /* Base address of task�s stack */
                 (APP_CFG_TASK_START_STK_SIZE/10u), /* Watermark limit for stack growth */
                 APP_CFG_TASK_START_STK_SIZE,       /* Stack size */
                 0,                                 /* Size of task message queue */
                 0,                                 /* Time quanta for round robin */
                 (void *) 0,                        /* Extension pointer is not used */
                 (OS_OPT_TASK_NONE), /* Options */
                 &os_err);                          /* Ptr to error code destination */

    while(os_err != OS_ERR_NONE){                   /* Error Trap                       */
    }

    OSStart(&os_err);               /*Start multitasking(i.e. give control to uC/OS)    */
    while(os_err != OS_ERR_NONE){                   /* Error Trap                       */
    }
}

/*****************************************************************************************
* STARTUP TASK
* This should run once and be suspended. Could restart everything by resuming.
* (Resuming not tested)
* Todd Morton, 01/06/2016
*****************************************************************************************/
static void AppStartTask(void *p_arg) {

    OS_ERR os_err;

    (void)p_arg;                        /* Avoid compiler warning for unused variable   */

    OS_CPU_SysTickInitFreq(SYSTEM_CLOCK);
    /* Initialize StatTask. This must be called when there is only one task running.
     * Therefore, any function call that creates a new task must come after this line.
     * Or, alternatively, you can comment out this line, or remove it. If you do, you
     * will not have accurate CPU load information                                       */
//    OSStatTaskCPUUsageInit(&os_err);

    ADCInit();
    UARTInit();
    PWMInit();
    GPIOInit();
    BIOOpen(BIO_BIT_RATE_9600);

//    OSTaskCreate(&ControlTaskTCB,                  /* Create DisplayTask                   */
//                    "Control Task ",
//                    ControlTask,
//                    (void *) 0,
//                    APP_CFG_CONTROL_TASK_PRIO,
//                    &ControlTaskStk[0],
//                    (APP_CFG_ADC_TASK_STK_SIZE / 10u),
//                    APP_CFG_ADC_TASK_STK_SIZE,
//                    0,
//                    0,
//                    (void *) 0,
//                    (OS_OPT_TASK_NONE),
//                    &os_err);

    OSTaskSuspend((OS_TCB *)0, &os_err);
    while(os_err != OS_ERR_NONE){                   /* Error Trap                   */
    }
}
//Future additions will be added in this task
//void ControlTask(void *p_arg) {
//    (void)p_arg;
//    OS_ERR os_err;
//
//    while(1) {
//
//
//
//        while(os_err != OS_ERR_NONE){           /* Error Trap                        */
//        }
//
//    }
//}

void GPIOInit(void){
    //initialize clock and mux to correct pins
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    PORTB->PCR[0] = PORT_PCR_MUX(1);
    PORTB->PCR[1] = PORT_PCR_MUX(1);
    PORTB->PCR[2] = PORT_PCR_MUX(1);
    PORTB->PCR[3] = PORT_PCR_MUX(1);
    PORTB->PCR[16] = PORT_PCR_MUX(1);
    PORTB->PCR[18] = PORT_PCR_MUX(1);

    //Clear all pins to start with all LEDs off
    GPIOB->PCOR = GPIO_PIN(0)|GPIO_PIN(1)|GPIO_PIN(2)|GPIO_PIN(3)|GPIO_PIN(16)|GPIO_PIN(18);

    //Set data direction to output
    GPIOB->PDDR |= GPIO_PIN(0)|GPIO_PIN(1)|GPIO_PIN(2)|GPIO_PIN(3)|GPIO_PIN(16)|GPIO_PIN(18);

}
