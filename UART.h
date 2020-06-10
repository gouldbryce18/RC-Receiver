

/*
 * UART.h - Header file for UART.c
 *
 */

#ifndef UART_H_
#define UART_H_




/*******************************************************************************************
* Public Function Prototypes
*******************************************************************************************/

void UARTInit(void);

INT16U DrivePend(OS_TICK tout, OS_ERR *p_err);


typedef struct{
    INT8U data;
    INT8U speed;
    OS_SEM flag;
}DATA;

static DATA Drive;

//write a function

#endif /* UART_H_ */
