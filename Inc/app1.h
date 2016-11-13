/*
 * app1.h
 *
 *  Created on: Sep 25, 2016
 *      Author: manny
 */

#ifndef APP1_H_
#define APP1_H_
#ifdef __cplusplus
extern "C" {  
#endif  
    #include <stm32l1xx_hal.h>
    int app1_main(UART_HandleTypeDef * huart1);
#ifdef __cplusplus  
} // extern "C"  
#endif

#endif /* APP1_H_ */
