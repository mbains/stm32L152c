/*
 * app1.cpp
 *
 *  Created on: Sep 25, 2016
 *      Author: manny
 */
#include <app1.h>
#include <stm32l1xx_hal.h>
#include <string.h>
#include <mxconstants.h>
class BlinkTest_c 
{
public:
    BlinkTest_c(GPIO_TypeDef * port, uint16_t pin):
            m_port(port),
            m_pin(pin),
            m_state(GPIO_PIN_RESET)
    {
        
    };
    void toggle() {
        if(m_state == GPIO_PIN_RESET) {
            m_state = GPIO_PIN_SET;
        }else {
            m_state = GPIO_PIN_RESET;
        }
        HAL_GPIO_WritePin(m_port, m_pin, m_state);
    }
private:
    GPIO_TypeDef * m_port;
    uint16_t m_pin;
    GPIO_PinState m_state;
    
};

static BlinkTest_c b_test(LD3_GPIO_Port, LD3_Pin);
UART_HandleTypeDef * huart1;

int app1_main() {
    
    huart1->Instance = USART1;
    const char * test = "test\r\n";
    b_test.toggle();
    HAL_Delay(500);
    b_test.toggle();
    HAL_Delay(500);
    HAL_UART_Transmit(huart1, (uint8_t *)test, strlen(test), 100);
    return 0;
}


