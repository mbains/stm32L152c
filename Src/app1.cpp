/*
 * app1.cpp
 *
 *  Created on: Sep 25, 2016
 *      Author: manny
 */
#include <app1.h>
#include <stm32l1xx_hal.h>
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

int app1_main() {

    
    b_test.toggle();
    HAL_Delay(500);
    b_test.toggle();
    HAL_Delay(500);
    return 0;
}


