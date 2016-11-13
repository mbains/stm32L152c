/*
 * app1.cpp
 *
 *  Created on: Sep 25, 2016
 *      Author: manny
 */
#include <app1.h>
class BlinkTest_c 
{
public:
    BlinkTest_c(GPIO_TypeDef * port, uint16_t pin):
            m_port(port),
            m_pin(pin)
    {
        
    };
    void toggle() {
        
    }
private:
    GPIO_TypeDef * m_port;
    uint16_t m_pin;
    
};
int app1_main() {

	return 0;
}


