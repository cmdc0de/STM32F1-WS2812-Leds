#ifndef __WS2812_H__
#define __WS2812_H__

//#include <stdint.h>
#include "stm32f10x_conf.h"

namespace cmdc0de {
/*
 * @author cmdc0de
 * @ date: 6/25/17
 *
 * Class to handle communications to WS2812 LEDs
 * NOTE:
 * 	LED_PER_DMA_BUFFER if this is set it will be used to set the DMA buffer size
 * 	Otherwise the DMA buffer size will be set large enough for 2 LEDs
 * 		Remember that's not the total number of LEDS you can have just the number that is buffered.
 */
class WS2818 {
public:
	WS2818(uint16_t LedPin, GPIO_TypeDef *ledPort, TIM_TypeDef *ledTimer, DMA_Channel_TypeDef *ledDMAChannel, IRQn_Type irqt);
	void init();
	void sendColors(uint8_t (*color)[3], uint16_t len);
	~WS2818();
	void handleISR();
protected:
	void fillLed(uint8_t *buffer, uint8_t *color);
private:
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint16_t LedPin;
	GPIO_TypeDef *LedPort;
	TIM_TypeDef *LedTimer;
	DMA_Channel_TypeDef *LedDMAChannel;
	IRQn_Type Irqt;
	int CurrentLed;
	int TotalLeds;
	uint8_t (*ColorLed)[3];
};

} //cmdc0de

#endif
