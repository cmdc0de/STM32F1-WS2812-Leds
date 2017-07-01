#ifndef __WS2812_H__
#define __WS2812_H__

//#include <stdint.h>
#include "stm32f10x_conf.h"

namespace cmdc0de {

class RGB {
public:
	static RGB WHITE;
	static RGB BLACK;
	static RGB RED;
	static RGB GREEN;
	static RGB BLUE;
	public:
	RGB();
	RGB(uint8_t r, uint8_t g, uint8_t b);
	~RGB();
	const uint8_t *getArray() const;
	uint8_t getR() const {
		return Clr[0];
	}
	uint8_t getG() const {
		return Clr[1];
	}
	uint8_t getB() const {
		return Clr[2];
	}
public:
	static RGB createRandomColor();
	private:
	uint8_t Clr[3];
};

class LedBuffer {
public:
	LedBuffer(uint8_t *ledColorBuffer, uint32_t NUM_LEDS) :
			LedData(ledColorBuffer), NumLeds(NUM_LEDS) {
	}
	~LedBuffer() {
	}
	uint8_t *getLeds() {return LedData;}
	uint8_t *getLed(uint16_t led) {return &LedData[led*3];}
	uint16_t getNumLeds() {return NumLeds;}
private:
	uint8_t *LedData;
	uint16_t NumLeds;
};

template<typename T, uint8_t D>
class Stack {
public:
	Stack() : StackMem(), InsertionPos(0) {}
	~Stack() {}
	bool push(T &v) {
		bool bRetVal = false;
		if(InsertionPos==(D)) {
			return false;
		}
		StackMem[InsertionPos]=v;
		InsertionPos++;
		return true;
	}
	T &pop() {
		if(InsertionPos==0) {
			return 0;
		} else {
			return StackMem[--InsertionPos];
		}
	}
protected:
	T StackMem[D];
	uint8_t InsertionPos;
};

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
	WS2818(uint16_t LedPin, GPIO_TypeDef *ledPort, TIM_TypeDef *ledTimer, DMA_Channel_TypeDef *ledDMAChannel,
			IRQn_Type irqt);
	void init();
	//void sendColors(uint8_t (*color)[3], uint16_t len);
	bool sendColors(LedBuffer *ColorLeds, uint32_t timeOut);
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
	LedBuffer *ColorLeds;
};

} //cmdc0de

#endif
