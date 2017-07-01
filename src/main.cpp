//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "Timer.h"
#include "BlinkLed.h"
#include "ws2812.h"

// Definitions visible only within this translation unit.
namespace
{
// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
constexpr Timer::ticks_t BLINK_ON_TICKS = 75; //Timer::FREQUENCY_HZ * 3 / 4;
constexpr Timer::ticks_t BLINK_OFF_TICKS = 100; //Timer::FREQUENCY_HZ- BLINK_ON_TICKS;
}

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

uint8_t One[36];
cmdc0de::WS2818 Leds1(GPIO_Pin_7, GPIOA, TIM1, DMA1_Channel2, DMA1_Channel2_IRQn);
cmdc0de::LedBuffer LBuffer(&One[0], 12);

extern "C" {
void DMA1_Channel2_IRQHandler() {
	Leds1.handleISR();
}
}

int
main(int argc, char* argv[]) {
	// Send a greeting to the trace device (skipped on Release).
	trace_puts("Hello ARM World!");

	Leds1.init();

	// At this stage the system clock should have already been configured
	// at high speed.
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	Timer timer;
	timer.start();

	BlinkLed blinkLed;
	blinkLed.powerUp();

	uint32_t seconds = 0;

	// Infinite loop
	while (1)
	{

		for (int i = 0; i < 12; i++) {
			One[i * 3] = rand() % 256;
			One[i * 3 + 1] = rand() % 256;
			One[i * 3 + 2] = rand() % 256;
		}
		Leds1.sendColors(&LBuffer, 50);

		blinkLed.turnOn();
		timer.sleep(seconds == 0 ? Timer::FREQUENCY_HZ : BLINK_ON_TICKS);

		for (int i = 0; i < 12; i++) {
			One[i * 3] = rand() % 256;
			One[i * 3 + 1] = rand() % 256;
			One[i * 3 + 2] = rand() % 256;
		}
		Leds1.sendColors(&LBuffer, 12);

		blinkLed.turnOff();
		timer.sleep(BLINK_OFF_TICKS);

		++seconds;

		// Count seconds on the trace device.
		trace_printf("Second %u\n", seconds);
	}
	// Infinite loop, never return.
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
