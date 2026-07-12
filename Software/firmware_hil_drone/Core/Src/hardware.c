/*
 * hardware.c
 *
 *  Created on: 12 jul 2026
 *      Author: kevin
 */

#include "hardware.h"

void Hardware_Init(void)
{

	TIM2_Init();
	TIM6_Init();

}

void TIM2_Init(void)
{

	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; // Enable the clock for the Timer
	RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN; // Enable the clock for GPIO Port A, where PWM Outputs are (PA0 - PA3)

	GPIOA->MODER   &= ~(GPIO_MODER_MODE0_1     | GPIO_MODER_MODE1_1     | GPIO_MODER_MODE2_1     | GPIO_MODER_MODE3_1); // Clear MODER register
	GPIOA->MODER   |=   GPIO_MODER_MODE0_1     | GPIO_MODER_MODE1_1     | GPIO_MODER_MODE2_1     | GPIO_MODER_MODE3_1;  // Configure GPIOs in Alternate Function mode

	GPIOA->AFR[0]  &= ~(GPIO_AFRL_AFSEL0_0     | GPIO_AFRL_AFSEL1_0     | GPIO_AFRL_AFSEL2_0     | GPIO_AFRL_AFSEL3_0); // Clear AFSEL register
	GPIOA->AFR[0]  |=   GPIO_AFRL_AFSEL0_0     | GPIO_AFRL_AFSEL1_0     | GPIO_AFRL_AFSEL2_0     | GPIO_AFRL_AFSEL3_0;  // Configure the Alternate Function to match Timer Channels (AF1)

	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0_1 | GPIO_OSPEEDR_OSPEED1_1 | GPIO_OSPEEDR_OSPEED2_1 | GPIO_OSPEEDR_OSPEED3_1); // Clear OSPEEDR register
	GPIOA->OSPEEDR |=   GPIO_OSPEEDR_OSPEED0_1 | GPIO_OSPEEDR_OSPEED1_1 | GPIO_OSPEEDR_OSPEED2_1 | GPIO_OSPEEDR_OSPEED3_1;  // Configure the GPIOs speed in HIGH for a quick response in PWM

	TIM2->PSC = 169;  // Configure the Prescaler value
	TIM2->ARR = 2499; // Configure the Auto-Reload value

	TIM2->EGR |= TIM_EGR_UG;  // Restart manually the counter
	TIM2->SR  &= ~TIM_SR_UIF; // Clear the interrupt flag, it's HIGH due to previous manual reset

	TIM2->CCMR1 |= (0x0006UL << TIM_CCMR1_OC1M_Pos) | (0x0006UL << TIM_CCMR1_OC2M_Pos); // Configure channels 1-2 to be in PWM mode
	TIM2->CCMR2 |= (0x0006UL << TIM_CCMR2_OC3M_Pos) | (0x0006UL << TIM_CCMR2_OC4M_Pos); // Configure channels 3-4 to be in PWM mode
	TIM2->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE; // Enable the Preload to avoid signal failures when changing PWM value in channels 1-2
	TIM2->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE; // Enable the Preload to avoid signal failures when changing PWM value in channels 3-4

	TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E; // Enable the GPIO outputs
	TIM2->CCR1  = 0; TIM2->CCR2 = 0; TIM2->CCR3 = 0; TIM2->CCR4 = 0;			 // Set all PWM to 0 duty cycle initially

	TIM2->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE; // Enable the Timer to start counting and Enable the ARR, to avoid problems if ARR value is changed

}

void TIM6_Init(void)
{

	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN; // Enable the clock for the Timer

	// We want an Interrupt in this timer every 1 ms
	TIM6->PSC = 169; // Configure the Prescaler value
	TIM6->ARR = 999; // Configure the Auto-Reload value

	TIM6->EGR |= TIM_EGR_UG;  // Restart manually the counter
	TIM6->SR  &= ~TIM_SR_UIF; // Clear the interrupt flag, it's HIGH due to previous manual reset

	TIM6->DIER |= TIM_DIER_UIE;			// Enable the Timer Interrupts
	NVIC_SetPriority(TIM6_DAC_IRQn, 1); // Configure the TIM6 Interrupt priority
	NVIC_EnableIRQ(TIM6_DAC_IRQn);		// Enable CPU to manage TIM6 Interrupts

	TIM6->CR1 |= TIM_CR1_CEN; // Enable the Timer to start counting

}


