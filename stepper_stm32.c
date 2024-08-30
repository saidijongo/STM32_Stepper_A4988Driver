#include "stm32f4xx.h"

// Define GPIO pins for the stepper motor driver
#define STEP_PIN GPIO_Pin_8
#define DIR_PIN GPIO_Pin_9
#define ENABLE_PIN GPIO_Pin_10

// Define the timer used for PWM output
#define PWM_TIMER TIM2

// Define the frequency of the PWM signal
#define PWM_FREQ 1000

// Define the maximum speed of the stepper motor in steps per second
#define MAX_SPEED 200

// Define the acceleration of the stepper motor in steps per second squared
#define ACCELERATION 1000

// Define the number of steps per revolution of the stepper motor
#define STEPS_PER_REV 200

// Define the number of microsteps per step
#define MICROSTEPS 16

// Define the number of steps to move for each microstep
#define STEPS_PER_MICROSTEP (STEPS_PER_REV * MICROSTEPS)

// Define the maximum delay between steps in microseconds
#define MAX_STEP_DELAY (1000000 / MAX_SPEED)

// Define the minimum delay between steps in microseconds
#define MIN_STEP_DELAY (1000000 / (MAX_SPEED * ACCELERATION))

// Define the current step delay in microseconds
uint32_t step_delay = MAX_STEP_DELAY;

void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    // Enable the GPIOA clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // Configure the STEP, DIR, and ENABLE pins as digital output pins
    GPIO_InitStruct.GPIO_Pin = STEP_PIN | DIR_PIN | ENABLE_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_InitStruct;

    // Enable the GPIOA and TIM2 clocks
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE_PIN, ENABLE);
// Configure TIM2 CH1 (PA5) as PWM output
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_TIM2);

// Configure TIM2 as a PWM timer
    TIM_InitStruct.TIM_Prescaler = SystemCoreClock / (PWM_FREQ * 65535) - 1;
    TIM_InitStruct.TIM_Period = 65535;
    TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(PWM_TIMER, &TIM_InitStruct);

    TIM_OCInitTypeDef TIM_OC_InitStruct;

    TIM_OC_InitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OC_InitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC_InitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC_InitStruct.TIM_Pulse = 0;
    TIM_OC1Init(PWM_TIMER, &TIM_OC_InitStruct);

    TIM_Cmd(PWM_TIMER, ENABLE);
}

void delay_us(uint32_t us)
{
    TIM_Cmd(TIM6, DISABLE);
    TIM_SetCounter(TIM6, 0);
    TIM_SetAutoreload(TIM6, us - 1);
    TIM_Cmd(TIM6, ENABLE);
    while (TIM_GetCounter(TIM6) > 0)
        ;
}
void step_motor(uint16_t steps, uint8_t dir)
{
// Set the DIR pin to the desired direction
    if (dir == 0)
    {
        GPIO_ResetBits(GPIOA, DIR_PIN);
    }
    else
    {
        GPIO_SetBits(GPIOA, DIR_PIN);
    }
// Generate the desired number of steps
    for (uint16_t i = 0; i < steps; i++)
    {
        // Generate a pulse on the STEP pin
        GPIO_SetBits(GPIOA, STEP_PIN);
        delay_us(step_delay);
        GPIO_ResetBits(GPIOA, STEP_PIN);
        delay_us(step_delay);
    }
}

int main(void)
{
// Initialize the GPIO and PWM peripherals
    GPIO_Init();
    PWM_Init();
// Enable the TIM6 clock for delay_us function
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

// Disable the stepper motor driver
    GPIO_SetBits(GPIOA, ENABLE_PIN);

// Set the current step delay to the maximum delay
    step_delay = MAX_STEP_DELAY;

    while (1)
    {
        // Enable the stepper motor driver
        GPIO_ResetBits(GPIOA, ENABLE_PIN);

        // Move the motor in one direction for one revolution
        step_motor(STEPS_PER_REV * MICROSTEPS, 0);

        // Delay for 1 second
        delay_us(1000000);

        // Move the motor in the other direction for one revolution
        step_motor(STEPS_PER_REV * MICROSTEPS, 1);

        // Delay for 1 second
        delay_us(1000000);

        // Gradually increase the speed of the motor
        for (uint32_t delay = MAX_STEP_DELAY; delay > MIN_STEP_DELAY; delay -=
                ACCELERATION)
        {
            step_delay = delay;
            step_motor(STEPS_PER_MICROSTEP, 0);
        }

        // Gradually decrease the speed of the motor
        for (uint32_t delay = MIN_STEP_DELAY;

        delay < MAX_STEP_DELAY; delay += ACCELERATION)
        {
            step_delay = delay;
            step_motor(STEPS_PER_MICROSTEP, 1);
        }
        // Disable the stepper motor driver
        GPIO_SetBits(GPIOA, ENABLE_PIN);

        // Delay for 1 second
        delay_us(1000000);
    }
}
