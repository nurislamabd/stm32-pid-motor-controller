#include "stm32f4xx.h"

/* ===================== UART ===================== */

void uart_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // PA2 (TX), PA3 (RX) -> alternate function mode
    GPIOA->MODER &= ~(0x3 << (2 * 2));
    GPIOA->MODER |=  (0x2 << (2 * 2));
    GPIOA->MODER &= ~(0x3 << (3 * 2));
    GPIOA->MODER |=  (0x2 << (3 * 2));

    // AF7 for USART2
    GPIOA->AFR[0] &= ~(0xF << (2 * 4));
    GPIOA->AFR[0] |=  (0x7 << (2 * 4));
    GPIOA->AFR[0] &= ~(0xF << (3 * 4));
    GPIOA->AFR[0] |=  (0x7 << (3 * 4));

    // 115200 baud @ 16MHz HSI
    USART2->BRR = (8 << 4) | 11;

    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void uart_send_char(char c) {
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = c;
}

void uart_print(const char* str) {
    while (*str) uart_send_char(*str++);
}

void uart_print_int(int value) {
    char temp[12];
    int idx = 0;

    if (value < 0) {
        uart_send_char('-');
        value = -value;
    }

    if (value == 0) {
        temp[idx++] = '0';
    } else {
        while (value > 0) {
            temp[idx++] = '0' + (value % 10);
            value /= 10;
        }
    }

    for (int i = idx - 1; i >= 0; i--) {
        uart_send_char(temp[i]);
    }
}

/* ===================== PWM (TIM1, motor drive) ===================== */

void pwm_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // PA8 -> alternate function mode (TIM1_CH1)
    GPIOA->MODER &= ~(0x3 << (8 * 2));
    GPIOA->MODER |=  (0x2 << (8 * 2));
    GPIOA->AFR[1] &= ~(0xF << ((8 - 8) * 4));
    GPIOA->AFR[1] |=  (0x1 << ((8 - 8) * 4));

    // PB4, PB5 -> general purpose output (IN1, IN2)
    GPIOB->MODER &= ~(0x3 << (4 * 2));
    GPIOB->MODER |=  (0x1 << (4 * 2));
    GPIOB->MODER &= ~(0x3 << (5 * 2));
    GPIOB->MODER |=  (0x1 << (5 * 2));

    // NOTE: core clock is 16MHz (HSI, no PLL configured)
    // PSC = 15 -> 1MHz counter clock, ARR = 999 -> 1kHz PWM
    TIM1->PSC = 15;
    TIM1->ARR = 999;

    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM1->CCMR1 |= (0x6 << 4);   // PWM mode 1
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE;

    TIM1->CCER |= TIM_CCER_CC1E;
    TIM1->BDTR |= TIM_BDTR_MOE;  // required for TIM1 advanced timer output

    TIM1->CCR1 = 0;

    TIM1->CR1 |= TIM_CR1_CEN;
}

void set_motor_speed(int duty_percent, int forward) {
    if (duty_percent > 100) duty_percent = 100;
    if (duty_percent < 0) duty_percent = 0;

    TIM1->CCR1 = (duty_percent * 999) / 100;

    if (forward) {
        GPIOB->ODR |= (1 << 4);
        GPIOB->ODR &= ~(1 << 5);
    } else {
        GPIOB->ODR &= ~(1 << 4);
        GPIOB->ODR |= (1 << 5);
    }
}

/* ===================== Encoder (TIM2) ===================== */

void encoder_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // PA0, PA1 -> alternate function mode (TIM2_CH1, TIM2_CH2)
    GPIOA->MODER &= ~(0x3 << (0 * 2));
    GPIOA->MODER |=  (0x2 << (0 * 2));
    GPIOA->MODER &= ~(0x3 << (1 * 2));
    GPIOA->MODER |=  (0x2 << (1 * 2));

    GPIOA->AFR[0] &= ~(0xF << (0 * 4));
    GPIOA->AFR[0] |=  (0x1 << (0 * 4));
    GPIOA->AFR[0] &= ~(0xF << (1 * 4));
    GPIOA->AFR[0] |=  (0x1 << (1 * 4));

    TIM2->SMCR &= ~TIM_SMCR_SMS;
    TIM2->SMCR |= 0x3;  // encoder mode 3 (count on both TI1 and TI2 edges)

    TIM2->CCMR1 |= TIM_CCMR1_CC1S_0;  // CC1 mapped to TI1
    TIM2->CCMR1 |= TIM_CCMR1_CC2S_0;  // CC2 mapped to TI2

    TIM2->ARR = 0xFFFFFFFF;

    TIM2->CR1 |= TIM_CR1_CEN;
}

int32_t get_encoder_count(void) {
    return (int32_t)TIM2->CNT;
}

/* ===================== SysTick (10ms control loop tick) ===================== */

volatile int control_loop_flag = 0;
volatile uint32_t millis_counter = 0;

void systick_init(void) {
    // 16MHz HSI core clock, want 10ms tick -> 160,000 cycles
    SysTick->LOAD = 160000 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void) {
    control_loop_flag = 1;
    millis_counter += 10;  // since SysTick fires every 10ms
}

/* ===================== PID ===================== */

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float output_min, output_max;
} PID;

float pid_update(PID* pid, float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    float output = pid->kp * error
                 + pid->ki * pid->integral
                 + pid->kd * derivative;

    if (output > pid->output_max) {
        output = pid->output_max;
        pid->integral -= error * dt;  // anti-windup
    }
    if (output < pid->output_min) {
        output = pid->output_min;
        pid->integral -= error * dt;
    }

    return output;
}

/* ===================== RPM calculation ===================== */

float calculate_rpm(int32_t delta_count, float dt_seconds) {
    const float CPR = 700.0f;  // counts per output-shaft revolution (assumed)
    float revolutions = delta_count / CPR;
    float rpm = (revolutions / dt_seconds) * 60.0f;
    return rpm;
}

/* ===================== Main ===================== */

int main(void) {
	SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));

    uart_init();
    pwm_init();
    encoder_init();
    systick_init();

    uart_print("PID test starting\r\n");

    PID pid = {
        .kp = 1.0f,
        .ki = 0.1f,
        .kd = 0.0f,
        .integral = 0.0f,
        .prev_error = 0.0f,
        .output_min = 0.0f,
        .output_max = 100.0f
    };

    float target_rpm = 100.0f;
    int32_t last_count = 0;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= (1 << 10);  // PA5 as output

    while (1) {
        if (control_loop_flag) {
            control_loop_flag = 0;

            int32_t current_count = get_encoder_count();
            int32_t delta = current_count - last_count;
            last_count = current_count;

            float rpm = calculate_rpm(delta, 0.010f);

            float duty = pid_update(&pid, target_rpm, rpm, 0.010f);

            set_motor_speed((int)duty, 1);

            uart_print_int(millis_counter);
            uart_print(",");
            uart_print_int((int)target_rpm);
            uart_print(",");
            uart_print_int((int)rpm);
            uart_print("\r\n");
        }
    }
}
