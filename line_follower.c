#include "stm32f10x.h"
#include <stdbool.h>

// Function Prototypes
void GPIOInit(void);
void PWMConfig(void);
void TIM2Config(void);
int QTRead(int *sensor);
void delay_us(uint16_t us);
void delay_ms(uint16_t ms);
void HCSR04_Config(void);
uint32_t HCSR04_ReadDistance(void);

int error_vector[8] = { -20, -15, -10, -5, 5, 10, 15, 20 };
double KP = 1.0;
double KI = 0.00015;
double KD = 3.0;

#define PWMValue 45
#define INTEGRAL_LIMIT 10000
#define STOP_DISTANCE 50 // Stop distance threshold in cm

void delay_us(uint16_t us) {
    TIM2->CNT = 0; // Reset counter
    while (TIM2->CNT < us);
}

void delay_ms(uint16_t ms) {
    for (uint16_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}

void TIM2Config(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 72 - 1;
    TIM2->ARR = 0xFFFF;
    TIM2->CR1 |= TIM_CR1_CEN;
}

void GPIOInit(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
    GPIOB->CRL = 0x33333333; // Input for sensors
    GPIOC->CRH = 0x33333333; // Configure PC pins (e.g., for LED control)
}

void HCSR04_Config(void) {
    // PB0 -> TRIG, PB1 -> ECHO
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; // Enable GPIOB clock

    // TRIG (PB0) -> Output, Push-Pull
    GPIOB->CRL &= ~(0xF << (4 * 0));   // Clear PB0 configuration
    GPIOB->CRL |= (0x3 << (4 * 0));    // Output mode, max speed 50 MHz
    GPIOB->CRL &= ~(0xC << (4 * 0));   // Push-pull output

    // ECHO (PB1) -> Input, Pull-Down
    GPIOB->CRL &= ~(0xF << (4 * 1));   // Clear PB1 configuration
    GPIOB->CRL |= (0x8 << (4 * 1));    // Input mode, pull-down
    GPIOB->ODR &= ~(1 << 1);           // Enable pull-down
}

uint32_t HCSR04_ReadDistance(void) {
    uint32_t start, end;

    // Generate TRIG pulse (10 µs HIGH)
    GPIOB->BSRR = (1 << 0); // Set PB0 HIGH
    delay_us(10);
    GPIOB->BRR = (1 << 0);  // Set PB0 LOW

    // Wait for ECHO to go HIGH
    while (!(GPIOB->IDR & (1 << 1)));
    start = TIM2->CNT;

    // Wait for ECHO to go LOW
    while (GPIOB->IDR & (1 << 1));
    end = TIM2->CNT;

    // Calculate duration and convert to distance (in cm)
    uint32_t duration = end - start;
    return (duration * 0.0343) / 2; // Speed of sound: 343 m/s
}

int QTRead(int *sensor) {
    GPIOA->CRL = 0x33333333; // Output mode
    GPIOA->ODR = 0xFF;
    delay_us(15);
    GPIOA->CRL = 0x88888888; // Input mode
    delay_ms(6);

    int SensorNum = 0;
    int Position = 0;

    for (int i = 0; i < 8; i++) {
        if ((GPIOA->IDR >> i) & 1) {
            sensor[i] = 1;
            Position += error_vector[i];
            SensorNum++;
        } else {
            sensor[i] = 0;
        }
    }
    if (SensorNum == 0) {
        return 0; // Avoid division by zero
    }
    return Position / SensorNum;
}

void PWMConfig(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_TIM1EN;
    GPIOA->CRH |= GPIO_CRH_MODE8 | GPIO_CRH_MODE11;
    GPIOA->CRH &= ~((GPIO_CRH_CNF8_0 | GPIO_CRH_CNF11_0));
    GPIOA->CRH |= GPIO_CRH_CNF8_1 | GPIO_CRH_CNF11_1;

    TIM1->PSC = 71;
    TIM1->ARR = 100;
    TIM1->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
    TIM1->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC4E;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1 |= TIM_CR1_CEN;

    TIM1->CCR1 = 50;
    TIM1->CCR4 = 50;
}

int main(void) {
    PWMConfig();
    GPIOInit();
    TIM2Config();
    HCSR04_Config();

    int32_t CurrPosError = 0, PrevPosError = 0;
    int32_t ProportionalPosition = 0, DerivativePosition = 0;
    int64_t IntegralPosition = 0;

    while (1) {
        int sensor[8];
        uint32_t distance = HCSR04_ReadDistance(); // Read distance from HC-SR04

        if (distance < STOP_DISTANCE) {
            // Stop the robot if an obstacle is detected within STOP_DISTANCE
            TIM1->CCR1 = 0; // Stop motor 1
            TIM1->CCR4 = 0; // Stop motor 2
            continue; // Skip the rest of the loop
        }

        int PositionError = QTRead(sensor);
        bool SensorEmptyFlag = true;

        for (int i = 0; i < 8; i++) {
            if (sensor[i] == 1) {
                SensorEmptyFlag = false;
                break;
            }
        }

        if (SensorEmptyFlag) {
            CurrPosError = PrevPosError * 36;
            if (CurrPosError < -4500) CurrPosError = -4500;
            if (CurrPosError > 4500) CurrPosError = 4500;
        } else {
            CurrPosError = PositionError;
        }

        ProportionalPosition = CurrPosError;
        DerivativePosition = CurrPosError - PrevPosError;
        IntegralPosition += CurrPosError;

        if (IntegralPosition > INTEGRAL_LIMIT) IntegralPosition = INTEGRAL_LIMIT;
        if (IntegralPosition < -INTEGRAL_LIMIT) IntegralPosition = -INTEGRAL_LIMIT;

        int PWManager = ProportionalPosition * KP + DerivativePosition * KD + IntegralPosition * KI;

        if (PWManager > PWMValue) PWManager = PWMValue;
        if (PWManager < -PWMValue) PWManager = -PWMValue;

        if (PWManager < 0) {
            TIM1->CCR1 = PWMValue + PWManager;
            TIM1->CCR4 = PWMValue;
        } else {
            TIM1->CCR1 = PWMValue;
            TIM1->CCR4 = PWMValue - PWManager;
        }

        PrevPosError = CurrPosError;
    }
}
