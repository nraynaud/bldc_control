
#include "stm32f4_discovery.h"
#include "arm_math.h"

uint8_t stepConfiguration[6][6] = {
    {0, 1, 1, 0, 0, 0},
    {0, 0, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 1},
    {1, 0, 0, 1, 0, 0},
    {0, 0, 0, 1, 1, 0},
    {0, 1, 0, 0, 1, 0}
};//time is vertical, switch state horizontal


__IO float32_t rotationFreq = 1;
__IO float32_t attenuation = 0;
__IO uint32_t rampPosition = 0;
__IO float32_t rotorPosition = 0;

uint32_t rampSteps = 100;
float32_t rampRate = 40;//Hz
float32_t minRotationFreq = 1;
float32_t maxRotationFreq = 300;
float32_t finalAttenuation = 0.19f;
float32_t startAttenuation = 0.4f;
float32_t pwmFreq = 14000;
uint16_t prescaler = 9;
float32_t division = 0;
uint16_t deadTime = 200; //strange units
uint16_t TimerPeriod = 0;

void tim3Config(void) {
    uint16_t tim3Prescaler = 999;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    /* Enable the TIM3 global Interrupt */
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* Time base configuration */
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    
    TIM_TimeBaseStructure.TIM_Period = (SystemCoreClock / rampRate / (tim3Prescaler + 1)) - 1;;
    TIM_TimeBaseStructure.TIM_Prescaler = tim3Prescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);
    TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void tim1Config(void) {
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Enable the TIM1 CC Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* Time Base configuration */
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
    TIM_TimeBaseStructure.TIM_ClockDivision = division;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    
    /* Automatic Output enable, Break, dead time and lock configuration*/
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    TIM_BDTRInitStructure.TIM_DeadTime = deadTime;
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
    
    
    TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
    
    /* TIM1 counter enable */
    TIM_Cmd(TIM1, ENABLE);
    
    /* Main Output Enable */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void configureTim1Pins(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* GPIOA and GPIOB clocks enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOE, ENABLE);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    
    /* GPIOA Configuration: Channel 1 and 3 as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* GPIOA Configuration: Channel 2 as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    /* GPIOB Configuration: BKIN, Channel 1N, 2N and 3N as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* Connect TIM pins to AF1 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
}

//use PC7 as input phase 1
void configMotorInputPins() {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* Enable GPIOC clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    /* Connect EXTI Line7 to PA7 pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource7);
    EXTI_InitTypeDef   EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line7;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    /* Enable and set EXTI Line9_5 Interrupt to the lowest priority */
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

// TIM1 is used for fast PWM stuff
// TIM3 is used for the speed ramping
int main(void)
{
    //enable FPU
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
    
    STM_EVAL_LEDInit(LED3);
    STM_EVAL_LEDInit(LED4);
    STM_EVAL_LEDInit(LED5);
    STM_EVAL_LEDInit(LED6);
    STM_EVAL_LEDOff(LED3);
    STM_EVAL_LEDOff(LED4);
    STM_EVAL_LEDOff(LED5);
    STM_EVAL_LEDOff(LED6);
    configureTim1Pins();
    TimerPeriod = (SystemCoreClock / pwmFreq / (prescaler + 1) / (division + 1)) - 1;
    tim3Config();
    tim1Config();
    while (1)
    {
    }
}


void disable(uint16_t bitMask) {
    TIM1->CCER &= ~bitMask;
}

void enable(uint16_t bitMask) {
    TIM1->CCER |= bitMask;
}

void enableDisable(uint16_t bitMask, uint8_t value) {
    if (value)
        enable(bitMask);
    else
        disable(bitMask);
}

void setChannel(uint8_t chanVal, uint8_t chanNVal, volatile uint32_t* CCRx, uint16_t ccOutputFlag, uint16_t ccNOutpoutFlag, uint16_t TIM_Channel) {
    uint16_t durationA = lroundf(TimerPeriod * attenuation);
    *CCRx = chanVal || chanNVal ? durationA : 0;
    enableDisable(ccOutputFlag, chanVal);
    enableDisable(ccNOutpoutFlag, chanNVal);
}

uint8_t stepFromAngle(float angle) {
    return (int)(rotorPosition * 6.0 / (2.0F * PI)) % 6;
}

void rampCleanup() {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);
    STM_EVAL_LEDOff(LED4);
    configMotorInputPins();
}

void 

void TIM1_CC_IRQHandler(void) {
    if (TIM_GetITStatus(TIM1, TIM_IT_CC1)) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
        uint8_t previousStep = stepFromAngle(rotorPosition);
        rotorPosition += 2.0F * PI / pwmFreq * rotationFreq;
        rotorPosition = fmod(rotorPosition, 2.0F * PI);
        uint8_t step = stepFromAngle(rotorPosition);
        
        if (step != previousStep) {
            STM_EVAL_LEDToggle(LED6);
            uint8_t *chans = stepConfiguration[step];
            setChannel(chans[0], chans[1], &(TIM1->CCR1), TIM_CCER_CC1E, TIM_CCER_CC1NE, TIM_Channel_1);
            setChannel(chans[2], chans[3], &(TIM1->CCR2), TIM_CCER_CC2E, TIM_CCER_CC2NE, TIM_Channel_2);
            setChannel(chans[4], chans[5], &(TIM1->CCR3), TIM_CCER_CC3E, TIM_CCER_CC3NE, TIM_Channel_3);
        }
    }
}

void TIM3_IRQHandler(void) {
    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
        STM_EVAL_LEDToggle(LED4);
        float32_t relativePosition = (float32_t)rampPosition / rampSteps;
        rotationFreq = minRotationFreq + (maxRotationFreq - minRotationFreq) * relativePosition;
        attenuation = startAttenuation + (finalAttenuation - startAttenuation) * relativePosition;
        rampPosition++;
        if (rampPosition >= rampSteps) {
            TIM_Cmd(TIM3, DISABLE);
            rampCleanup();
        }
    }
}

void EXTI9_5_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line7) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line7);
        if (GPIOC->IDR & GPIO_Pin_7) {
            STM_EVAL_LEDToggle(LED5);
        }
    }
}
