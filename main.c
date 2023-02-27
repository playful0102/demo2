/*log
Tested USART3 and successfully send out character
SPI2.c change the char c in unsigned
SPI2.c change all PB13 to PA5,PB15 to PA7
SPI2.c change SPI2 to SPI1
SPI2.c change RCC_APB1ENR_SPI2EN to RCC_APB2ENR_SPI1EN
*/
#include "stm32f10x.h"
unsigned char state = 0;
unsigned char c;
static __IO uint32_t msTicks;
void DelayMs(uint32_t ms);
void readFloor();
int state = 1;

void SystemInit(){
    // Update SystemCoreClock value
    SystemCoreClockUpdate();
    // Configure the SysTick timer to overflow every 1 ms
    SysTick_Config(SystemCoreClock / 1000);
}

void spi1Init(){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);

    RCC_APB2PeriphClockCmd(RCC_APB2ENR_SPI1EN, ENABLE);

    SPI_InitTypeDef SPI_InitStructure;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    // 36 MHz / 256 = 140.625 kHz
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(SPI1, &SPI_InitStructure);
    // Enable the receive interrupt
    SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
    // Enable SPI1
    SPI_Cmd(SPI1, ENABLE);
}

void usart3Init(){
    // Enable USART3 and GPIOB clocks
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // Configure PB10 and PB11 as USART3 pins
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Initialize USART3
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);
}

void usart2Init(){
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // Configure PA2 and PA3 as USART3 pins
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    // Enable USART2,USART23
    USART_Cmd(USART3, ENABLE);
    USART_Cmd(USART2, ENABLE);
}

void wheelInit(){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    // GPIO set up for PA8(green) PA9(blue) PA10(red)
    // GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // GPIO set up for PB13
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    // PB6 LED
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Timer 1 set up
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseInitTypeDef timerInitStructure;
    timerInitStructure.TIM_Prescaler = 720 - 1; // 1/(72Mhz/720)=0.01ms=0.00001s
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 200 - 1; // 0.00001s*200=0.002s=1/500Hz
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &timerInitStructure);
    TIM_Cmd(TIM1, ENABLE);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    // http://news.eeworld.com.cn/mcu/ic471919.html
}

void wheelControl(int leftWheel, int rightWheel){
    //Foward;
    // Enable Tim1 CH1 PWM PA8 PA13
    TIM_OCInitTypeDef CH1;
    CH1.TIM_OCMode = TIM_OCMode_PWM1;
    CH1.TIM_Pulse = 188; //
    CH1.TIM_OutputState = TIM_OutputState_Enable;
    CH1.TIM_OCPolarity = TIM_OCPolarity_High;
    // CH1.TIM_OCIdleState=TIM_OCIdleState_Reset;
    CH1.TIM_OutputNState = TIM_OutputNState_Disable; // N
    CH1.TIM_OCNPolarity = TIM_OCNPolarity_Low;       // N
    // CH1.TIM_OCNIdleState=TIM_OCNIdleState_Reset;//N
    TIM_OC1Init(TIM1, &CH1); // PA8

    TIM_OCInitTypeDef CH2;
    CH2.TIM_OCMode = TIM_OCMode_PWM1;
    CH2.TIM_Pulse = 188; //
    CH2.TIM_OutputState = TIM_OutputState_Disable;
    CH2.TIM_OCPolarity = TIM_OCPolarity_Low;
    // CH2.TIM_OCIdleState=TIM_OCIdleState_Reset;
    CH2.TIM_OutputNState = TIM_OutputNState_Enable; // N
    CH2.TIM_OCNPolarity = TIM_OCNPolarity_High;     // N
    // CH2.TIM_OCNIdleState=TIM_OCNIdleState_Reset;//N
    TIM_OC2Init(TIM1, &CH2); // PA9

    DelayMs(200);

    // Enable Tim1 CH1 PWM PA8 PA13
    CH1.TIM_OCMode = TIM_OCMode_PWM1;
    CH1.TIM_Pulse = 0; //
    CH1.TIM_OutputState = TIM_OutputState_Enable;
    CH1.TIM_OCPolarity = TIM_OCPolarity_High;
    // CH1.TIM_OCIdleState=TIM_OCIdleState_Reset;
    CH1.TIM_OutputNState = TIM_OutputNState_Disable; // N
    CH1.TIM_OCNPolarity = TIM_OCNPolarity_Low;       // N
    // CH1.TIM_OCNIdleState=TIM_OCNIdleState_Reset;//N
    TIM_OC1Init(TIM1, &CH1); // PA8

    CH2.TIM_OCMode = TIM_OCMode_PWM1;
    CH2.TIM_Pulse = 0; //
    CH2.TIM_OutputState = TIM_OutputState_Disable;
    CH2.TIM_OCPolarity = TIM_OCPolarity_Low;
    // CH2.TIM_OCIdleState=TIM_OCIdleState_Reset;
    CH2.TIM_OutputNState = TIM_OutputNState_Enable; // N
    CH2.TIM_OCNPolarity = TIM_OCNPolarity_High;     // N
    // CH2.TIM_OCNIdleState=TIM_OCNIdleState_Reset;//N
    TIM_OC2Init(TIM1, &CH2); // PA9
}

int main(void)
{
    SystemInit();
    spi1Init();
    usart2Init();
    usart3Init();
    wheelInit();
    while (1)
    {
        readFloor();
        DelayMs(3); // can not too fast, maybe need to clear some flag

        // debug and test code
        // SPI_I2S_SendData(SPI1,c);

        // GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
        // DelayMs(3);
        // GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
        // DelayMs(3);
        switch(state){
            case 1:
                if(state == 1)
                {
                    wheelControl(100,100);
                    state++;
                }
                break;
            case 2:
                if (state == 2)
                {
                    wheelControl(0,0);
                    state++;
                }
                break;
        }
    }
}

void readFloor()
{
    // Set PA7 to 1
    GPIO_SetBits(GPIOA, GPIO_Pin_7);
    // Initialize the data transmission from the master to the slave
    SPI_I2S_SendData(SPI1, 0);
    // Enable the interrupt to receive data by using the ISR handler
    NVIC_EnableIRQ(SPI1_IRQn);
}

char getFloorReading(char reading){
    return reading;
}

void SPI1_IRQHandler()
{
    // put the readings to the variable c
    // the received character has all the readings
    c = (char)SPI_I2S_ReceiveData(SPI1) & 0xff;
    getFloorReading(c);
    if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_7) == 1)
    {
        // Set PA7 to 0 to trigger the shift register
        GPIO_ResetBits(GPIOA, GPIO_Pin_7);
        // Go to get the next reading
        SPI_I2S_SendData(SPI1, 0);
    }
    else
    {
        // debug and test code
        {
            if (c & (1 << 0))
            {
                while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
                    ;
                USART_SendData(USART3, '0');
            }
            else
            {
                while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
                    ;
                USART_SendData(USART3, '1');
            }
            if (c & (1 << 1))
            {
                while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
                    ;
                USART_SendData(USART3, '0');
            }
            else
            {
                while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
                    ;
                USART_SendData(USART3, '1');
            }
            if (c & (1 << 2))
            {
                while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
                    ;
                USART_SendData(USART3, '0');
            }
            else
            {
                while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
                    ;
                USART_SendData(USART3, '1');
            }
            if (c & (1 << 3))
            {
                while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
                    ;
                USART_SendData(USART3, '0');
            }
            else
            {
                while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
                    ;
                USART_SendData(USART3, '1');
            }
            if (c & (1 << 4))
            {
                while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
                    ;
                USART_SendData(USART3, '0');
            }
            else
            {
                while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
                    ;
                USART_SendData(USART3, '1');
            }
            if (c & (1 << 5))
            {
                while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
                    ;
                USART_SendData(USART3, '0');
            }
            else
            {
                while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
                    ;
                USART_SendData(USART3, '1');
            }
            if (c & (1 << 6))
            {
                while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
                    ;
                USART_SendData(USART3, '0');
            }
            else
            {
                while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
                    ;
                USART_SendData(USART3, '1');
            }
            if (c & (1 << 7))
            {
                while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
                    ;
                USART_SendData(USART3, '0');
            }
            else
            {
                while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
                    ;
                USART_SendData(USART3, '1');
            }

            while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
            USART_SendData(USART3, c);
            while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
            USART_SendData(USART3, '\r');
            while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
            USART_SendData(USART3, '\n');
        }

        // Check PA7. If it is 1, it means the data is ready

        // disable the interrupt because it is not ready
        NVIC_DisableIRQ(SPI1_IRQn);
    }
}

void DelayMs(uint32_t ms)
{
    // Reload ms value
    msTicks = ms;
    // Wait until msTick reach zero
    while (msTicks)
        ;
}

void SysTick_Handler()
{
    // SysTick_Handler function will be called every 1 ms
    if (msTicks != 0)
    {
        msTicks--;
    }
}
