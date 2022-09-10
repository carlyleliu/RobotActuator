#include <zephyr/kernel.h>

#include <stm32_encoder.h>

#include <stm32_ll_bus.h>
#include <stm32_ll_exti.h>
#include <stm32_ll_gpio.h>
#include <stm32_ll_pwr.h>
#include <stm32_ll_system.h>
#include <stm32_ll_tim.h>

/**
  * @brief  Initialize GPIO registers according to the specified parameters in GPIO_InitStruct.
  * @param  GPIOx GPIO Port
  * @param  GPIO_InitStruct pointer to a @ref LL_GPIO_InitTypeDef structure
  *         that contains the configuration information for the specified GPIO peripheral.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: GPIO registers are initialized according to GPIO_InitStruct content
  *          - ERROR:   Not applicable
  */
ErrorStatus LL_GPIO_Init(GPIO_TypeDef *GPIOx, LL_GPIO_InitTypeDef *GPIO_InitStruct)
{
    uint32_t pinpos;
    uint32_t currentpin;

    /* ------------------------- Configure the port pins ---------------- */
    /* Initialize  pinpos on first pin set */
    pinpos = POSITION_VAL(GPIO_InitStruct->Pin);

    /* Configure the port pins */
    while (((GPIO_InitStruct->Pin) >> pinpos) != 0x00000000U)
    {
        /* Get current io position */
        currentpin = (GPIO_InitStruct->Pin) & (0x00000001UL << pinpos);

        if (currentpin != 0x00u)
        {
            if ((GPIO_InitStruct->Mode == LL_GPIO_MODE_OUTPUT) || (GPIO_InitStruct->Mode == LL_GPIO_MODE_ALTERNATE))
            {
                /* Check Speed mode parameters */
                assert_param(IS_LL_GPIO_SPEED(GPIO_InitStruct->Speed));

                /* Speed mode configuration */
                LL_GPIO_SetPinSpeed(GPIOx, currentpin, GPIO_InitStruct->Speed);

                /* Check Output mode parameters */
                assert_param(IS_LL_GPIO_OUTPUT_TYPE(GPIO_InitStruct->OutputType));

                /* Output mode configuration*/
                LL_GPIO_SetPinOutputType(GPIOx, GPIO_InitStruct->Pin, GPIO_InitStruct->OutputType);
            }

            /* Pull-up Pull down resistor configuration*/
            LL_GPIO_SetPinPull(GPIOx, currentpin, GPIO_InitStruct->Pull);

            if (GPIO_InitStruct->Mode == LL_GPIO_MODE_ALTERNATE)
            {
                /* Check Alternate parameter */
                assert_param(IS_LL_GPIO_ALTERNATE(GPIO_InitStruct->Alternate));

                /* Speed mode configuration */
                if (currentpin < LL_GPIO_PIN_8)
                {
                    LL_GPIO_SetAFPin_0_7(GPIOx, currentpin, GPIO_InitStruct->Alternate);
                }
                else
                {
                    LL_GPIO_SetAFPin_8_15(GPIOx, currentpin, GPIO_InitStruct->Alternate);
                }
            }

            /* Pin Mode configuration */
            LL_GPIO_SetPinMode(GPIOx, currentpin, GPIO_InitStruct->Mode);
        }
        pinpos++;
    }
    return (SUCCESS);
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
void Tim2Init(void)
{
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    /**TIM2 GPIO Configuration
    PA0   ------> TIM2_CH1
    PA1   ------> TIM2_CH2
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 4.294967295E9;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM2, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM2);
    LL_TIM_SetEncoderMode(TIM2, LL_TIM_ENCODERMODE_X4_TI12);
    LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
    LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
    LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM2);
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
void Tim3Init(void)
{
    LL_TIM_InitTypeDef TIM_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    /**TIM3 GPIO Configuration
    PA4   ------> TIM3_CH2
    PB4   ------> TIM3_CH1
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 65535;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM3, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM3);
    LL_TIM_SetEncoderMode(TIM3, LL_TIM_ENCODERMODE_X4_TI12);
    LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
    LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
    LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM3);
}

/**
  * @brief EncoderStart Function
  * @param None
  * @retval None
  */
void EncoderStart(uint8_t id)
{
    if (0 == id) {
        LL_TIM_EnableCounter(TIM2);
    } else if (1 == id) {
        LL_TIM_EnableCounter(TIM3);
    }
}

/**
  * @brief EncoderStop Function
  * @param None
  * @retval None
  */
void EncoderStop(uint8_t id)
{
    if (0 == id) {
        LL_TIM_DisableCounter(TIM2);
    } else if (1 == id) {
        LL_TIM_DisableCounter(TIM3);
    }
}

/**
  * @brief EncoderReadDirection Function
  * @param None
  * @retval None
  */
int16_t EncoderReadDirection(uint8_t id)
{
    int16_t dir = 0;

    if (0 == id) {
        dir = LL_TIM_GetDirection(TIM2);
    } else if (1 == id) {
        dir = LL_TIM_GetDirection(TIM3);
    }

    return dir;
}

/**
  * @brief EncoderReadCount Function
  * @param None
  * @retval None
  */
int16_t EncoderReadCount(uint8_t id)
{
    int16_t count = 0;

    if (0 == id) {
        count = LL_TIM_GetCounter(TIM2);
    } else if (1 == id) {
        count = LL_TIM_GetCounter(TIM3);
    }

    return count;
}
