#include "app_pulse_counter.h"

/**
 * TIM ETR pinmap.
 *
 * It's difficult to create TIM ETR input map for all MCUs, so we cover only most common cases.
 * Note: this pinmap doesn't cover all cases and may not work with some boards/mcus.
 */
static const PinMap PinMap_ETR[] = {
#if defined(TARGET_STM32F0) || defined(TARGET_STM32G0)
        {PA_0, PWM_2, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF2_TIM2)},
        {PA_5, PWM_2, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF2_TIM2)},
        {PA_12, PWM_1, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF2_TIM1)},
#elif defined (TARGET_STM32F1)
        {PA_0, PWM_2, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0)},
        {PA_12, PWM_1, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0)},
        {PA_15, PWM_2, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 8)},
#elif defined(TARGET_STM32F2) || defined(TARGET_STM32F2) || defined(TARGET_STM32F4) || defined(TARGET_STM32F7) || defined(TARGET_STM32H7)
        {PA_0, PWM_2, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF1_TIM2)},
        {PA_5, PWM_2, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF1_TIM2)},
        {PA_12, PWM_1, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF1_TIM1)},
        {PA_15, PWM_2, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF1_TIM2)},
#elif defined(TARGET_STM32G4)
        {PA_0, PWM_2, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF14_TIM2)},
        {PA_5, PWM_2, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF2_TIM2)},
        {PA_12, PWM_1, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF11_TIM1)},
        {PA_15, PWM_2, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF14_TIM2)},
        {PB_3, PWM_3, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF10_TIM3)},
#elif defined(TARGET_STM32L0)
        {PA_0, PWM_2, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF5_TIM2)},
        {PA_5, PWM_2, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF2_TIM2)},
#elif defined(TARGET_STM32L4) || defined(TARGET_STM32L5)
        {PA_0, PWM_2, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF14_TIM2)},
        {PA_5, PWM_2, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF2_TIM2)},
        {PA_12, PWM_1, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF1_TIM1)},
        {PA_15, PWM_2, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF2_TIM2)},
#elif defined(TARGET_STM32WB) || defined(TARGET_STM32WL)
        {PA_0, PWM_2, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF14_TIM2)},
        {PA_5, PWM_2, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF2_TIM2)},
        {PA_12, PWM_1, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF1_TIM1)},
        {PA_15, PWM_2, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF2_TIM2)},
#else
#error unsupported STM32 target for EdgeCounter
#endif
        {NC, NC, 0}
};

PWMName PulseCounter::_get_timer(PinName pin)
{
    return (PWMName)pinmap_peripheral(pin, PinMap_ETR);
}

void PulseCounter::_enable_tim_clock(PWMName pwm)
{
    // Enable TIM clock
    switch (pwm) {
#if defined(TIM1_BASE)
        case PWM_1:
            __HAL_RCC_TIM1_CLK_ENABLE();
            break;
#endif
#if defined(TIM2_BASE)
        case PWM_2:
            __HAL_RCC_TIM2_CLK_ENABLE();
            break;
#endif
#if defined(TIM3_BASE)
        case PWM_3:
            __HAL_RCC_TIM3_CLK_ENABLE();
            break;
#endif
#if defined(TIM4_BASE)
        case PWM_4:
            __HAL_RCC_TIM4_CLK_ENABLE();
            break;
#endif
        default:
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_PINMAP_INVALID),
                       "Specified timer isn't supported by EdgeCounter");
    }
}

int PulseCounter::_get_encoder_function(PinName pin, PinMode mode)
{
    int function = (int)pinmap_find_function(pin, PinMap_ETR);
    if (function == NC) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_PINMAP_INVALID),
                   "Invalid TIM ETR pin for EdgeCounter");
    }
    // update pull up/down method
    int hal_mode;
    switch (mode) {
        case PullUp:
        case OpenDrainPullUp:
            hal_mode = GPIO_PULLUP;
            break;
        case PullDown:
        case OpenDrainPullDown:
            hal_mode = GPIO_PULLDOWN;
            break;
        default:
            hal_mode = GPIO_NOPULL;
    }
    function &= ~STM_PIN_PUPD_BITS;
    function |= (hal_mode & STM_PIN_PUPD_MASK) << STM_PIN_PUPD_SHIFT;

    return function;
}

PulseCounter::PulseCounter(PinName pin, PulseCounter::PulseEdge edge, PinMode mode)
        : _pin(pin)
{
    // find target timer
    _pwm = _get_timer(pin);
    // get pwm function (with channel information)
    int function = _get_encoder_function(pin, mode);

    sleep_manager_lock_deep_sleep();

    // enable timer clock
    _enable_tim_clock(_pwm);

    // Configure TIM ETR mode
    TIM_HandleTypeDef htim = {};
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    htim.Instance = (TIM_TypeDef *)(_pwm);
    htim.Init.Prescaler = 0;
    htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim.Init.Period = 0xFFFF'FFFF;
    htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim.Init.RepetitionCounter = 0;
    htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim) != HAL_OK) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_INITIALIZATION_FAILED),
                   "Fail to configure timer for EdgeCounter");
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
    sClockSourceConfig.ClockPolarity =
            edge == RisingEdge ? TIM_CLOCKPOLARITY_NONINVERTED : TIM_CLOCKPOLARITY_INVERTED;
    sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
    sClockSourceConfig.ClockFilter = 0;
    if (HAL_TIM_ConfigClockSource(&htim, &sClockSourceConfig) != HAL_OK) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_INITIALIZATION_FAILED),
                   "Fail to configure clock source for EdgeCounter");
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig) != HAL_OK) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_INITIALIZATION_FAILED),
                   "Fail to configure master mode for EdgeCounter");
    }

    // enable pin
    pin_function(pin, function);

    // run timer
    if (HAL_TIM_Base_Start(&htim) != HAL_OK) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_INITIALIZATION_FAILED),
                   "Fail to start timer for EdgeCounter");
    }

}

PulseCounter::~PulseCounter()
{
    TIM_HandleTypeDef htim = {};
    htim.Instance = (TIM_TypeDef *)(_pwm);

    // stop timer
    HAL_TIM_Base_Stop(&htim);
    // configure GPIO back to reset value
    pin_function(_pin, STM_PIN_DATA(STM_MODE_ANALOG, GPIO_NOPULL, 0));

    sleep_manager_unlock_deep_sleep();
}

void PulseCounter::reset()
{
    auto tim = (TIM_TypeDef *)(_pwm);
    LL_TIM_SetCounter(tim, 0);
}

int PulseCounter::get_count()
{
    auto tim = (TIM_TypeDef *)(_pwm);
    return (int)LL_TIM_GetCounter(tim);
}
