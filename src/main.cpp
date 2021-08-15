/**
 * SPI 3 Wire usage demo with BMX160 sensor.
 *
 * sensor: https://wiki.dfrobot.com/BMX160_9_Axis_Sensor_Module_SKU_SEN0373
 *
 * The demo runs multiple times 3-Wire SPI test with different parameters and print result table.
 *
 * Single test includes the following steps:
 * 1. SPI object creation
 * 2. minimal BMX160 configuration (some commands should be send via SPI)
 * 3. single byte read/write test. It includes the following steps, that are repeated 64 times:
 *    1. start transaction (CS = 0)
 *    2. send register address (1 byte)
 *    3. send value (1 byte)
 *    4. finish transaction (CS = 1)
 *    5. start transaction (CS = 0)
 *    6. send register address with read bit flag (1 byte)
 *    7. receive value (1 byte)
 *    8. finish transaction (CS = 1)
 *    9. do the following checks:
 *       - check that sent and received values are same
 *       - check that SPI method calls don't return errors
 *       - check the SPI generated 32 clock cycles
 * 4. burst read/write test. It includes the following steps, that are repeated 64 times:
 *    1. start transaction (CS = 0)
 *    2. send register address (1 byte)
 *    3. send value (6 bytes)
 *    4. finish transaction (CS = 1)
 *    5. start transaction (CS = 0)
 *    6. send register address with read bit flag (1 byte)
 *    7. receive value (6 bytes)
 *    8. finish transaction (CS = 1)
 *    9. do the following checks:
 *       - check that sent and received values are same
 *       - check that SPI method calls don't return errors
 *       - check the SPI generated 112 clock cycles
 * 5. destroy SPI object
 *
 * Test parameters:
 * - target SPI frequency
 * - API type:
 *   - "synchronous" - SPI::write methods are used for communication
 *   - "asynchronous" - SPI::transfer method is used for communication
 *
 * Result table has the following columns with information about each test:
 * 1. "target SPI frequency" - SPI frequency that is set with SPI::frequency method
 * 2. "actual SPI frequency" - actual SPI frequency (it may differ from target one)
 * 3. "API type" - API type:
 *    - "synchronous" - SPI::write methods are used for communication
 *    - "asynchronous" - SPI::transfer method is used for communication
 * 4. "test result". It's "OK" if:
 *    - sensor is initialized without errors
 *    - single byte read/write test finished without errors
 *    - burst read/write test finished without errors
 * 5. "init error" - sensor initialization error
 *    - 0 - any SPI method calls that are used to perform basic BMX160 configuration don't return any error code
 *    - non-zero - some SPI method calls that are used to perform basic BMX160 configuration returns error code
 * 6. "single r/w result" - single byte read/write test result. It's "OK" if:
 *    - SPI api calls don't return any error codes
 *    - sent and received values coincide
 *    - SPI generated correct number of clock cycles
 * 7. "single r/w data error":
 *    - "no" - sent and received values coincide
 *    - "yes" - sent and received values are different
 * 8. "single r/w clock error":
 *    - "no" - SPI generated correct number of clock cycles
 *    - "yes" - SPI generated wrong number of clock cycles
 * 9. "burst r/w result" - burst byte read/write test result. It's "OK" if:
 *    - SPI api calls don't return any error codes
 *    - sent and received values coincide
 *    - SPI generated correct number of clock cycles
 * 10. "burst r/w data error":
 *    - "no" - sent and received values coincide
 *    - "yes" - sent and received values are different
 * 11. "burst r/w clock error":
 *    - "no" - SPI generated correct number of clock cycles
 *    - "yes" - SPI generated wrong number of clock cycles
 */
#include <cstdarg>
#include <cstring>

#include "mbed.h"
#include "pwmout_api.h"


//----------------------------------------------------------------------------//
// Hardware pins
//----------------------------------------------------------------------------//
// SPI MOSI pin
#define BMX160_SPI_MOSI PB_5
// SPI MISO pin
// This pin should be connected to BMX160_SPI_MOSI to use 4-wire mode for 3-wire spi device.
// If it isn't used, then NC should be set.
#define BMX160_SPI_MISO NC
// SPI CLK pin
#define BMX160_SPI_SCK PB_3
// SPI SSEL pin
#define BMX160_SPI_CSB PB_6
// SPI CLK counter pint
#define BMX160_SPI_CLK_COUNTER PA_0

//----------------------------------------------------------------------------//
// Helper SPI interface that able to print actual frequency
//----------------------------------------------------------------------------//
extern "C" {
extern int spi_get_clock_freq(spi_t *obj);
}

/**
 * Convert SPI_BAUDRATEPRESCALER_X constant into prescaler rank.
 */
static uint8_t spi_get_baudrate_prescaler_rank(uint32_t value)
{
    switch (value) {
        case SPI_BAUDRATEPRESCALER_2:
            return 0;
        case SPI_BAUDRATEPRESCALER_4:
            return 1;
        case SPI_BAUDRATEPRESCALER_8:
            return 2;
        case SPI_BAUDRATEPRESCALER_16:
            return 3;
        case SPI_BAUDRATEPRESCALER_32:
            return 4;
        case SPI_BAUDRATEPRESCALER_64:
            return 5;
        case SPI_BAUDRATEPRESCALER_128:
            return 6;
        case SPI_BAUDRATEPRESCALER_256:
            return 7;
        default:
            return 0xFF;
    }
}

class SPIExt : public SPI {
public:
    SPIExt(PinName mosi, PinName miso, PinName sclk)
            : SPI(mosi, miso, sclk) {}

    /**
     * Get actual SPI frequency;
     */
    int get_real_frequency()
    {
        spi_t *obj = &_peripheral->spi;
        SPI_HandleTypeDef *handle = &obj->spi.handle;
        int base_freq = spi_get_clock_freq(obj);
        return base_freq >> (spi_get_baudrate_prescaler_rank(handle->Init.BaudRatePrescaler) + 1);
    }
};

//----------------------------------------------------------------------------//
// Helper PWM base class to count signal edges using STM32 ETR TIM feature.
//----------------------------------------------------------------------------//

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

/**
 * Pulse counter.
 *
 * Unlike common interrupt base implementation it doesn't consume CPU time and
 * it able to work with higher frequencies.
 */
class PulseCounter : NonCopyable<PulseCounter> {
protected:
    PinName _pin;
    PWMName _pwm;

    static PWMName _get_timer(PinName pin)
    {
        return (PWMName)pinmap_peripheral(pin, PinMap_ETR);
    }

    static void _enable_tim_clock(PWMName pwm)
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

    static int _get_encoder_function(PinName pin, PinMode mode)
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


public:
    enum PulseEdge {
        RisingEdge = 0,
        FallingEdge = 1
    };

    /**
     * Constructor.
     *
     * @param pin PulseCounter pin to connect to
     * @param edge signal edge to count pulse
     */
    PulseCounter(PinName pin, PulseEdge edge = FallingEdge, PinMode mode = PullNone)
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

    ~PulseCounter()
    {
        TIM_HandleTypeDef htim = {};
        htim.Instance = (TIM_TypeDef *)(_pwm);

        // stop timer
        HAL_TIM_Base_Stop(&htim);
        // configure GPIO back to reset value
        pin_function(_pin, STM_PIN_DATA(STM_MODE_ANALOG, GPIO_NOPULL, 0));

        sleep_manager_unlock_deep_sleep();
    }

    void reset()
    {
        auto tim = (TIM_TypeDef *)(_pwm);
        LL_TIM_SetCounter(tim, 0);
    }

    int get_count()
    {
        auto tim = (TIM_TypeDef *)(_pwm);
        return (int)LL_TIM_GetCounter(tim);
    }

};

//----------------------------------------------------------------------------//
// BMX160 register API
//----------------------------------------------------------------------------//

enum class BMX160Register : uint8_t {
    CHIP_ID = 0x00,
    ERROR_REG = 0x02,

    // power status
    PMU_STATUS = 0x03,

    // data register
    DATA_MAG_X_L = 0x04,
    DATA_MAG_X_H = 0x05,
    DATA_MAG_Y_L = 0x06,
    DATA_MAG_Y_H = 0x07,
    DATA_MAG_Z_L = 0x08,
    DATA_MAG_Z_H = 0x09,
    DATA_RHALL_L = 0x0A,
    DATA_RHALL_H = 0x0B,
    DATA_GYRO_X_L = 0x0C,
    DATA_GYRO_X_H = 0x0D,
    DATA_GYRO_Y_L = 0x0E,
    DATA_GYRO_Y_H = 0x0F,
    DATA_GYRO_Z_L = 0x10,
    DATA_GYRO_Z_H = 0x11,
    DATA_ACCEL_X_L = 0x12,
    DATA_ACCEL_X_H = 0x13,
    DATA_ACCEL_Y_L = 0x14,
    DATA_ACCEL_Y_H = 0x15,
    DATA_ACCEL_Z_L = 0x16,
    DATA_ACCEL_Z_H = 0x17,
    SENSOR_TIME_0 = 0x18,
    SENSOR_TIME_1 = 0x19,
    SENSOR_TIME_2 = 0x1A,

    // status registers
    STATUS = 0x1B,
    INT_STATUS_0 = 0x1C,
    INT_STATUS_1 = 0x1D,
    INT_STATUS_2 = 0x1E,
    INT_STATUS_4 = 0x1F,

    // temperature
    TEMPERATURE_L = 0x20,
    TEMPERATURE_H = 0x21,

    // FIFO data
    FIFO_LENGTH_L = 0x22,
    FIFO_LENGTH_H = 0x22,
    FIFO_DATA = 0x24,

    // sensor configuration
    ACCEL_CONFIG = 0x40,
    ACCEL_RANGE = 0x41,
    GYRO_CONFIG = 0x42,
    GYRO_RANGE = 0x43,
    MAGN_CONFIG = 0x44,

    // fifo configuration
    FIFO_DOWN = 0x45,
    FIFO_CONFIG_0 = 0x46,
    FIFO_CONFIG_1 = 0x47,

    // manual magnetometer access registers
    MAG_IF_0 = 0x4C,
    MAG_IF_1 = 0x4D,
    MAG_IF_2 = 0x4E,
    MAG_IF_3 = 0x4F,

    // interrupt configuration registers
    INT_EN_0 = 0x50,
    INT_EN_1 = 0x51,
    INT_EN_2 = 0x52,
    INT_OUT_CTRL = 0x53,
    INT_LATCH = 0x54,
    INT_MAP_0 = 0x55,
    INT_MAP_1 = 0x56,
    INT_MAP_2 = 0x57,
    INT_DATA_0 = 0x58,
    INT_DATA_1 = 0x59,
    INT_LOWHIGH_0 = 0x5A,
    INT_LOWHIGH_1 = 0x5B,
    INT_LOWHIGH_2 = 0x5C,
    INT_LOWHIGH_3 = 0x5D,
    INT_LOWHIGH_4 = 0x5E,
    INT_MOTION_0 = 0x5F,
    INT_MOTION_1 = 0x60,
    INT_MOTION_2 = 0x61,
    INT_MOTION_3 = 0x62,
    INT_TAP_0 = 0x63,
    INT_TAP_1 = 0x64,
    INT_ORIENT_0 = 0x65,
    INT_ORIENT_1 = 0x66,
    INT_FLAT_0 = 0x67,
    INT_FLAT_1 = 0x68,

    // configuration registers
    FOC_CONF = 0x69,
    CONF = 0x6A,
    IF_CONF = 0x6B,
    SELF_TEST = 0x6D,
    NV_CONF = 0x70,

    // offset compensation registers
    OFFSET_ACCEL_X = 0x71,
    OFFSET_ACCEL_Y = 0x72,
    OFFSET_ACCEL_Z = 0x73,
    OFFSET_GYRO_X_L = 0x74,
    OFFSET_GYRO_Y_L = 0x75,
    OFFSET_GYRO_Z_L = 0x76,
    OFFSET_CONF = 0x77,

    // step counter
    STEP_CNT_L = 0x78,
    STEP_CNT_H = 0x78,
    STEP_CONFIG_0 = 0x7A,
    STEP_CONFIG_1 = 0x7B,

    // command register
    CMD = 0x7E,
};

/**
 * Helper wrapper around SPI object for communication with BMX160 sensor.
 *
 * It provides the following functionality:
 *
 * - SPI object configuration (mode, frame length and frequency)
 * - minimal BMX160 configuration for tests
 * - SPI transaction logging
 * - SPI synchronous/asynchronous API usage selection
 */
class BMX160SPI3WireAPI : NonCopyable<BMX160SPI3WireAPI> {
public:
    // SPI interface object and configuration
    static const int _SPI_MODE = 3;
    SPIExt _spi;
    int _spi_freq;
    DigitalOut _spi_ssel;

    // asynchronous API usage flags
    bool _use_async = false;
    EventFlags _async_operation_complete_flag;

    // helper logging data
    int _transaction_count = 0;
    bool _log = false;
    static constexpr int _LOG_BUF_SIZE = 128;
    char _log_buf[_LOG_BUF_SIZE];

    void _log_msg(const char *type, const char *msg, ...)
    {
        if (!_log) {
            return;
        }

        printf("[%s] ", type);
        va_list args;
        va_start(args, msg);
        vprintf(msg, args);
        va_end(args);
        printf("\n");
    }

    static char *_format_byte(char *str, int *buf_size, char prefix, uint8_t value)
    {
        if (*buf_size >= 4) {
            sprintf(str, "%c%02X", prefix, value);
            *buf_size -= 3;
            return str + 3;
        } else {
            // overflow
            str[-1] = '!';
            str[0] = '\0';
            return str;
        }
    }

    void _log_transaction(int err, uint8_t reg_addr_cmd, bool read_flag, const uint8_t *data, int len)
    {
        if (_log) {
            int buf_size = _LOG_BUF_SIZE;
            char *data_msg = _log_buf;
            char data_prefix = read_flag ? '<' : '>';
            data_msg = _format_byte(data_msg, &buf_size, '>', reg_addr_cmd);
            for (int i = 0; i < len; i++) {
                data_msg = _format_byte(data_msg, &buf_size, data_prefix, data[i]);
            }
            _log_msg("INFO", "spi transaction %2i; err = %2i; data: %s", _transaction_count, err, _log_buf);
        }
    }

    void _notify_flag(int event)
    {
        if (event == SPI_EVENT_COMPLETE) {
            // success
            _async_operation_complete_flag.set(0x1);
        } else {
            // error
            _async_operation_complete_flag.set(0x3);
        }

    }

    int _write_async(const uint8_t *data, int len)
    {
        _async_operation_complete_flag.clear();
        _spi.transfer<uint8_t>(data, len, nullptr, 0, callback(this, &BMX160SPI3WireAPI::_notify_flag),
                               SPI_EVENT_ERROR | SPI_EVENT_COMPLETE);
        uint32_t result = _async_operation_complete_flag.wait_any(0x3);
        return result == 0x01 ? 0 : -1;
    }

    int _read_async(uint8_t *data, int len)
    {
        _async_operation_complete_flag.clear();
        _spi.transfer<uint8_t>(nullptr, 0, data, len, callback(this, &BMX160SPI3WireAPI::_notify_flag),
                               SPI_EVENT_ERROR | SPI_EVENT_COMPLETE);
        uint32_t result = _async_operation_complete_flag.wait_any(0x3);
        return result == 0x01 ? 0 : -1;
    }

    int _write_sync(const uint8_t *data, int len)
    {
        int err = 0;
        if (len == 1) {
            _spi.write(data[0]);
        } else {
            int res = _spi.write((const char *)data, len, nullptr, 0);
            if (res != len) {
                err = -1;
            }
        }
        return err;
    }

    int _read_sync(const uint8_t *data, int len)
    {
        int err = 0;

        int res = _spi.write(nullptr, 0, (char *)data, len);
        if (res != len) {
            err = -1;
        }

        return err;
    }

public:
    /**
     * Constructor.
     *
     * @param mosi SPI MOSI pin
     * @param miso SPI MISO pin. It should be NC for 3 wire SPI
     * @param sclk SPI SCLK pin
     * @param ssel SPI SSEL pin
     * @param freq SPI frequency
     */
    BMX160SPI3WireAPI(PinName mosi, PinName miso, PinName sclk, PinName ssel, int freq = 10'000'00)
            : _spi(mosi, miso, sclk), _spi_freq(freq), _spi_ssel(ssel, 1)
    {
    }

    /**
     * Enable/disable logging.
     */
    void set_log(bool value)
    {
        _log = value;
    }

    /**
     * Enable/disable asynchronous API usage.
     * @param value
     */
    void set_async(bool value)
    {
        _use_async = value;
    }

    /**
     * Get actual SPI frequency.
     */
    int get_real_spi_frequency()
    {
        return _spi.get_real_frequency();
    }

    /**
     * Perform minimal BMX160 initialization for 3-wire testing.
     */
    int init()
    {
        return reset();
    }

    /**
     * Reset and initialize BMX160 for for 3-wire testing.
     */
    int reset()
    {
        int op_err, err;
        err = 0;

        // configure SPI
        _spi.format(8, _SPI_MODE);
        _spi.frequency(_spi_freq);

        // delay to be sure that bmx has started
        ThisThread::sleep_for(20ms);

        // create rising edge with SSEL to activate SPI
        _spi_ssel = 0;
        ThisThread::sleep_for(1ms);
        _spi_ssel = 1;
        ThisThread::sleep_for(1ms);

        // software reset
        if ((op_err = register_write(BMX160Register::CMD, 0xB6))) {
            err = op_err;
        }
        ThisThread::sleep_for(20ms);

        // create rising edge with SSEL to activate SPI
        _spi_ssel = 0;
        ThisThread::sleep_for(1ms);
        _spi_ssel = 1;
        ThisThread::sleep_for(1ms);

        // activate SPI 3-wire mode
        if ((op_err = register_write(BMX160Register::IF_CONF, 0x01))) {
            err = op_err;
        }

        // enable accelerometer to switch sensor to normal mode
        if ((op_err = register_write(BMX160Register::CMD, 0x11))) {
            err = op_err;
        }

        return err;
    }

    int register_write(BMX160Register reg, const uint8_t *data, int len)
    {
        int err_a, err_d;
        int err;

        uint8_t reg_addr_cmd = (uint8_t)reg;

        _spi_ssel = 0;
        if (!_use_async) {
            err_a = _write_sync(&reg_addr_cmd, 1);
            err_d = _write_sync(data, len);
        } else {
            err_a = _write_async(&reg_addr_cmd, 1);
            err_d = _write_async(data, len);
        }
        _spi_ssel = 1;

        err = err_a == 0 && err_d == 0 ? 0 : -1;

        // log transaction
        _log_transaction(err, reg_addr_cmd, false, data, len);
        _transaction_count++;

        return err;
    }

    int register_write(BMX160Register reg, uint8_t data)
    {
        return register_write(reg, &data, 1);
    }

    int register_read(BMX160Register reg, uint8_t *data, int len)
    {
        int err_a, err_d;
        int err;

        uint8_t reg_addr_cmd = (uint8_t)reg | 0x80;

        _spi_ssel = 0;
        if (!_use_async) {
            err_a = _write_sync(&reg_addr_cmd, 1);
            err_d = _read_sync(data, len);
        } else {
            err_a = _write_async(&reg_addr_cmd, 1);
            err_d = _read_async(data, len);
        }
        _spi_ssel = 1;

        err = err_a == 0 && err_d == 0 ? 0 : -1;

        // log transaction
        _log_transaction(err, reg_addr_cmd, true, data, len);
        _transaction_count++;

        return err;
    }

    int register_read(BMX160Register reg, uint8_t *data)
    {
        return register_read(reg, data, 1);
    }

    /**
     * Call SPI::abort_all_transfers method.
     */
    void call_spi_abort_all_transfers()
    {
        return _spi.abort_all_transfers();
    }
};


/**
 * SPI test results
 *
 * Note: in some cases we may transitive and receive, but dummy "reads" may be generated,
 * that is indicated with `*_rw_data_ok_count` and `*_rw_clock_ok_count` fields
 */
struct spi_3_wire_test_result_t {
    /** If during BMX160 configuration any SPI object methods returns an error, it will be saved here */
    int init_error;

    /** Total number of tests with single byte read/write */
    int single_rw_count;
    /** Total number of tests with single byte read/write that succeed */
    int single_rw_ok_count;
    /** Total number of tests with single byte read/write that returns correct data */
    int single_rw_data_ok_count;
    /** Total number of tests with single byte read/write that has correct number of clock cycles */
    int single_rw_clock_ok_count;

    /** Total number of tests with multiple byte read/write */
    int burst_rw_count;
    /** Total number of tests with multiple byte read/write that succeed */
    int burst_rw_ok_count;
    /** Total number of tests with multiple byte read/write that returns correct data */
    int burst_rw_data_ok_count;
    /** Total number of tests with multiple byte read/write that has correct number of clock cycles */
    int burst_rw_clock_ok_count;

    /** Asynchronous API usage flag */
    bool async_api;
    /** Target SPI frequency */
    int target_spi_frequency;
    /** Actual SPI frequency */
    int actual_spi_frequency;
};


/**
 * BMX160 sensor based SPI 3 wire tester.
 */
class BMX160SPI3WireTester : NonCopyable<BMX160SPI3WireTester> {
    static constexpr int DEFAULT_TEST_NUMBER = 64;
    static constexpr int DEFAULT_SPI_FREQ = 10'000'000;
protected:
    PinName _spi_mosi_pin;
    PinName _spi_miso_pin;
    PinName _spi_sclk_pin;
    PinName _spi_ssel_pin;
    PinName _spi_sclk_counter_pin;

    int _single_rw_test_number = DEFAULT_TEST_NUMBER;
    int _burst_rw_test_number = DEFAULT_TEST_NUMBER;
    int _spi_freq = DEFAULT_SPI_FREQ;
    int _spi_async_api_usage;

    bool _log = false;

    void _log_msg(const char *type, const char *msg, ...)
    {
        if (!_log) {
            return;
        }

        printf("[%s] ", type);
        va_list args;
        va_start(args, msg);
        vprintf(msg, args);
        va_end(args);
        printf("\n");
    }

public:
    /**
     * Constructor.
     *
     * @param spi_mosi SPI MOSI pin
     * @param spi_miso SPI MISO pin. Use NC in case of 3-wire spi usage. If MCU should use 4-wire SPI whereas sensor 3-wire SPI, then connect MISO and MOSI pins.
     * @param spi_sclk SPI clock pin
     * @param spi_ssel SPI SSEL pin
     * @param spi_sclk_counter SPI clock counter pin. It should be connected to spi_sclk pin
     */
    BMX160SPI3WireTester(PinName spi_mosi, PinName spi_miso, PinName spi_sclk, PinName spi_ssel,
                         PinName spi_sclk_counter)
            : _spi_mosi_pin(spi_mosi), _spi_miso_pin(spi_miso), _spi_sclk_pin(spi_sclk), _spi_ssel_pin(spi_ssel),
              _spi_sclk_counter_pin(spi_sclk_counter)
    {
    }

    /**
     * Enable/disable logging.
     */
    void set_log(bool value)
    {
        _log = value;
    }

    /**
     * Set test parameters.
     *
     * @param spi_freq target spi frequency
     * @param async_api_usage asynchronous API
     * @param single_rw_test_number number of tests with single byte read/write
     * @param burst_rw_test_number number of tests with multiple bytes read/write
     */
    void configure(int spi_freq, bool async_api_usage,
                   int single_rw_test_number = DEFAULT_TEST_NUMBER, int burst_rw_test_number = DEFAULT_TEST_NUMBER)
    {
        _single_rw_test_number = single_rw_test_number;
        _burst_rw_test_number = burst_rw_test_number;
        _spi_freq = spi_freq;
        _spi_async_api_usage = async_api_usage;
    }

    /**
     * Execute tests.
     */
    int test(spi_3_wire_test_result_t *test_result)
    {
        int api_init_err;
        uint8_t out_buf[6];
        uint8_t in_buf[6];


        BMX160SPI3WireAPI *api = new BMX160SPI3WireAPI(
                _spi_mosi_pin, _spi_miso_pin, _spi_sclk_pin, _spi_ssel_pin,
                _spi_freq);
        PulseCounter *pc = new PulseCounter(_spi_sclk_counter_pin, PulseCounter::RisingEdge, PullNone);


        _log_msg("INFO", "================= start test =================");
        api->set_async(_spi_async_api_usage);
        api->set_log(_log);
        if ((api_init_err = api->init())) {
            _log_msg("ERROR", "API initialization error %i", api_init_err);
        }

        // log main information
        int actual_spi_freq = api->get_real_spi_frequency();
        _log_msg("INFO", "target SPI frequency: %i Hz", _spi_freq);
        _log_msg("INFO", "actual SPI frequency: %i Hz", actual_spi_freq);
        _log_msg("INFO", "SPI API type: %s", _spi_async_api_usage ? "asynchronous" : "synchronous");

        // initialize result structure
        test_result->init_error = api_init_err;
        test_result->single_rw_count = _single_rw_test_number;
        test_result->single_rw_ok_count = 0;
        test_result->single_rw_data_ok_count = 0;
        test_result->single_rw_clock_ok_count = 0;
        test_result->burst_rw_count = _single_rw_test_number;
        test_result->burst_rw_ok_count = 0;
        test_result->burst_rw_data_ok_count = 0;
        test_result->burst_rw_clock_ok_count = 0;
        test_result->async_api = _spi_async_api_usage;
        test_result->target_spi_frequency = _spi_freq;
        test_result->actual_spi_frequency = actual_spi_freq;

        _log_msg("INFO", "test single read/write");
        out_buf[0] = 0;
        auto single_data_updater = [](uint8_t *data) {
            data[0] += 7;
        };
        _rw_test_impl(api, pc, test_result->single_rw_count,
                      out_buf, in_buf, 1, single_data_updater,
                      &test_result->single_rw_ok_count,
                      &test_result->single_rw_data_ok_count,
                      &test_result->single_rw_clock_ok_count
        );

        _log_msg("INFO", "test burst read/write");
        memset(out_buf, 0, sizeof(out_buf));
        auto burst_data_updater = [](uint8_t *data) {
            for (int i = 0; i < 6; i++) {
                data[i] += i + 1;
            }
        };
        _rw_test_impl(api, pc, test_result->burst_rw_count,
                      out_buf, in_buf, 6, burst_data_updater,
                      &test_result->burst_rw_ok_count,
                      &test_result->burst_rw_data_ok_count,
                      &test_result->burst_rw_clock_ok_count
        );


        delete pc;
        delete api;
        _log_msg("INFO", "================ complete test ================");
        return 0;
    }

    static char *_format_bytes(char *buf, const uint8_t *data, size_t len)
    {
        char *buf_ptr = buf;
        for (size_t i = 0; i < len; i++) {
            sprintf(buf_ptr, "%02X", data[i]);
            buf_ptr += 2;
        }
        return buf;
    }

    int _rw_test_impl(BMX160SPI3WireAPI *api, PulseCounter *pc, int total_tests,
                      uint8_t *out_buf, uint8_t *in_buf, size_t buf_len, Callback<void(uint8_t *data)> buf_updater,
                      int *ok_count, int *data_ok_count, int *clock_ok_count)
    {
        BMX160Register base_reg = BMX160Register::OFFSET_ACCEL_X;
        int err_read, err_write, err_data, err_clock;
        int actual_clock_count;
        int expected_clock_count = (buf_len + 1) * 2 * 8;
        char in_buf_msg[16];
        char out_buf_msg[16];

        MBED_ASSERT(buf_len <= 6);

        for (int i = 0; i < total_tests; i++) {
            // clear input buffer and clock counter
            memset(in_buf, 0, buf_len);
            pc->reset();

            // call SPI:spi_abort_all_transfers to check that it has no side effects
            api->call_spi_abort_all_transfers();

            // transmit data to BMX160
            err_write = api->register_write(base_reg, out_buf, buf_len);
            // receive data from BMX160
            err_read = api->register_read(base_reg, in_buf, buf_len);

            // get actual number of clock cycles
            actual_clock_count = pc->get_count();

            // check data correctness
            if (memcmp(out_buf, in_buf, buf_len) == 0) {
                err_data = 0;
                (*data_ok_count)++;
            } else {
                err_data = -1;
                _log_msg("ERROR", "transmitted and received data differs: 0x%s != 0x%s",
                         _format_bytes(out_buf_msg, out_buf, buf_len),
                         _format_bytes(in_buf_msg, in_buf, buf_len));
            }

            // check clock cycles
            if (actual_clock_count == expected_clock_count) {
                err_clock = 0;
                (*clock_ok_count)++;
            } else {
                err_clock = -1;
                _log_msg("ERROR", "expected %i SPI clock cycles, but got %i", expected_clock_count, actual_clock_count);
            }

            // check read/write error
            if (err_read) {
                _log_msg("ERROR", "reading from register error %i", err_read);
            }
            if (err_write) {
                _log_msg("ERROR", "writing to register error %i", err_read);
            }

            // check overall result
            if (err_read == 0 && err_write == 0 && err_data == 0 && err_clock == 0) {
                (*ok_count)++;
            }

            // update test data
            buf_updater(out_buf);
        }

        return 0;
    }
};


struct test_conf_t {
    int spi_freq;
    int async_api_usage;

    spi_3_wire_test_result_t result;
};


int main()
{
    static DigitalOut user_led(LED1, 1);
    static BMX160SPI3WireTester tester(
            BMX160_SPI_MOSI, BMX160_SPI_MISO, BMX160_SPI_SCK, BMX160_SPI_CSB,
            BMX160_SPI_CLK_COUNTER
    );
    tester.set_log(true);

    static test_conf_t tests_conf[] = {
            {200'000,    false}, // 200 KHz, synchronous API
            {1'000'000,  false}, // 1 MHz, synchronous API
            {12'500'000, false}, // 12.5 MHz, synchronous API
            {200'000,   true}, // 200 KHz, asynchronous API
            {1'000'000, true}, // 1 MHz, asynchronous API
            {12'500'000, true}, // 12.5 MHz, asynchronous API
    };

    // run tests
    for (auto &test_conf: tests_conf) {
        tester.configure(test_conf.spi_freq, test_conf.async_api_usage);
        tester.test(&test_conf.result);
        // delay between test
        ThisThread::sleep_for(1s);
    }

    printf("================== test results ===================\n");
    // print test results as table
    printf("| target SPI frequency | actual SPI frequency |   API type   | test result | init error | single r/w result | single r/w data error | single r/w clock error | burst r/w result | burst r/w data error | burst r/w clock error |\n");
    printf("| -------------------- | -------------------- | ------------ | ----------- | ---------- | ----------------- | --------------------- | ---------------------- | ---------------- | -------------------- | --------------------- |\n");
    for (auto &test_conf: tests_conf) {
        bool single_rw_ok = test_conf.result.single_rw_count == test_conf.result.single_rw_ok_count;
        bool single_rw_data_error = test_conf.result.single_rw_count != test_conf.result.single_rw_data_ok_count;
        bool single_rw_clock_error = test_conf.result.single_rw_count != test_conf.result.single_rw_clock_ok_count;
        bool burst_rw_ok = test_conf.result.burst_rw_count == test_conf.result.burst_rw_ok_count;
        bool burst_rw_data_error = test_conf.result.burst_rw_count != test_conf.result.burst_rw_data_ok_count;
        bool burst_rw_clock_error = test_conf.result.burst_rw_count != test_conf.result.burst_rw_clock_ok_count;

        printf("| %17i Hz | %17i Hz | %12s | %11s | %10i | %17s | %21s | %22s | %16s | %20s | %21s |\n",
               test_conf.result.target_spi_frequency, test_conf.result.actual_spi_frequency,
               test_conf.result.async_api ? "asynchronous" : "synchronous",
               single_rw_ok && burst_rw_ok && (test_conf.result.init_error == 0) ? "OK" : "ERROR",
               test_conf.result.init_error,

               single_rw_ok ? "OK" : "ERROR",
               single_rw_data_error ? "yes" : "no",
               single_rw_clock_error ? "yes" : "no",

               burst_rw_ok ? "OK" : "ERROR",
               burst_rw_data_error ? "yes" : "no",
               burst_rw_clock_error ? "yes" : "no"
        );
    }

    printf("=================== blink demo ====================\n");
    while (true) {
        ThisThread::sleep_for(500ms);
        user_led = !user_led;
    }

    return 0;
}
