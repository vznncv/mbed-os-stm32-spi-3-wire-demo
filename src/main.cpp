/**
 * SPI 3 Wire usage demo with BMX160 sensor.
 *
 * sensor: https://wiki.dfrobot.com/BMX160_9_Axis_Sensor_Module_SKU_SEN0373
 *
 * The demo runs multiple times 3-Wire SPI test with different parameters and print result table.
 *
 * The test cases are defined by test_configurations.
 *
 * Single test case includes the following steps:
 * 1. SPI object creation and configuration
 * 2. minimal BMX160 configuration (some commands should be send via SPI)
 * 3. Repeating writing and reading of data (64 times):
 *    1. start transaction (CS = 0)
 *    2. send register address (1 byte, 8-bit mode)
 *    3. send value (one or more words in 8 or 16 bit modes (up to 6 bytes))
 *    4. finish transaction (CS = 1)
 *    5. start transaction (CS = 0)
 *    6. send register address with read bit flag (1 byte)
 *    7. receive value (one or more words in 8 or 16 bit modes (up to 6 bytes))
 *    8. finish transaction (CS = 1)
 *    9. do the following checks:
 *       - check that sent and received values are same
 *       - check that SPI method calls don't return errors
 *       - check the SPI generated 32 clock cycles
 * 4. destroy SPI object
 *
 * Test parameters:
 * - target SPI frequency
 * - API type:
 *   - API_USAGE_SYNC - SPI::write methods are used for communication
 *   - API_USAGE_ASYNC - SPI::transfer method is used for communication
 * - test case type:
 *   - SINGLE_8W_8R - write 1 8-bit word and read 1 8-bit word
 *   - BURST_8W_8R -  write 6 8-bit words and read 6 8-bit words
 *   - SINGLE_16W_8R - write 1 16-bit word and read 2 8-bit words
 *   - BURST_16W_8R - write 3 16-bit words and read 6 8-bit words
 *   - SINGLE_8W_16R - write 2 8-bit words and read 1 16-bit word
 *   - BURST_8W_16R - write 6 8-bit words and read 2 16-bit words
 *   note: mixed 8/16-bit operations are used to check that bytes are swapped in the 16-bit mode.
 *
 * Result table has the following columns with information about each test:
 * 1. test number
 * 2. "api" - test api (async/sync)
 * 3. "case name" - it correspond test case type (see "Test parameters" above)
 * 4. "target freq (Hz)" - SPI frequency that is set with SPI::frequency method
 * 5. "target freq (Hz)" - actual SPI frequency (it may differ from target one)
 * 6. "init error" - sensor initialization error
 * 7. "result" - overall test result. It's "success" if:
 *    - SPI api calls don't return any error codes
 *    - sent and received values coincide
 *    - SPI generated correct number of clock cycles
 * 8. "data" - it's "success" if received and send data are same (with byte order correction).
 * 9. "clock" - it's "success" if SPI generated correct number of clock ticks.
 */
#include <cstdarg>
#include <cstring>
#include <utility>
#include <type_traits>
#include <chrono>

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
// SPI CLK counter pin. It should be connected to SPI CLK pin
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

/**
 * Helper SPI extension for STM32.
 *
 * It provides method ::get_real_frequency to get actual SPI frequency.
 */
class SPIExt : public SPI {
public:
    SPIExt(PinName mosi, PinName miso, PinName sclk)
            : SPI(mosi, miso, sclk)
    {}

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


/**
 * Helper wrapper around SPI object for communication with BMX160 sensor.
 *
 * It provides the following functionality:
 *
 * - SPI object configuration (mode, frame length)
 * - minimal BMX160 configuration for tests
 * - SPI transaction logging
 * - SPI synchronous/asynchronous API usage selection
 */
class BMX160SPI3WireAPITester : NonCopyable<BMX160SPI3WireAPITester> {
public:
    // Minimal BMX registers that are needed for testing
    enum class BMX160Register : uint8_t {
        // interface configuration register
        IF_CONF = 0x6B,
        // offset compensation registers
        // they aren't used by default, so we can use them to store testing data
        OFFSET_ACCEL_X = 0x71,
        OFFSET_ACCEL_Y = 0x72,
        OFFSET_ACCEL_Z = 0x73,
        OFFSET_GYRO_X_L = 0x74,
        OFFSET_GYRO_Y_L = 0x75,
        OFFSET_GYRO_Z_L = 0x76,
        // command register
        CMD = 0x7E,
    };

    // SPI interface object and configuration
    static const int _SPI_MODE = 3;
    SPI *_spi;
    DigitalOut *_spi_ssel;

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

    int _write_async(const uint8_t *data, int len, int bits)
    {
        _async_operation_complete_flag.clear();
        _spi->transfer<uint8_t>(data, len, nullptr, 0, callback(this, &BMX160SPI3WireAPITester::_notify_flag),
                                SPI_EVENT_ERROR | SPI_EVENT_COMPLETE);
        uint32_t result = _async_operation_complete_flag.wait_any(0x3);
        return result == 0x01 ? 0 : -1;
    }

    int _read_async(uint8_t *data, int len, int bits)
    {
        _async_operation_complete_flag.clear();
        _spi->transfer<uint8_t>(nullptr, 0, data, len, callback(this, &BMX160SPI3WireAPITester::_notify_flag),
                                SPI_EVENT_ERROR | SPI_EVENT_COMPLETE);
        uint32_t result = _async_operation_complete_flag.wait_any(0x3);
        return result == 0x01 ? 0 : -1;
    }

    int _write_sync(const uint8_t *data, int len, int bits)
    {
        bool single_word = bits / 8 == len;

        int err = 0;
        if (single_word) {
            int word;
            if (bits == 8) {
                word = *((const uint8_t *)data);
            } else {
                word = *((const uint16_t *)data);
            }
            _spi->write(word);
        } else {
            int res = _spi->write((const char *)data, len, nullptr, 0);
            if (res != len) {
                err = -1;
            }
        }
        return err;
    }

    int _read_sync(uint8_t *data, int len, int bits)
    {
        int err = 0;

        int res = _spi->write(nullptr, 0, (char *)data, len);
        if (res != len) {
            err = -1;
        }

        return err;
    }

    int _setup_register_write(BMX160Register reg, uint8_t data)
    {
        auto reg_addr_cmd = (uint8_t)reg;
        _spi_ssel->write(0);
        int err_a = _write_sync(&reg_addr_cmd, 1, 8);
        int err_d = _write_sync(&data, 1, 8);
        _spi_ssel->write(1);
        return err_a == 0 && err_d == 0 ? 0 : -1;
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
    BMX160SPI3WireAPITester(SPI *spi, DigitalOut *ssel)
            : _spi(spi), _spi_ssel(ssel)
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
     * Perform minimal BMX160 initialization for 3-wire testing.
     */
    int init()
    {
        return reset();
    }

    /**
     * Reset and initialize BMX160 for 3-wire testing.
     */
    int reset()
    {
        int op_err, err;
        err = 0;

        // configure SPI
        _spi->format(8, _SPI_MODE);

        // delay to be sure that bmx has started
        ThisThread::sleep_for(20ms);

        // create rising edge with SSEL to activate SPI
        _spi_ssel->write(0);
        ThisThread::sleep_for(1ms);
        _spi_ssel->write(1);
        ThisThread::sleep_for(1ms);

        // software reset
        if ((op_err = _setup_register_write(BMX160Register::CMD, 0xB6))) {
            err = op_err;
        }
        ThisThread::sleep_for(20ms);

        // create rising edge with SSEL to activate SPI
        _spi_ssel->write(0);
        ThisThread::sleep_for(1ms);
        _spi_ssel->write(1);
        ThisThread::sleep_for(1ms);

        // enable accelerometer to switch sensor to normal mode
        if ((op_err = _setup_register_write(BMX160Register::CMD, 0x11))) {
            err = op_err;
        }

        // activation delay
        ThisThread::sleep_for(1ms);

        // activate SPI 3-wire mode
        if ((op_err = _setup_register_write(BMX160Register::IF_CONF, 0x01))) {
            err = op_err;
        }

        return err;
    }

    /**
     * Send data to BMX160.
     *
     * Up to 6 bytes can be send. The bytes can be read later with ::read_data method
     *
     * @param data data to send
     * @param len data length (bytes)
     * @param bits SPI word size
     * @return 0 on success, otherwise non-zero value
     */
    int write_data(const uint8_t *data, int len, int bits = 8)
    {
        MBED_ASSERT(len <= 6);
        MBED_ASSERT(bits == 8 || bits == 16);
        int err_a, err_d;
        int err;

        uint8_t base_reg_cmd = (uint8_t)BMX160Register::OFFSET_ACCEL_X;
        int (BMX160SPI3WireAPITester::*write_data_impl)(const uint8_t *data, int len, int bits);
        if (_use_async) {
            write_data_impl = &BMX160SPI3WireAPITester::_write_async;
        } else {
            write_data_impl = &BMX160SPI3WireAPITester::_write_sync;
        }

        _spi_ssel->write(0);
        // send address using 8-bit mode
        err_a = (this->*write_data_impl)(&base_reg_cmd, 1, 8);
        // select mode for main data transmission
        if (bits != 8) {
            _spi->format(bits, _SPI_MODE);
        }
        err_d = (this->*write_data_impl)(data, len, bits);
        _spi_ssel->write(1);
        if (bits != 8) {
            // switch back to 8 bit mode
            _spi->format(8, _SPI_MODE);
        }
        err = err_a == 0 && err_d == 0 ? 0 : -1;

        // log transaction
        _log_transaction(err, base_reg_cmd, false, data, len);
        _transaction_count++;

        return err;
    }

    /**
    * Receive data from BMX160.
    *
    * Up to 6 bytes can be received. The bytes should be write in advance with ::write_data method.
    *
    * @param data data to receive
    * @param len data length (bytes)
    * @param bits SPI word size
    * @return 0 on success, otherwise non-zero value
    */
    int read_data(uint8_t *data, int len, int bits = 8)
    {
        MBED_ASSERT(len <= 6);
        MBED_ASSERT(bits == 8 || bits == 16);
        int err_a, err_d;
        int err;

        uint8_t base_reg_cmd = (uint8_t)BMX160Register::OFFSET_ACCEL_X | 0x80;
        int (BMX160SPI3WireAPITester::*write_data_impl)(const uint8_t *data, int len, int bits);
        int (BMX160SPI3WireAPITester::*read_data_impl)(uint8_t *data, int len, int bits);
        if (_use_async) {
            write_data_impl = &BMX160SPI3WireAPITester::_write_async;
            read_data_impl = &BMX160SPI3WireAPITester::_read_async;
        } else {
            write_data_impl = &BMX160SPI3WireAPITester::_write_sync;
            read_data_impl = &BMX160SPI3WireAPITester::_read_sync;
        }

        _spi_ssel->write(0);
        // send address using 8-bit mode
        err_a = (this->*write_data_impl)(&base_reg_cmd, 1, 8);
        // select mode for main data transmission
        if (bits != 8) {
            _spi->format(bits, _SPI_MODE);
        }
        err_d = (this->*read_data_impl)(data, len, bits);
        _spi_ssel->write(1);
        if (bits != 8) {
            // switch back to 8 bit mode
            _spi->format(8, _SPI_MODE);
        }
        err = err_a == 0 && err_d == 0 ? 0 : -1;

        // log transaction
        _log_transaction(err, base_reg_cmd, true, data, len);
        _transaction_count++;

        return err;
    }

    /**
     * Call SPI::abort_all_transfers method.
     */
    void call_spi_abort_all_transfers()
    {
        return _spi->abort_all_transfers();
    }
};


//----------------------------------------------------------------------------//
// Test code and helper functions.
//----------------------------------------------------------------------------//
template<typename T>
class SimpleList {
private:
    struct item_t {
        T value;
        item_t *next = nullptr;

        template<typename ...Args>
        explicit item_t(Args &&...args) : value(std::forward<Args>(args)...)
        {
        }
    };

    item_t *_items = nullptr;

    void _append(item_t *elem)
    {
        item_t **last_item = &_items;
        while (*last_item != nullptr) {
            last_item = &(*last_item)->next;
        }
        *last_item = elem;
    }

public:
    SimpleList()
    {
    }

    ~SimpleList()
    {
        item_t *item = _items;
        while (item != nullptr) {
            item_t *next_item = item->next;
            delete item;
            item = next_item;
        }
    }

    template<typename... Args>
    T *create_and_append(Args &&... args)
    {
        auto elem = new item_t(std::forward<Args>(args)...);
        _append(elem);
        return &(elem->value);
    }

    class SimpleListIterator {
    private:
        item_t *_item;
        friend SimpleList;

        SimpleListIterator(item_t *item) : _item(item)
        {}

    public:
        SimpleListIterator() : SimpleListIterator(nullptr)
        {}

        T *next()
        {
            item_t *current = _item;
            if (current) {
                _item = current->next;
            }
            return &(current->value);
        }
    };

    SimpleListIterator iterator()
    {
        return SimpleListIterator(_items);
    }
};

enum TestCaseType {
    SINGLE_8W_8R = 1 << 0,
    BURST_8W_8R = 1 << 1,
    SINGLE_16W_8R = 1 << 2,
    BURST_16W_8R = 1 << 3,
    SINGLE_8W_16R = 1 << 4,
    BURST_8W_16R = 1 << 5,
};
const TestCaseType TEST_CASE_TYPES[] = {
        SINGLE_8W_8R, BURST_8W_8R, SINGLE_16W_8R, BURST_16W_8R, SINGLE_8W_16R, BURST_8W_16R
};

const char *get_test_case_type_name(TestCaseType value)
{
    switch (value) {
        case SINGLE_8W_8R:
            return "single_w8_r8";
        case BURST_8W_8R:
            return "burst_w8_r8";
        case SINGLE_16W_8R:
            return "single_w16_r8";
        case BURST_16W_8R:
            return "burst_w16_r8";
        case SINGLE_8W_16R:
            return "single_w8_r16";
        case BURST_8W_16R:
            return "burst_w8_r16";
    }
    MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT), "Unknown case type");
}

enum TestCaseAPIUsage {
    API_USAGE_SYNC = 1 << 0,
    API_USAGE_ASYNC = 1 << 1,
    // call SPI::abort_all_transfers before each write/read cycle
    API_USAGE_CALL_ABORT_BEFORE = 1 << 2,
    // call SPI::abort_all_transfers after each write/read cycle
    API_USAGE_CALL_ABORT_AFTER = 1 << 3,
    // add small delay before each write/read cycle
    API_USAGE_ADD_DELAY_BEFORE = 1 << 4,
    //add small delay after each write/read cycle
    API_USAGE_ADD_DELAY_AFTER = 1 << 5,
};

const char *get_api_usage_name(int value)
{
    switch (value) {
        case API_USAGE_ASYNC:
            return "async";
        case API_USAGE_ASYNC | API_USAGE_CALL_ABORT_BEFORE:
            return "async (+ abort_all_transfers before)";
        case API_USAGE_ASYNC | API_USAGE_CALL_ABORT_AFTER:
            return "async (+ abort_all_transfers after)";
        case API_USAGE_ASYNC | API_USAGE_ADD_DELAY_BEFORE:
            return "async (+ small delay before)";
        case API_USAGE_ASYNC | API_USAGE_ADD_DELAY_AFTER:
            return "async (+ small delay after)";
        case API_USAGE_SYNC:
            return "sync";
        case API_USAGE_SYNC | API_USAGE_CALL_ABORT_BEFORE:
            return "sync (+ abort_all_transfers before)";
        case API_USAGE_SYNC | API_USAGE_CALL_ABORT_AFTER:
            return "sync (+ abort_all_transfers after)";
        case API_USAGE_SYNC | API_USAGE_ADD_DELAY_BEFORE:
            return "sync (+ small delay before)";
        case API_USAGE_SYNC | API_USAGE_ADD_DELAY_AFTER:
            return "sync (+ small delay after)";
        default:
            // don't process other cases for simplicity
            return "invalid combination";
    }
}


/**
 * Test case result.
 *
 * Note: in some cases we may transitive and receive, but dummy "reads" may be generated,
 * that is indicated with `data_ok` and `clock_ok` fields
 */
class TestCaseResult {
public:
    TestCaseResult(const TestCaseType case_type) : case_type(case_type)
    {
        reset();
    }

    /* case type */
    const TestCaseType case_type;

    /* total passes */
    int total;
    /* total successful passes */
    int successful;
    /* total successful passes with correct data */
    int data_ok;
    /* total successful passes with correct clock count */
    int clock_ok;


    bool is_ok() const
    {
        return total == successful;
    }

    bool is_data_ok() const
    {
        return total == data_ok;
    }

    bool is_clock_ok() const
    {
        return total == clock_ok;
    }

    void reset()
    {
        total = 0;
        successful = 0;
        data_ok = 0;
        clock_ok = 0;
    }
};


class ScenarioConfiguration {
public:
    const int spi_freq;
    const int api_usage;
    const int test_case_type;

    ScenarioConfiguration(int spi_freq, int api_usage, int test_case_type)
            : spi_freq(spi_freq), api_usage(api_usage), test_case_type(test_case_type)
    {
    }
};

struct ScenarioResult {
public:
    ScenarioResult(const ScenarioConfiguration *configuration)
            : configuration(configuration)
    {
        target_spi_frequency = configuration->spi_freq;
        for (TestCaseType test_case_type: TEST_CASE_TYPES) {
            if (configuration->test_case_type & test_case_type) {
                test_case_results.create_and_append((TestCaseType)test_case_type);
            }
        }
    }

    const ScenarioConfiguration *const configuration;

    /** If during BMX160 configuration any SPI object methods returns an error, it will be saved here */
    int init_error = 0;

    /** Target SPI frequency */
    int target_spi_frequency = 0;
    /** Actual SPI frequency */
    int actual_spi_frequency = 0;

    /** test results list */
    SimpleList<TestCaseResult> test_case_results;
};


/**
 * BMX160 sensor based SPI 3 wire tester.
 */
class BMX160SPI3WireTester : NonCopyable<BMX160SPI3WireTester> {
    static constexpr int DEFAULT_TEST_NUMBER = 64;
protected:
    PinName _spi_mosi_pin;
    PinName _spi_miso_pin;
    PinName _spi_sclk_pin;
    PinName _spi_ssel_pin;
    DigitalOut _spi_ssel;
    PinName _spi_sclk_counter_pin;

    int _test_number = DEFAULT_TEST_NUMBER;

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
              _spi_ssel(spi_ssel, 1),
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
     * @param test_number number of test passes for each scenario
     */
    void configure(int test_number = DEFAULT_TEST_NUMBER)
    {
        _test_number = test_number;
    }

    /**
     * Execute tests.
     */
    int test(ScenarioResult *test_result)
    {
        const ScenarioConfiguration *configuration = test_result->configuration;

        // construct SPI and pulse counter
        _spi_ssel = 1;
        SPIExt spi(_spi_mosi_pin, _spi_miso_pin, _spi_sclk_pin);
        spi.frequency(configuration->spi_freq);
        PulseCounter pc(_spi_sclk_counter_pin, PulseCounter::RisingEdge, PullNone);
        BMX160SPI3WireAPITester api(&spi, &_spi_ssel);

        int async_flag = configuration->api_usage & (API_USAGE_SYNC | API_USAGE_ASYNC);
        if (async_flag == API_USAGE_SYNC) {
            api.set_async(false);
        } else if (async_flag == API_USAGE_ASYNC) {
            api.set_async(true);
        } else {
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),
                       "API_USAGE_SYNC or API_USAGE_ASYNC flags must be set");
        }

        api.set_log(_log);

        // cleanup results
        test_result->init_error = 0;
        test_result->target_spi_frequency = configuration->spi_freq;
        test_result->actual_spi_frequency = spi.get_real_frequency();


        _log_msg("INFO", "================= start test =================");
        test_result->init_error = api.init();
        if (test_result->init_error) {
            _log_msg("ERROR", "API initialization error %i", test_result->init_error);
        }
        // run test cases
        auto test_case_iterator = test_result->test_case_results.iterator();
        TestCaseResult *test_case;
        while ((test_case = test_case_iterator.next()) != nullptr) {
            _test_passes_case_impl(test_case, configuration->api_usage, &api, &pc);
        }
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

    void _swap_16_bit_bytes(uint8_t *data, int len)
    {
        MBED_ASSERT(len % 2 == 0);
        for (int i = 0; i < len; i += 2) {
            uint8_t tmp = data[i];
            data[i] = data[i + 1];
            data[i + 1] = tmp;
        }
    }

    struct test_pass_result_t {
        bool ok;
        bool data_ok;
        bool clock_ok;
    };

    test_pass_result_t _test_pass_impl(BMX160SPI3WireAPITester *api, PulseCounter *pc,
                                       int api_usage_flags, int w_bits, int r_bits,
                                       const uint8_t data[6], int len)
    {
        MBED_ASSERT(w_bits == 8 || w_bits == 16);
        MBED_ASSERT(r_bits == 8 || r_bits == 16);
        MBED_ASSERT(len <= 6);
        uint8_t w_data[6];
        uint8_t r_data[6];
        char in_buf_msg[16];
        char out_buf_msg[16];

        int w_err;
        int r_err;
        int w_ticks;
        int r_ticks;
        test_pass_result_t result;

        if (api_usage_flags & API_USAGE_ADD_DELAY_BEFORE) {
            ThisThread::sleep_for(1ms);
        }
        if (api_usage_flags & API_USAGE_CALL_ABORT_BEFORE) {
            api->call_spi_abort_all_transfers();
        }

        // send bytes
        memcpy(w_data, data, len);
        pc->reset();
        w_err = api->write_data(w_data, len, w_bits);
        w_ticks = pc->get_count();

        // read bytes
        pc->reset();
        r_err = api->read_data(r_data, len, r_bits);
        r_ticks = pc->get_count();

        if (api_usage_flags & API_USAGE_CALL_ABORT_AFTER) {
            api->call_spi_abort_all_transfers();
        }
        if (api_usage_flags & API_USAGE_ADD_DELAY_AFTER) {
            ThisThread::sleep_for(1ms);
        }

        // check API errors
        result.ok = true;
        if (w_err) {
            _log_msg("ERROR", "Transmit error %i", w_err);
            result.ok = false;
        }
        if (r_err) {
            _log_msg("ERROR", "Receive error %i", r_err);
            result.ok = false;
        }
        // check ticks number
        result.clock_ok = true;
        int expected_ticks = 8 * (len + 1);
        if (w_ticks != expected_ticks) {
            _log_msg("ERROR", "Write error. Expect %i ticks, but got %i", expected_ticks, w_ticks);
            result.clock_ok = false;
        }
        if (r_ticks != expected_ticks) {
            _log_msg("ERROR", "Read error. Expect %i ticks, but got %i", expected_ticks, r_ticks);
            result.clock_ok = false;
        }
        if (!result.clock_ok) {
            result.ok = false;
        }
        // check data
        result.data_ok = true;
        if (w_bits == 16) {
            _swap_16_bit_bytes(w_data, len);
        }
        if (r_bits == 16) {
            _swap_16_bit_bytes(r_data, len);
        }
        if (strncmp((const char *)w_data, (const char *)r_data, len) != 0) {
            result.data_ok = false;
            result.ok = false;
            _log_msg("ERROR", "transmitted and received data differs: 0x%s != 0x%s",
                     _format_bytes(out_buf_msg, w_data, len),
                     _format_bytes(in_buf_msg, r_data, len)
            );
        }

        return result;
    }

    void _test_passes_impl(BMX160SPI3WireAPITester *api, PulseCounter *pc,
                           int api_usage_flags, int w_bits, int r_bits, int len, int test_number,
                           TestCaseResult *result)
    {
        test_pass_result_t step_result;
        uint8_t data[6] = {0};

        MBED_ASSERT(w_bits == 8 || w_bits == 16);
        MBED_ASSERT(r_bits == 8 || r_bits == 16);
        MBED_ASSERT(len <= 6);

        result->reset();
        result->total = test_number;

        for (int i = 0; i < test_number; i++) {
            step_result = _test_pass_impl(api, pc, api_usage_flags, w_bits, r_bits, data, len);
            if (step_result.ok) {
                result->successful += 1;
            }
            if (step_result.clock_ok) {
                result->clock_ok += 1;
            }
            if (step_result.data_ok) {
                result->data_ok += 1;
            }
            // update data
            for (int j = 0; j < len; j++) {
                data[j] += j + 1;
            }
        }
    }

    int _test_passes_case_impl(TestCaseResult *result, int api_usage_flags,
                               BMX160SPI3WireAPITester *api, PulseCounter *pc)
    {
        switch (result->case_type) {
            case SINGLE_8W_8R:
                _log_msg("INFO", "test single write (8 bit) / read (8 bit)");
                _test_passes_impl(api, pc, api_usage_flags, 8, 8, 1, _test_number, result);
                break;
            case BURST_8W_8R:
                _log_msg("INFO", "test burst write (8 bit) / read (8 bit)");
                _test_passes_impl(api, pc, api_usage_flags, 8, 8, 6, _test_number, result);
                break;
            case SINGLE_16W_8R:
                _log_msg("INFO", "test single write (16 bit) / read (8 bit)");
                _test_passes_impl(api, pc, api_usage_flags, 16, 8, 2, _test_number, result);
                break;
            case BURST_16W_8R:
                _log_msg("INFO", "test burst write (16 bit) / read (8 bit)");
                _test_passes_impl(api, pc, api_usage_flags, 16, 8, 6, _test_number, result);
                break;
            case SINGLE_8W_16R:
                _log_msg("INFO", "test burst write (8 bit) / read (16 bit)");
                _test_passes_impl(api, pc, api_usage_flags, 8, 16, 2, _test_number, result);
                break;
            case BURST_8W_16R:
                _log_msg("INFO", "test burst write (8 bit) / read (16 bit)");
                _test_passes_impl(api, pc, api_usage_flags, 8, 16, 6, _test_number, result);
                break;
            default:
                MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),
                           "Unknown case type");
        }
        return 0;
    }


};

static DigitalOut user_led(LED1, 1);

static BMX160SPI3WireTester bmx160SPI3WireTester(BMX160_SPI_MOSI, BMX160_SPI_MISO, BMX160_SPI_SCK, BMX160_SPI_CSB,
                                                 BMX160_SPI_CLK_COUNTER);


static const ScenarioConfiguration test_configurations[] = {
        {1'000'000, API_USAGE_SYNC, SINGLE_8W_8R},
        {1'000'000, API_USAGE_SYNC, BURST_8W_8R},
        {1'000'000, API_USAGE_SYNC, SINGLE_16W_8R},
        {1'000'000, API_USAGE_SYNC, BURST_16W_8R},
        {1'000'000, API_USAGE_SYNC, SINGLE_8W_16R},
        {1'000'000, API_USAGE_SYNC, BURST_8W_16R},

        {200'000, API_USAGE_SYNC, SINGLE_8W_8R},
        {200'000, API_USAGE_SYNC, BURST_8W_8R},
        {200'000, API_USAGE_SYNC, SINGLE_16W_8R},
        {200'000, API_USAGE_SYNC, BURST_16W_8R},
        {200'000, API_USAGE_SYNC, SINGLE_8W_16R},
        {200'000, API_USAGE_SYNC, BURST_8W_16R},

        {10'000'000, API_USAGE_SYNC, SINGLE_8W_8R},
        {10'000'000, API_USAGE_SYNC, BURST_8W_8R},
        {10'000'000, API_USAGE_SYNC, SINGLE_16W_8R},
        {10'000'000, API_USAGE_SYNC, BURST_16W_8R},
        {10'000'000, API_USAGE_SYNC, SINGLE_8W_16R},
        {10'000'000, API_USAGE_SYNC, BURST_8W_16R},

        {1'000'000, API_USAGE_ASYNC | API_USAGE_CALL_ABORT_AFTER, SINGLE_8W_8R},
        {1'000'000, API_USAGE_ASYNC, BURST_8W_8R},
        {1'000'000, API_USAGE_ASYNC, SINGLE_16W_8R},
        {1'000'000, API_USAGE_ASYNC, BURST_16W_8R},
        {1'000'000, API_USAGE_ASYNC, SINGLE_8W_16R},
        {1'000'000, API_USAGE_ASYNC, BURST_8W_16R},

        {200'000, API_USAGE_ASYNC | API_USAGE_CALL_ABORT_AFTER, SINGLE_8W_8R},
        {200'000, API_USAGE_ASYNC, BURST_8W_8R},
        {200'000, API_USAGE_ASYNC, SINGLE_16W_8R},
        {200'000, API_USAGE_ASYNC, BURST_16W_8R},
        {200'000, API_USAGE_ASYNC, SINGLE_8W_16R},
        {200'000, API_USAGE_ASYNC, BURST_8W_16R},

        {10'000'000, API_USAGE_ASYNC, SINGLE_8W_8R},
        {10'000'000, API_USAGE_ASYNC, BURST_8W_8R},
        {10'000'000, API_USAGE_ASYNC, SINGLE_16W_8R},
        {10'000'000, API_USAGE_ASYNC, BURST_16W_8R},
        {10'000'000, API_USAGE_ASYNC, SINGLE_8W_16R},
        {10'000'000, API_USAGE_ASYNC, BURST_8W_16R},
};
static bool TEST_VERBOSE = true;

int main()
{
    SimpleList<ScenarioResult> results;
    ScenarioResult *result;
    SimpleList<ScenarioResult>::SimpleListIterator result_iterator;
    TestCaseResult *test_case_result;
    SimpleList<TestCaseResult>::SimpleListIterator test_case_result_iterator;


    // prepare test_result structures
    for (const auto &test_configuration: test_configurations) {
        results.create_and_append(&test_configuration);
    }

    // run tests
    printf("================================ start ================================\n");
    bmx160SPI3WireTester.set_log(TEST_VERBOSE);
    result_iterator = results.iterator();
    while ((result = result_iterator.next()) != nullptr) {
        bmx160SPI3WireTester.test(result);
    }
    printf("================================ finish ================================\n");

    // show results
    printf("================================ result ================================\n");
    result_iterator = results.iterator();
    int scenario_i = 1;
    auto result_str = [](bool result) { return result ? "success" : "error"; };
    printf("|    |                                 api  |      case name | target freq (Hz) | actual freq (Hz) |    init |  result |    data |   clock |\n");
    printf("|----|--------------------------------------|----------------|------------------|------------------|---------|---------|---------|---------|\n");
    while ((result = result_iterator.next()) != nullptr) {
        test_case_result_iterator = result->test_case_results.iterator();
        while ((test_case_result = test_case_result_iterator.next()) != nullptr) {
            printf("| %2i | %36s | %14s | %16i | %16i | %7s | %7s | %7s | %7s |\n",
                   scenario_i,
                   get_api_usage_name(result->configuration->api_usage),
                   get_test_case_type_name(test_case_result->case_type),
                   result->target_spi_frequency,
                   result->actual_spi_frequency,
                   result_str(result->init_error == 0),
                   result_str(test_case_result->is_ok()),
                   result_str(test_case_result->is_data_ok()),
                   result_str(test_case_result->is_clock_ok())
            );
        }
        scenario_i++;
    }
    printf("========================================================================\n");

    while (true) {
        user_led = !user_led;
        ThisThread::sleep_for(1000ms);
    }

    return 0;
}
