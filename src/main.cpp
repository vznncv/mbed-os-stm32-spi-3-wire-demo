/**
 * SPI 3 Wire usage demo with BMX160 sensor.
 *
 * sensor: https://wiki.dfrobot.com/BMX160_9_Axis_Sensor_Module_SKU_SEN0373
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
// SPI CLK pin
#define BMX160_SPI_SCK PB_3
// SPI SSEL pin
#define BMX160_SPI_CSB PB_6
// SPI CLK counter pint
#define BMX160_SPI_CLK_COUNTER PA_0

// helper macro to check return code
static char app_error_msg_buf[256];
#define CHECK_RET_CODE(expr) do {                                                                         \
    int err = expr;                                                                                       \
    if (err != 0) {                                                                                       \
        snprintf(app_error_msg_buf, 256, "expression \"%s\" has returned error code %i", #expr, err);     \
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_UNKNOWN), app_error_msg_buf); \
    }                                                                                                     \
} while(0);

//----------------------------------------------------------------------------//
// Helper SPI interface that able to print actual frequency
//----------------------------------------------------------------------------//
extern "C" {
extern int spi_get_clock_freq(spi_t *obj);
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
        int psk_rank;
#if TARGET_STM32H7
        psk_rank = handle->Init.BaudRatePrescaler >> 28 & 0x07;
#else /* TARGET_STM32H7 */
        psk_rank = handle->Init.BaudRatePrescaler >> 3 & 0x07;
#endif /* TARGET_STM32H7 */
        return base_freq >> (psk_rank + 1);
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

class BMX160SPI3WireAPI : NonCopyable<BMX160SPI3WireAPI> {
public:
    static const int _SPI_MODE = 3;
    SPIExt _spi;
    int _spi_freq;
    DigitalOut _spi_ssel;
    bool _async = false;
    EventFlags _async_flag;

    int _transaction_count = 0;
    bool _log = false;

    static constexpr int _log_buf_size = 128;
    char _log_buf[_log_buf_size];

    void _log_msg(const char *msg, ...)
    {
        if (!_log) {
            return;
        }

        printf("bmx160: ");
        va_list args;
        va_start(args, msg);
        vprintf(msg, args);
        va_end(args);
        printf("\n");
    }

    char *_format_byte(char *str, int *buf_size, char prefix, uint8_t value)
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
            int buf_size = _log_buf_size;
            char *data_msg = _log_buf;
            char data_prefix = read_flag ? '<' : '>';
            data_msg = _format_byte(data_msg, &buf_size, '>', reg_addr_cmd);
            for (int i = 0; i < len; i++) {
                data_msg = _format_byte(data_msg, &buf_size, data_prefix, data[i]);
            }
            _log_msg("transaction %2i; err = %2i; data: %s", _transaction_count, err, _log_buf);
        }
    }

    void _notify_flag(int event)
    {
        if (event == SPI_EVENT_COMPLETE) {
            _async_flag.set(0x1);
        } else {
            _async_flag.set(0x3);
        }

    }

    int _write_async(const uint8_t *data, int len)
    {
        _async_flag.clear();
        _spi.transfer<uint8_t>(data, len, nullptr, 0, callback(this, &BMX160SPI3WireAPI::_notify_flag),
                               SPI_EVENT_ERROR | SPI_EVENT_COMPLETE);
        uint32_t result = _async_flag.wait_any(0x3);
        return result == 0x01 ? 0 : -1;
    }

    int _read_async(uint8_t *data, int len)
    {
        _async_flag.clear();
        _spi.transfer<uint8_t>(nullptr, 0, data, len, callback(this, &BMX160SPI3WireAPI::_notify_flag),
                               SPI_EVENT_ERROR | SPI_EVENT_COMPLETE);
        uint32_t result = _async_flag.wait_any(0x3);
        return result == 0x01 ? 0 : -1;
    }

public:
    BMX160SPI3WireAPI(PinName mosi, PinName sclk, PinName ssel, int freq = 10'000'00)
            : _spi(mosi, NC, sclk), _spi_freq(freq), _spi_ssel(ssel, 1)
    {
    }

    void set_log(bool value)
    {
        _log = value;
    }

    void set_async(bool value)
    {
        _async = value;
    }

    int init()
    {
        return reset();
    }

    int reset()
    {
        int err;

        // configure SPI
        _spi.format(8, _SPI_MODE);
        _spi.frequency(_spi_freq);
        _log_msg("target SPI frequency: %i Hz; actual SPI frequency: %i Hz", _spi_freq, _spi.get_real_frequency());
        _log_msg("SPI API type: %s", _async ? "asynchronous" : "synchronous");

        // delay to be sure that bmx has started
        ThisThread::sleep_for(20ms);

        // create rising edge with SSEL to activate SPI
        _spi_ssel = 0;
        ThisThread::sleep_for(1ms);
        _spi_ssel = 1;
        ThisThread::sleep_for(1ms);

        // software reset
        if ((err = register_write(BMX160Register::CMD, 0xB6))) {
            return err;
        }
        ThisThread::sleep_for(20ms);

        // create rising edge with SSEL to activate SPI
        _spi_ssel = 0;
        ThisThread::sleep_for(1ms);
        _spi_ssel = 1;
        ThisThread::sleep_for(1ms);

        // activate SPI 3-wire mode
        if ((err = register_write(BMX160Register::IF_CONF, 0x01))) {
            return err;
        }

        // enable accelerometer to switch sensor to normal mode
        if ((err = register_write(BMX160Register::CMD, 0x11))) {
            return err;
        }

        return 0;
    }

    int register_write(BMX160Register reg, const uint8_t *data, int len)
    {
        int res_a, res_d;
        int err;

        uint8_t reg_addr_cmd = (uint8_t)reg;
        _spi_ssel = 0;
        if (_async) {
            res_a = _write_async(&reg_addr_cmd, 1);
            res_d = _write_async(data, len);
            err = res_a == 0 && res_d == 0 ? 0 : -1;
        } else {
            _spi.write((uint8_t)reg_addr_cmd);
            res_d = _spi.write((const char *)data, len, nullptr, 0);
            err = res_d == len ? 0 : -1;
        }
        _spi_ssel = 1;


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
        int res_a, res_d, res;
        int err;

        uint8_t reg_addr_cmd = (uint8_t)reg | 0x80;
        _spi_ssel = 0;
        if (_async) {
            res_a = _write_async(&reg_addr_cmd, 1);
            res_d = _read_async(data, len);
            err = res_a == 0 && res_d == 0 ? 0 : -1;
        } else {
            res = _spi.write((char *)&reg_addr_cmd, 1, (char *)data, len);
            err = res == len + 1 ? 0 : -1;
        }
        _spi_ssel = 1;

        // log transaction
        _log_transaction(err, reg_addr_cmd, true, data, len);
        _transaction_count++;

        return err;
    }

    int register_read(BMX160Register reg, uint8_t *data)
    {
        return register_read(reg, data, 1);
    }
};


class SPIClockVerifier {
private:
    PulseCounter _pulse_counter;

public:
    SPIClockVerifier(PinName input_clk, PinMode pin_mode)
            : _pulse_counter(input_clk, PulseCounter::RisingEdge, pin_mode) {}

    void reset()
    {
        _pulse_counter.reset();
    }

    int verify_clock_ticks(int byte_num)
    {
        int actual_ticks = _pulse_counter.get_count();
        int expected_ticks = byte_num * 8;

        if (actual_ticks != expected_ticks) {
            printf("ERROR: expect to get %i clock ticks, but got %i ticks\n", expected_ticks, actual_ticks);
            return -1;
        }
        return 0;
    }
};


/**
 * Helper function to run SPI 3-wire test.
 *
 * @param freq
 * @param async
 * @return
 */
int test_spi(int freq, bool async)
{
    int error_count = 0;
    int data_ok_count = 0;
    int ok_count = 0;

    int err_clk;
    int err_test;
    int err_w;
    int err_r;

    printf("=========================== Test SPI ===========================\n");
    printf("Target freq: %i\n", freq);
    printf("API type: %s\n", async ? "asynchronous" : "synchronous");

    BMX160SPI3WireAPI register_api(BMX160_SPI_MOSI, BMX160_SPI_SCK, BMX160_SPI_CSB, freq);
    register_api.set_log(true);
    register_api.set_async(async);

    // clock verify pin
    SPIClockVerifier clock_verifier(BMX160_SPI_CLK_COUNTER, PullUp);

    printf("======== initialization ========\n");
    CHECK_RET_CODE(register_api.init());

    // test 1. Read CHIP_ID
    printf("============= test 1 ===========\n");
    const uint8_t BMX160_CHIP_ID = 0xD8;
    uint8_t chip_id = 0x00;

    clock_verifier.reset();
    register_api.register_read(BMX160Register::CHIP_ID, &chip_id);
    err_clk = clock_verifier.verify_clock_ticks(2);

    if (chip_id != BMX160_CHIP_ID) {
        printf("ERROR: invalid chip id: 0x%02X\n", chip_id);
        err_test = -1;
    } else {
        data_ok_count++;
        err_test = 0;
    }

    if (err_clk || err_test) {
        error_count++;
    } else {
        ok_count++;
    }

    // test 2. Burst read/write operation.
    // The test writes data to registers, then read it and check that written and read data are equal.
    // For these purposes BMX160 offset compensation are used, as by default they aren't used by sensor,
    // so we can use them to store temporary data.
    printf("============= test 2 ===========\n");
    BMX160Register reg_addr = BMX160Register::OFFSET_ACCEL_X;
    constexpr int data_len = 6;
    uint8_t out_data[data_len] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t in_data[data_len];
    for (int i = 0; i < 32; i++) {
        memset(in_data, 0, sizeof(in_data));
        clock_verifier.reset();
        err_w = register_api.register_write(reg_addr, out_data, data_len);
        err_r = register_api.register_read(reg_addr, in_data, data_len);
        err_clk = clock_verifier.verify_clock_ticks((data_len + 1) * 2);
        err_test = err_w || err_r ? -1 : 0;

        if (err_test) {
            printf("ERROR: read/write operation failed (err_w = %i, err_r = %i).\n", err_w, err_r);
        }
        if (memcmp(out_data, in_data, data_len) != 0) {
            err_test = -1;
            printf("ERROR: Invalid read/write operation.\n");
            printf("ERROR: out data: 0x");
            for (int j = 0; j < data_len; j++) {
                printf("%02X", out_data[j]);
            }
            printf("\nERROR: in data: 0x");
            for (int j = 0; j < data_len; j++) {
                printf("%02X", in_data[j]);
            }
            printf("\n");
        } else {
            data_ok_count++;
        }

        if (err_clk || err_test) {
            error_count++;
        } else {
            ok_count++;
        }

        // update test data
        for (int j = 0; j < data_len; j++) {
            out_data[j] += j;
        }
    }

    // show results
    printf("============= results ==========\n");
    printf("- total tests ok:        %i\n", ok_count);
    printf("- total tests ok (data): %i\n", data_ok_count);
    printf("- total tests error:     %i\n", error_count);
    printf("- status:                %s\n", error_count == 0 ? "OK" : "ERROR");

    printf("================================================================\n\n");

    return error_count == 0 ? 0 : -1;
}

int main()
{
    static DigitalOut user_led(LED1, 1);

    //
    // synchronous API SPI tests
    //

    test_spi(200'000, false);
    ThisThread::sleep_for(1s);

    test_spi(1'000'000, false);
    ThisThread::sleep_for(1s);

    test_spi(12'500'000, false);
    ThisThread::sleep_for(1s);

    //
    // asynchronous API SPI tests
    //

    test_spi(200'000, true);
    ThisThread::sleep_for(1s);

    test_spi(1'000'000, true);
    ThisThread::sleep_for(1s);

    test_spi(12'500'000, true);
    ThisThread::sleep_for(1s);


    printf("================== Blink demo ===================\n");
    while (true) {
        ThisThread::sleep_for(500ms);
        user_led = !user_led;
    }

    return 0;
}
