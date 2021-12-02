/**
 * SPI 4 Wire 16-bit mode demo.
 */
#if MBED_CONF_APP_TEST_TARGET == MAIN_4WIRE

#include <cstring>
#include <chrono>

#include "mbed.h"

#include "app_pulse_counter.h"
#include "app_spi_ext.h"
#include "app_utils.h"


//----------------------------------------------------------------------------//
// Hardware pins
//----------------------------------------------------------------------------//
// SPI MOSI pin
#define DEMO_SPI_MOSI PA_7
// SPI MISO pin
#define DEMO_SPI_MISO PA_6
// SPI SCLK pin
#define DEMO_SPI_SCLK PA_5
// SPI SSEL pin
#define DEMO_SPI_SSEL PA_4
// SPI CLK counter pin
#define DEMO_SPI_SCLK_COUNTER PA_12

//
// Connection notes:
//
// - DEMO_SPI_MOSI should be connected with DEMO_SPI_MISO
// - DEMO_SPI_SCLK should be connected with DEMO_SPI_SCLK_COUNTER
//

//----------------------------------------------------------------------------//
// Test code and helper functions.
//----------------------------------------------------------------------------//


/**
 * Test case result.
 *
 * Note: in some cases we may transitive and receive, but dummy "reads" may be generated,
 * that is indicated with `data_ok` and `clock_ok` fields
 */
class TestCaseResult {
public:
    TestCaseResult()
    {
        reset();
    }

    /* test case name */
    const char *name = "unknown";
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

    ScenarioConfiguration(int spi_freq)
            : spi_freq(spi_freq)
    {}
};

struct ScenarioResult {
public:
    ScenarioResult(const ScenarioConfiguration *configuration)
            : configuration(configuration), target_spi_frequency(configuration->spi_freq)
    {
    }

    const ScenarioConfiguration *const configuration;

    /** Target SPI frequency */
    int target_spi_frequency = 0;
    /** Actual SPI frequency */
    int actual_spi_frequency = 0;

    /** test case results list */
    SimpleList<TestCaseResult> test_case_results;
};


/**
 * SPI 4-wire mode 16 bit tester.
 */
class SPI4Wire16BitTester : NonCopyable<SPI4Wire16BitTester> {
    static constexpr int DEFAULT_TEST_NUMBER = 64;
protected:
    PinName _spi_mosi_pin;
    PinName _spi_miso_pin;
    PinName _spi_sclk_pin;
    PinName _spi_ssel_pin;
    DigitalOut _spi_ssel;
    PinName _spi_sclk_counter_pin;

    int _test_number = DEFAULT_TEST_NUMBER;
    SimpleLogger _logger;

    struct test_context_t {
        SPIExt *spi;
        SPISyncTransfer *spi_sync;
        DigitalOut *ssel;
        PulseCounter *pc;
        int test_number;

        void reset()
        {
            ssel->write(1);
            pc->reset();
            spi->set_default_write_value(SPI_FILL_CHAR);
        }
    };

public:
    /**
     * Constructor.
     *
     * @param spi_mosi SPI MOSI pin
     * @param spi_miso SPI MISO pin t should be connected to spi_mosi pin
     * @param spi_sclk SPI clock pin
     * @param spi_ssel SPI SSEL pin
     * @param spi_sclk_counter SPI clock counter pin. It should be connected to spi_sclk pin
     */
    SPI4Wire16BitTester(PinName spi_mosi, PinName spi_miso, PinName spi_sclk, PinName spi_ssel,
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
        _logger.set_enabled(value);
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
        SPIExt spi(_spi_mosi_pin, _spi_miso_pin, _spi_sclk_pin);
        spi.frequency(configuration->spi_freq);
        spi.format(16, 0);
        PulseCounter pc(_spi_sclk_counter_pin, PulseCounter::RisingEdge, PullNone);
        SPISyncTransfer spi_sync(&spi);

        test_context_t ctx;
        ctx.test_number = _test_number;
        ctx.spi = &spi;
        ctx.spi_sync = &spi_sync;
        ctx.ssel = &_spi_ssel;
        ctx.pc = &pc;
        ctx.reset();

        // cleanup results
        test_result->target_spi_frequency = configuration->spi_freq;
        test_result->actual_spi_frequency = spi.get_real_frequency();
        test_result->test_case_results.clear();
        _logger.info("================= start test =================");
        _test_single_word_transmission(test_result->test_case_results.create_and_append(), &ctx);
        ctx.reset();
        _test_multiple_words_transmission(test_result->test_case_results.create_and_append(), &ctx);
        ctx.reset();
        _test_multiple_words_transmission_async(test_result->test_case_results.create_and_append(), &ctx);
        ctx.reset();
        _test_multiple_words_transmission_with_default_fill(test_result->test_case_results.create_and_append(), &ctx);
        ctx.reset();
        _logger.info("================ complete test ================");
        return 0;
    }

private:
    int _assert_tick_number(int actual_ticks, int expected_ticks)
    {
        if (actual_ticks != expected_ticks) {
            _logger.error("Expect %i ticks, but get %i", expected_ticks, actual_ticks);
            return -1;
        } else {
            return 0;
        }
    }


    int _assert_data(const uint16_t *actual_data, const uint16_t *expected_data, size_t len)
    {
        const uint8_t *actual_data_bytes = (const uint8_t *)actual_data;
        const uint8_t *expected_data_bytes = (const uint8_t *)expected_data;
        len *= 2;

        const size_t max_len = 8;
        MBED_ASSERT(len <= max_len);

        if (memcmp(actual_data_bytes, expected_data_bytes, len) != 0) {
            char actual_data_buf[max_len * 2 + 1];
            char expected_data_buf[max_len * 2 + 1];
            _logger.error("Expect data 0x%s, but get 0x%s",
                          format_bytes_hex(actual_data_buf, actual_data_bytes, len),
                          format_bytes_hex(expected_data_buf, expected_data_bytes, len)
            );
            return -1;
        } else {
            return 0;
        }
    }

    void _check_results(TestCaseResult *test_case_result,
                        const uint16_t *actual_data, const uint16_t *expected_data, size_t data_len,
                        int actual_ticks, int expected_ticks, int api_err = 0)
    {

        int err = 0;

        test_case_result->total++;
        if (_assert_data(actual_data, expected_data, data_len) == 0) {
            test_case_result->data_ok++;
        } else {
            err = -1;
        }
        if (_assert_tick_number(actual_ticks, expected_ticks) == 0) {
            test_case_result->clock_ok++;
        } else {
            err = -1;
        }
        if (api_err) {
            _logger.error("SPI API error: %i", api_err);
            err = -1;
        }
        if (err == 0) {
            test_case_result->successful++;
        }
    }

    void _test_single_word_transmission(TestCaseResult *test_case_result, test_context_t *ctx)
    {
        test_case_result->name = "single word write/read";
        _logger.info("run %s test", test_case_result->name);
        test_case_result->reset();

        uint16_t out_value;
        uint16_t in_value;
        int actual_ticks;
        const int expected_ticks = 16;

        out_value = 0;
        for (int i = 0; i < ctx->test_number; i++) {
            ctx->pc->reset();
            ctx->ssel->write(0);
            in_value = ctx->spi->write(out_value);
            ctx->ssel->write(1);
            actual_ticks = ctx->pc->get_count();
            // check result
            _check_results(test_case_result, &in_value, &out_value, 1, actual_ticks, expected_ticks);
            // update test value
            out_value += 42;
        }
    }

    void _test_multiple_words_transmission(TestCaseResult *test_case_result, test_context_t *ctx)
    {
        test_case_result->name = "multiple word write/read";
        _logger.info("run %s test", test_case_result->name);
        test_case_result->reset();

        const size_t data_len = 4;
        const size_t raw_data_len = data_len * 2;
        uint16_t out_value[data_len] = {0};
        uint16_t in_value[data_len] = {0};
        int actual_ticks;
        const int expected_ticks = 16 * data_len;
        int result;
        int api_err;

        for (int i = 0; i < ctx->test_number; i++) {
            ctx->pc->reset();
            ctx->ssel->write(0);
            result = ctx->spi->write((const char *)out_value, raw_data_len, (char *)in_value, raw_data_len);
            ctx->ssel->write(1);
            actual_ticks = ctx->pc->get_count();
            // check result
            api_err = result == raw_data_len ? 0 : -1;
            _check_results(test_case_result, in_value, out_value, data_len, actual_ticks, expected_ticks, api_err);
            // update test value
            for (size_t j = 0; j < data_len; j++) {
                out_value[j] += 12 + j * 10;
            }
        }
    }

    void _test_multiple_words_transmission_async(TestCaseResult *test_case_result, test_context_t *ctx)
    {
        test_case_result->name = "multiple word write/read async";
        _logger.info("run %s test", test_case_result->name);
        test_case_result->reset();

        const size_t data_len = 4;
        const size_t raw_data_len = data_len * 2;
        uint16_t out_value[data_len] = {0};
        uint16_t in_value[data_len] = {0};
        int actual_ticks;
        const int expected_ticks = 16 * data_len;
        int api_err;

        for (int i = 0; i < ctx->test_number; i++) {
            ctx->pc->reset();
            ctx->ssel->write(0);
            api_err = ctx->spi_sync->sync_transfer((const char *)out_value, raw_data_len, (char *)in_value,
                                                   raw_data_len);
            ctx->ssel->write(1);
            actual_ticks = ctx->pc->get_count();
            // check result
            _check_results(test_case_result, in_value, out_value, data_len, actual_ticks, expected_ticks, api_err);
            // update test value
            for (size_t j = 0; j < data_len; j++) {
                out_value[j] += 12 + j * 10;
            }
        }
    }


    void _test_multiple_words_transmission_with_default_fill(TestCaseResult *test_case_result, test_context_t *ctx)
    {
        test_case_result->name = "multiple word write/read with default fill";
        _logger.info("run %s test", test_case_result->name);
        test_case_result->reset();

        const size_t out_data_len = 2;
        const size_t raw_out_data_len = out_data_len * 2;
        const size_t in_data_len = 4;
        const size_t raw_in_data_len = in_data_len * 2;
        const size_t total_data_len = out_data_len > in_data_len ? out_data_len : in_data_len;
        const size_t raw_total_data_len = total_data_len * 2;

        uint16_t out_value[out_data_len] = {0};
        uint16_t in_value[in_data_len] = {0};
        uint16_t in_value_expected[in_data_len] = {0};
        uint8_t fill_sym = 0xFF;

        int actual_ticks;
        const int expected_ticks = 16 * total_data_len;
        int result;
        int api_err;

        for (int i = 0; i < ctx->test_number; i++) {
            ctx->pc->reset();
            ctx->spi->set_default_write_value(fill_sym);
            ctx->ssel->write(0);
            result = ctx->spi->write((const char *)out_value, raw_out_data_len, (char *)in_value, raw_in_data_len);
            ctx->ssel->write(1);
            actual_ticks = ctx->pc->get_count();
            // check result
            memcpy(in_value_expected, out_value, raw_out_data_len);
            for (size_t j = out_data_len; j < in_data_len; j++) {
                in_value_expected[j] = fill_sym | (fill_sym << 8);
            }
            api_err = result == raw_total_data_len ? 0 : -1;
            _check_results(test_case_result, in_value, in_value_expected, in_data_len,
                           actual_ticks, expected_ticks, api_err);
            // update test values
            for (size_t j = 0; j < out_data_len; j++) {
                out_value[j] += 12 + j * 10;
            }
            fill_sym += 0x07;
        }
    }
};

static DigitalOut user_led(LED1, 1);

static const ScenarioConfiguration test_configurations[] = {
        {1'000'000},
        {200'000},
        {10'000'000},
};
static bool TEST_VERBOSE = true;

int main()
{
    SimpleList<ScenarioResult> results;
    SPI4Wire16BitTester tester(DEMO_SPI_MOSI, DEMO_SPI_MISO, DEMO_SPI_SCLK, DEMO_SPI_SSEL, DEMO_SPI_SCLK_COUNTER);

    // prepare test_result structures
    for (const auto &test_configuration: test_configurations) {
        results.create_and_append(&test_configuration);
    }

    // run tests
    printf("================================ start ================================\n");
    tester.set_log(TEST_VERBOSE);
    for (ScenarioResult &result: results) {
        tester.test(&result);
    }
    printf("================================ finish ================================\n");

    // show results
    printf("================================ result ================================\n");
    int test_case_i = 1;
    auto result_str = [](bool result) { return result ? "success" : "error"; };
    printf("|    |                                  case name | target freq (Hz) | actual freq (Hz) |  result |    data |   clock |\n");
    printf("|----|--------------------------------------------|------------------|------------------|---------|---------|---------|\n");
    for (ScenarioResult &result: results) {
        for (TestCaseResult &test_case_result: result.test_case_results) {
            printf("| %2i | %42s | %16i | %16i | %7s | %7s | %7s |\n",
                   test_case_i,
                   test_case_result.name,
                   result.target_spi_frequency,
                   result.actual_spi_frequency,
                   result_str(test_case_result.is_ok()),
                   result_str(test_case_result.is_data_ok()),
                   result_str(test_case_result.is_clock_ok())
            );
            test_case_i++;
        }

    }
    printf("========================================================================\n");

    while (true) {
        user_led = !user_led;
        ThisThread::sleep_for(1000ms);
    }

    return 0;
}

#endif
