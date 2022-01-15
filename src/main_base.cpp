/**
 * Simple SPI 16-bit mode demo to check
 */
#if MBED_CONF_APP_TEST_TARGET == MAIN_BASE

#include <cstring>
#include <chrono>

#include "mbed.h"

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


//----------------------------------------------------------------------------//
// demo
//----------------------------------------------------------------------------//

static DigitalOut user_led(LED1, 1);
static SimpleLogger logger;
static int TARGET_FREQUENCY = 1'000'000;
static DigitalOut ssel(DEMO_SPI_SSEL, 1);

static int spi_write_wrapper(SPI *spi, const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length, bool async)
{
    if (!async) {
        return spi->write(tx_buffer, tx_length, rx_buffer, rx_length);
    } else {
        EventFlags event_flag;
        event_callback_t callback = [&event_flag](int event) {
            event_flag.set(event);
        };
        spi->transfer(tx_buffer, tx_length, rx_buffer, rx_length, callback, SPI_EVENT_ALL);
        int event_result = event_flag.wait_any(SPI_EVENT_ALL);
        return event_result == SPI_EVENT_COMPLETE ? tx_length : -1;
    }
}

void spi_demo(int freq, int wire_mode, bool async = false)
{
    // create spi object
    SPIExt *spi;
    if (wire_mode == 4) {
        spi = new SPIExt(DEMO_SPI_MOSI, DEMO_SPI_MISO, DEMO_SPI_SCLK);
    } else if (wire_mode == 3) {
        spi = new SPIExt(DEMO_SPI_MOSI, NC, DEMO_SPI_SCLK);
    } else {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT), "Unknown SPI wire mode");
    }
    spi->frequency(freq);
    logger.info("SPI demo. Mode: %i wire. API: %s", wire_mode, async ? "async" : "sync");

    // 8 bit demo
    int tx_data_8_len = 2;
    const uint8_t tx_data_8[2] = {0x12, 0x34};
    int rx_data_8_len = 2;
    uint8_t rx_data_8[2];
    int result_8;
    if (wire_mode == 3) {
        // ignore data receiving in 3 wire mode for simplicity
        rx_data_8_len = 0;
    }
    spi->format(8);
    ssel = 0;
    result_8 = spi_write_wrapper(
            spi,
            (const char *)tx_data_8, tx_data_8_len,
            (char *)rx_data_8, rx_data_8_len,
            async
    );
    ssel = 1;

    ThisThread::sleep_for(1ms);

    // 16 bit demo
    int tx_data_16_len = 4;
    const uint16_t tx_data_16[2] = {0x1122, 0x3344};
    int rx_data_16_len = 4;
    uint16_t rx_data_16[2];
    int result_16;
    if (wire_mode == 3) {
        // ignore data receiving in 3 wire mode for simplicity
        rx_data_16_len = 0;
    }
    spi->format(16);
    ssel = 0;
    result_16 = spi_write_wrapper(
            spi,
            (const char *)tx_data_16, tx_data_16_len,
            (char *)rx_data_16, rx_data_16_len,
            async
    );
    ssel = 1;

    // log result
    char tx_str_buf[24];
    char rx_str_buf[24];
    if (wire_mode == 3) {
        logger.info(" 8 bit. result = %i, tx_data = 0x%s", result_8,
                    format_bytes_hex(tx_str_buf, tx_data_8, 2)
        );
        logger.info("16 bit. result = %i, tx_data = 0x%s", result_16,
                    format_bytes_hex(tx_str_buf, (const uint8_t *)tx_data_16, 4)
        );
    } else {
        logger.info(" 8 bit. result = %i, tx_data = 0x%s; rx_data = 0x%s", result_8,
                    format_bytes_hex(tx_str_buf, tx_data_8, 2),
                    format_bytes_hex(rx_str_buf, rx_data_8, 2)
        );
        logger.info("16 bit. result = %i, tx_data = 0x%s; rx_data = 0x%s", result_16,
                    format_bytes_hex(tx_str_buf, (const uint8_t *)tx_data_16, 4),
                    format_bytes_hex(rx_str_buf, (const uint8_t *)rx_data_16, 4)
        );
    }

    // cleanup spi object
    delete spi;
}


int main()
{
    logger.info("================================ start =================================");
    spi_demo(TARGET_FREQUENCY, 3, false);
    spi_demo(TARGET_FREQUENCY, 3, true);
    spi_demo(TARGET_FREQUENCY, 4, false);
    spi_demo(TARGET_FREQUENCY, 4, true);
    logger.info("================================ finish ================================");

    while (true) {
        user_led = !user_led;
        ThisThread::sleep_for(1000ms);
    }

    return 0;
}

#endif
