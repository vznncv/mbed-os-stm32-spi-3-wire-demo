/**
 * SPI utility tools and extensions.
 */
#ifndef APP_SPI_EXT_H
#define APP_SPI_EXT_H

#include "mbed.h"

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
    int get_real_frequency();
};

class SPISyncTransfer : private NonCopyable<SPISyncTransfer> {
private:
    SPI *_spi;
    EventFlags _transfer_flag;

    void _process_transfer_event(int event);

public:
    explicit SPISyncTransfer(SPI *spi) : _spi(spi)
    {}

    /**
     * Helper wrapper around SPI::transfer method that additionally waits end of transfer execution.
     *
     * It's needed to simplify test wring for SPI::transfer method.
     *
     * Note: this method isn't thread-safe.
     *
     * @param tx_buffer
     * @param tx_length
     * @param rx_buffer
     * @param rx_length
     * @return 0 on success, otherwise non-zero value
     */
    int sync_transfer(const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length);
};

/**
 * Get string representation (hexadecimal values) of byte array.
 *
 * @param buf
 * @param data
 * @param len
 * @return source buffer @c buf
 */
char *format_bytes_hex(char *buf, const uint8_t *data, size_t len);

#endif //APP_SPI_EXT_H
