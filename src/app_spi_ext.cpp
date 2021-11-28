#include "app_spi_ext.h"

#include "pwmout_api.h"

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

int SPIExt::get_real_frequency()
{
    spi_t *obj = &_peripheral->spi;
    SPI_HandleTypeDef *handle = &obj->spi.handle;
    int base_freq = spi_get_clock_freq(obj);
    return base_freq >> (spi_get_baudrate_prescaler_rank(handle->Init.BaudRatePrescaler) + 1);
}


int SPISyncTransfer::sync_transfer(const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length)
{
    int err;
    _transfer_flag.clear();
    err = _spi->transfer(tx_buffer, tx_length, rx_buffer, rx_length,
                         callback(this, &SPISyncTransfer::_process_transfer_event),
                         SPI_EVENT_ALL);
    if (err) {
        return err;
    }
    uint32_t result = _transfer_flag.wait_any(0x3);
    return result == 0x01 ? 0 : -1;
}

void SPISyncTransfer::_process_transfer_event(int event)
{
    if (event == SPI_EVENT_COMPLETE) {
        // success
        _transfer_flag.set(0x1);
    } else {
        // error
        _transfer_flag.set(0x2);
    }
}

char *format_bytes_hex(char *buf, const uint8_t *data, size_t len)
{
    char *buf_ptr = buf;
    for (size_t i = 0; i < len; i++) {
        sprintf(buf_ptr, "%02X", data[i]);
        buf_ptr += 2;
    }
    return buf;
}
