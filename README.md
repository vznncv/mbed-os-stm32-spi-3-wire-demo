# stm32-spi-3-wire-demo

Helper project that demonstrate/test 3-wire SPI protocol with STM board.

## Hardware

The demo project requires:

- STM32 Mbed compatible board.
- BMX160 sensor.

with the following connection:

![sensor connection](docs/scheme.png)

where:

- `BMX160_SPI_MOSI` - SPI master out, slave in pin;
- `BMX160_SPI_SCK` - SPI clock pin;
- `BMX160_SPI_CSB` - SPI chip select pin.
- `BMX160_SPI_CLK_COUNTER` - any PWM non-inverted pin of channel 1/2 of general purpose timer. It uses STM32 timer
  encoder feature to count SPI clock ticks.

## Project preparation

1. Adjust "platform.stdio-baud-rate" option in the `mbed_app.json` file if it's needed.

2. Run `./export_project --cli 2 --target <target_name>` to generate base `CMakeLists.txt` file.

3. Adjust pin name macros (`BMX160_SPI_*`) in the `src/main.cpp` files according your hardware configuration.

3. Open project with any IDE that supports *CMake*, compile it and upload to test target.

## Test results

The demo project performs minimal sensor configuration for SPI 3-wire mode usage and run 2 simple test with different
frequencies and synchronous/asynchronous SPI API usage:

1. Test 1 - read chip ID (sensor should return `D8`)
2. Test 2 - write 6 bytes to device registers and then read them to test burst write/read. The "offset" bmx160 registers
   are used as sensor don't use then by default. This test is repeated many times to check SPI stability.

The test code log each transaction with the following format:

```
bmx160: transaction  9; err =  0; data: >F1<00<02<04<06<08<0A
```

where `err` shows any error that are returned by SPI API, `data` show transmitted bytes in hexadecimal format and
direction prefix (`>` - means that byte is send from MCU to sensor, `<` means that byte is send from sensor to MCU).

Additionally, code uses `BMX160_SPI_CLK_COUNTER` pin to count real number of SPI clock ticks and detect dummy reads.

If any error occurs, corresponding message is printed.
