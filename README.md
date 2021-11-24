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
- `BMX160_SPI_CLK_COUNTER` - any STM32 TIM ETR input pint. It used to check SPI clock cycles and should be connected
  to `BMX160_SPI_SCK` pin.

## Project preparation

1. Adjust "platform.stdio-baud-rate" option in the `mbed_app.json` file if it's needed.

2. Run `./export_project --cli 2 --target <target_name>` to generate base `CMakeLists.txt` file.

3. Adjust pin name macros (`BMX160_SPI_*`) in the `src/main.cpp` files according your hardware configuration.

3. Open project with any IDE that supports *CMake*, compile it and upload to test target.

## Test results

The demo project performs minimal sensor configuration for SPI 3-wire mode usage and run tests that repeatedly send and
read data from "free" sensor registers (the send/received data should be the same if spi works correctly). The test
details can be found in `main.cpp` docstrings.
