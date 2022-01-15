# stm32-spi-demo

Helper project that demonstrate/test 3/4-wire SPI protocol with STM32 board.

Currently, project contains the following demos:

- `src/main_base` - base SPI communication demo for checking SPI with logic analyzer.
- `src/main_3wire` - test to check SPI 3-wire mode with BMX160 sensor.
- `src/main_4wire` - test to check SPI 4-wire mode with loopback (`MISO` is connected to `MOSI`)

Active demo project is selected by `test_target` option in the `mbed_app.json`.

## 3-wire demo

### Hardware

The demo requires:

- STM32 Mbed compatible board.
- BMX160 sensor.

with the following connection:

![sensor connection](docs/scheme_3wire.png)

where:

- `BMX160_SPI_MOSI` - SPI master out, slave in pin;
- `BMX160_SPI_SCK` - SPI clock pin;
- `BMX160_SPI_CSB` - SPI chip select pin.
- `BMX160_SPI_CLK_COUNTER` - any STM32 TIM ETR input pin. It used to check SPI clock cycles and should be connected
  to `BMX160_SPI_SCK` pin.

### Project preparation

1. Adjust "platform.stdio-baud-rate" option in the `mbed_app.json` file if it's needed.
2. Set "test_target" option to "MAIN_3WIRE" in the `mbed_app.json`.
3. Run `./export_project --cli 2 --target <target_name>` to generate base `CMakeLists.txt` file.
4. Adjust pin name macros (`BMX160_SPI_*`) in the `src/main_3wire.cpp` files according your hardware configuration.
5. Open project with any IDE that supports *CMake*, compile it and upload to test target.

### Test results

The demo project performs minimal sensor configuration for SPI 3-wire mode usage and run tests that repeatedly send and
read data from "free" sensor registers (the send/received data should be the same if spi works correctly). The test
details can be found in `main.cpp` docstrings.

## 4-wire demo (16-bit mode)

### Hardware

The demo requires:

- STM32 Mbed compatible board.

with the following connection:

![sensor connection](docs/scheme_4wire.png)

where:

- `DEMO_SPI_MOSI` - SPI master out, slave in pin;
- `DEMO_SPI_MISO` - SPI master in, slave out pin;
- `DEMO_SPI_SCLK` - SPI clock pin;
- `DEMO_SPI_SSEL` - SPI chip selection.
- `DEMO_SPI_SCLK_COUNTER` - any STM32 TIM ETR input pin.

### Project preparation

1. Adjust "platform.stdio-baud-rate" option in the `mbed_app.json` file if it's needed.
2. Set "test_target" option to "MAIN_4WIRE" in the `mbed_app.json`.
3. Run `./export_project --cli 2 --target <target_name>` to generate base `CMakeLists.txt` file.
4. Adjust pin name macros (`DEMO_SPI_*`) in the `src/main_4wire.cpp` files according your hardware configuration.
5. Open project with any IDE that supports *CMake*, compile it and upload to test target.

### Test results

The demo project performs some data transmission tests from `MOSI` to `MISO` pins directly to check SPI in 16 bit mode.
The test results are printed to stdout as table.

## base demo

This demo has the same configuration like *4-wire demo (16-bit mode)* and is designed to check SPI output with any logic
analyzer device.
