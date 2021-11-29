add_executable(pio_sdio)

pico_generate_pio_header(pio_sdio ${CMAKE_CURRENT_LIST_DIR}/sdio.pio)

target_sources(pio_sdio PRIVATE
        sdio_test.c
        pio_sdio.c
        pio_sdio.h
        )

target_link_libraries(pio_sdio PRIVATE pico_stdlib hardware_pio hardware_dma)

# enable usb output, disable uart output
pico_enable_stdio_usb(pio_sdio 1)
pico_enable_stdio_uart(pio_sdio 0)

pico_add_extra_outputs(pio_sdio)

example_auto_set_url(pio_sdio)
