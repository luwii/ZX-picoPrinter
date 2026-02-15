#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "zx_printer_emulator.pio.h"
#include "st7789.h"

// --- EM5820 Printer Definitions ---
#define PRINTER_UART uart0
#define PRINTER_BAUD 9600
#define PRINTER_TX_PIN 16
#define PRINTER_RX_PIN 17

// --- ZX Printer Emulator Definitions ---
#define DEBUG_VERBOSE 1
#define DISPLAY_MODE 0  // 0 = first 80 pixels, 1 = last 80 pixels for debug dump
#define ZX_DATA_BASE 0
#define ZX_CS_PIN 8
#define ZX_PRD_PIN 9
#define DEBUG_PIN 18
#define ZX_PWR_PIN 21

static int PRINTER_SCALE_H_PERCENT = 100;
static int PRINTER_SCALE_V_PERCENT = 100;

// --- ST7789 Display Pins ---
#define DISP_SCK 10
#define DISP_MOSI 11
#define DISP_RST 13
#define DISP_DC 12
#define DISP_CS 14
#define DISP_BL 15

// ESC/POS Constants
const uint8_t CMD_INIT[] = {0x1B, 0x40};

// Buffer size
#define BUFFER_SIZE 65536
static uint8_t buffer[BUFFER_SIZE];
static uint8_t cleaned[BUFFER_SIZE];  // Cleaned pixel-only buffer

// --- Clean buffer: parse and extract only pixel bytes, skipping controls ---
uint32_t clean_pixel_buffer(uint8_t* raw, uint32_t raw_size) {
    uint32_t cleaned_idx = 0;
    uint32_t skipped = 0;
    uint32_t i = 0;

    while (i < raw_size) {
        uint8_t b = raw[i];
        if (b == 0x04) {  // Skip stop (motor off)
            skipped++;
            i++;
            continue;
        }
        if (b == 0x00 || b == 0x02) {  // Speed byte: skip it, then copy next 256 pixels
            skipped++;
            i++;
            for (uint32_t p = 0; p < 256 && i < raw_size; p++) {
                // Copy pixel byte (may be 0x00/0x02/0x80/0x82)
                cleaned[cleaned_idx++] = raw[i];
                i++;
            }
            continue;
        }
        // Unexpected byte: skip (e.g., BREAK or error cases)
        skipped++;
        i++;
    }

    // Warn if incomplete last line
    if (cleaned_idx % 256 != 0 && DEBUG_VERBOSE && stdio_usb_connected()) {
        printf("Warning: Incomplete last line (%lu extra bytes)\n", (unsigned long)(cleaned_idx % 256));
    }

    if (DEBUG_VERBOSE && stdio_usb_connected()) {
        printf("Cleaned buffer: %lu bytes (skipped %lu control/unexpected bytes)\n", 
               (unsigned long)cleaned_idx, (unsigned long)skipped);
    }

    return cleaned_idx;
}

// --- Thermal print function (updated to use clean pixel stream) ---
void print_buffer_to_thermal(uint8_t *buf, uint32_t num_lines, bool unused_is_copy) {
    (void)unused_is_copy;  // No longer needed

    // Set line spacing to 0 for continuous image
    const uint8_t cmd_no_spacing[] = {0x1B, 0x33, 0x00};
    uart_write_blocking(PRINTER_UART, cmd_no_spacing, 3);

    const int src_w = 256;
    const int src_h = (int)num_lines;

    int dst_w = (src_w * PRINTER_SCALE_H_PERCENT + 50) / 100; if (dst_w < 1) dst_w = 1;
    int dst_h = (src_h * PRINTER_SCALE_V_PERCENT + 50) / 100; if (dst_h < 1) dst_h = 1;

    uint8_t *dst = calloc(dst_w * dst_h, 1);
    if (!dst) {
        dst_w = src_w; dst_h = src_h;
    }

    // Area-averaging downscale
    for (int dy = 0; dy < dst_h; dy++) {
        int sy0 = (dy * src_h) / dst_h;
        int sy1 = ((dy + 1) * src_h) / dst_h - 1;
        if (sy1 < sy0) sy1 = sy0;
        for (int dx = 0; dx < dst_w; dx++) {
            int sx0 = (dx * src_w) / dst_w;
            int sx1 = ((dx + 1) * src_w) / dst_w - 1;
            if (sx1 < sx0) sx1 = sx0;
            int total = (sx1 - sx0 + 1) * (sy1 - sy0 + 1);
            int ones = 0;
            for (int sy = sy0; sy <= sy1; sy++) {
                uint32_t pixel_start = sy * 256;  // Now always 256-byte lines
                for (int sx = sx0; sx <= sx1; sx++) {
                    uint32_t idx = pixel_start + sx;
                    if (idx < BUFFER_SIZE && (buf[idx] & 0x80)) ones++;
                }
            }
            if (dst) dst[dy * dst_w + dx] = (ones * 2 >= total) ? 1 : 0;
        }
    }

    // Send as ESC/POS graphics (8-dot mode)
    for (int row_base = 0; row_base < dst_h; row_base += 8) {
        int n = dst_w;
        uint8_t header[5] = {0x1B, 0x2A, 0x00, (uint8_t)(n & 0xFF), (uint8_t)((n >> 8) & 0xFF)};
        uart_write_blocking(PRINTER_UART, header, sizeof(header));

        for (int col = 0; col < dst_w; col++) {
            uint8_t out = 0;
            for (int bit = 0; bit < 8; bit++) {
                int dy = row_base + bit;
                if (dy >= dst_h) break;
                uint8_t pix = dst ? dst[dy * dst_w + col] : 0;
                if (pix) out |= (1u << (7 - bit));
            }
            uart_write_blocking(PRINTER_UART, &out, 1);
        }
        uint8_t lf = 0x0A;
        uart_write_blocking(PRINTER_UART, &lf, 1);
        sleep_ms(2);
    }

    if (dst) free(dst);
    const uint8_t cmd_reset_spacing[] = {0x1B, 0x32};
    uart_write_blocking(PRINTER_UART, cmd_reset_spacing, 2);
}

// Debug helper: print cleaned captured pixel data to USB stdio as ASCII art.
// Runs only when `DEBUG_VERBOSE` is enabled; respects `DISPLAY_MODE` and
// a configurable pixel count for the dump.
static void debug_print_cleaned_dump(uint8_t *cleaned_buf, uint32_t cleaned_size, uint32_t num_lines) {
#if DEBUG_VERBOSE
    const uint32_t DEBUG_DISPLAY_PIXELS = 80; // change this to show different widths
    uint32_t display_pixels = DEBUG_DISPLAY_PIXELS;
    uint32_t pixel_offset = (DISPLAY_MODE == 0) ? 0 : (256 - display_pixels);

    printf("\n=== DEBUG: Cleaned data dump ===\n");
    printf("Lines: %lu, Display pixels: %lu, Offset: %lu\n",
           (unsigned long)num_lines, (unsigned long)display_pixels, (unsigned long)pixel_offset);

    for (uint32_t line = 0; line < num_lines; line++) {
        printf("Line %3lu: ", (unsigned long)line);
        uint32_t pixel_start = line * 256;
        for (uint32_t pixel = 0; pixel < display_pixels; pixel++) {
            uint32_t idx = pixel_start + pixel_offset + pixel;
            if (idx < cleaned_size) {
                uint8_t data = cleaned_buf[idx];
                printf("%c", (data & 0x80) ? '#' : ' ');
            } else {
                printf(" ");
            }
        }
        printf("\n");
    }
    printf("=== End DEBUG dump ===\n\n");
#endif
}

int main() {
    stdio_init_all();

    // 1. Init thermal printer UART
    uart_init(PRINTER_UART, PRINTER_BAUD);
    uart_set_format(PRINTER_UART, 8, 1, UART_PARITY_NONE);
    gpio_set_function(PRINTER_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(PRINTER_RX_PIN, GPIO_FUNC_UART);
    sleep_ms(1000);
    uart_write_blocking(PRINTER_UART, CMD_INIT, sizeof(CMD_INIT));

    // Debug blinker if verbose
#if DEBUG_VERBOSE
    gpio_init(DEBUG_PIN);
    gpio_set_dir(DEBUG_PIN, GPIO_OUT);
    for (int i = 0; i < 5; i++) {
        gpio_put(DEBUG_PIN, 1);
        uart_puts(PRINTER_UART, "DEBUG_VERBOSE is on\n");
        sleep_ms(100);
        gpio_put(DEBUG_PIN, 0);
        sleep_ms(100);
    }
#endif

    // 2. Init ST7789 display
    st7789_config_t display_cfg = {
        .spi = spi1, .pin_sck = DISP_SCK, .pin_mosi = DISP_MOSI,
        .pin_rst = DISP_RST, .pin_dc = DISP_DC, .pin_cs = DISP_CS, .pin_bl = DISP_BL
    };
    st7789_init(&display_cfg);
    st7789_fill_screen(ST7789_BLACK);
    st7789_draw_string(40, 100, "ZX PICO PRINTER", ST7789_WHITE, ST7789_BLACK, 2);
    st7789_fill_rect(220, 0, 100, 240, ST7789_BLACK);

    if (stdio_usb_connected()) {
        printf("\n====================================\n");
        printf("ZX Printer Emulator & EM5820\n");
        printf("====================================\n");
    }

    // 3. ZX bus GPIO setup
    gpio_init(ZX_CS_PIN); gpio_init(ZX_PRD_PIN); gpio_init(ZX_PWR_PIN);
    gpio_set_dir(ZX_CS_PIN, GPIO_IN);
    gpio_set_dir(ZX_PRD_PIN, GPIO_IN);
    gpio_set_dir(ZX_PWR_PIN, GPIO_IN);
    gpio_pull_up(ZX_CS_PIN);
    gpio_pull_up(ZX_PRD_PIN);
    gpio_pull_up(ZX_PWR_PIN);

    // 4. PIO setup
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &zx_printer_emulator_program);
    uint sm = pio_claim_unused_sm(pio, true);
    pio_sm_config c = zx_printer_emulator_program_get_default_config(offset);
    sm_config_set_in_pins(&c, ZX_DATA_BASE);
    sm_config_set_out_pins(&c, ZX_DATA_BASE, 8);
    sm_config_set_set_pins(&c, ZX_DATA_BASE, 8);
    sm_config_set_sideset_pins(&c, DEBUG_PIN);
    sm_config_set_in_shift(&c, false, true, 8);
    sm_config_set_jmp_pin(&c, ZX_PRD_PIN);
    pio_sm_init(pio, sm, offset, &c);
    for (int i = 0; i < 8; i++) pio_gpio_init(pio, ZX_DATA_BASE + i);
    pio_gpio_init(pio, DEBUG_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm, DEBUG_PIN, 1, true);
    pio_sm_set_pins_with_mask(pio, sm, 0, (1u << DEBUG_PIN));
    pio_sm_put_blocking(pio, sm, 0x81);

    // 5. DMA setup
    int dma_chan = dma_claim_unused_channel(true);
    dma_channel_config dma_config = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_8);
    channel_config_set_read_increment(&dma_config, false);
    channel_config_set_write_increment(&dma_config, true);
    channel_config_set_dreq(&dma_config, pio_get_dreq(pio, sm, false));
    dma_channel_configure(dma_chan, &dma_config, buffer, &pio->rxf[sm], BUFFER_SIZE, false);

    pio_sm_set_enabled(pio, sm, true);
    dma_channel_start(dma_chan);

    uint32_t buffer_index = 0, idle_count = 0;
    const uint32_t IDLE_THRESHOLD = 500000;

    while (true) {
        uint32_t bytes_transferred = BUFFER_SIZE - dma_channel_hw_addr(dma_chan)->transfer_count;
        if (bytes_transferred > buffer_index) {
            buffer_index = bytes_transferred;
            idle_count = 0;
        } else {
            idle_count++;
            if (idle_count >= IDLE_THRESHOLD && buffer_index > 0) {
                // Clean the buffer first
                uint32_t cleaned_size = clean_pixel_buffer(buffer, buffer_index);
                uint32_t num_lines = cleaned_size / 256;

                if (DEBUG_VERBOSE && stdio_usb_connected()) {
                    printf("Captured: %lu bytes → %lu clean pixel bytes → %lu lines\n",
                           (unsigned long)buffer_index, (unsigned long)cleaned_size, (unsigned long)num_lines);
                }

                // Optional: print cleaned pixel buffer (moved to helper for readability)
#if DEBUG_VERBOSE
                debug_print_cleaned_dump(cleaned, cleaned_size, num_lines);
#endif

                // Render to ST7789 (using cleaned buffer)
                st7789_fill_screen(ST7789_WHITE);
                uint32_t max_display_lines = (num_lines > 240) ? 240 : num_lines;
                for (uint32_t line = 0; line < max_display_lines; line++) {
                    uint32_t pixel_start = line * 256;
                    for (uint32_t pixel = 0; pixel < 256; pixel++) {
                        uint32_t idx = pixel_start + pixel;
                        if (idx < cleaned_size && (cleaned[idx] & 0x80)) {
                            st7789_draw_pixel(32 + pixel, 32 + line, ST7789_BLACK);
                        }
                    }
                }

                // Send to thermal printer
                print_buffer_to_thermal(cleaned, num_lines, false);  // is_copy ignored
                uart_puts(PRINTER_UART, "\n\n\n");

                // Reset DMA for next capture
                dma_channel_abort(dma_chan);
                dma_channel_set_write_addr(dma_chan, buffer, false);
                dma_channel_set_trans_count(dma_chan, BUFFER_SIZE, false);
                dma_channel_start(dma_chan);
                buffer_index = 0;
                idle_count = 0;
            }
            sleep_us(1);
        }
    }
}
