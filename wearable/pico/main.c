#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/adc.h"
#include "hardware/sync.h"
#include "imu.h"
#include "../core/packet.h"
#include "../core/ringbuffer.h"
#include "../core/builder.h"
#include "flashlog.h"
#define PIN_VBUS_SENSE 24

#define LINK_UART uart1
#define LINK_BAUD 115200
#define PIN_UART_TX 4
#define PIN_UART_RX 5
#define PIN_LED PICO_DEFAULT_LED_PIN
#define PIN_VSYS_ADC 29
#define ADC_CH_VSYS 3
#define TICK_MS 50
#define TICKS_PER_SEC (1000/ TICK_MS)
#define LOW_BATT_MV 3400

static ringbuf_t rb;
static builder_t builder;
static volatile bool packet_due = false;
static bool logging = false;
static repeating_timer_t tick_timer;

static volatile uint32_t isr_count = 0;

static void imu_irq(uint gpio, uint32_t events) {
    if (gpio != IMU_PIN_INT1 || !(events & GPIO_IRQ_EDGE_RISE)) {
        return;
    }
    isr_count++;
    imu_sample_t s;
    imu_read(&s);
    rb_push(&rb, &s);
}


static bool tick_cb(repeating_timer_t *t) {
    (void)t;
    packet_due = true;
    return true;
}

static void link_init(void) {
    uart_init(LINK_UART, LINK_BAUD);
    uart_set_format(LINK_UART, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(LINK_UART, false, false);


    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);



}


static void send_frame(const uint8_t *pkt, uint8_t len) {
    const uint8_t hdr[3] = {0xAA, 0x55, len};
    uart_write_blocking(LINK_UART, hdr, sizeof hdr);
    uart_write_blocking(LINK_UART, pkt, len );

    if (logging) {
        logging = flashlog_write(hdr, sizeof hdr) && flashlog_write(pkt, len);
    }
}

static void batt_init(void) {
    adc_init();
    adc_gpio_init(PIN_VSYS_ADC);
    adc_select_input(ADC_CH_VSYS);
}


static uint32_t batt_read_mv(void) {
    uint32_t sum = 0;
    for (int i = 0; i < 4; i ++) {
        sum += adc_read();

    }
    return (sum/4) * 3300u * 3u / 4095u;
}


static uint8_t batt_byte(uint32_t mv) {
    if(mv <= 2500) {
        return 0;
    }
    uint32_t v = (mv - 2500) / 10;
    return v > 255 ? 255 : (uint8_t)v;
}

static void handle_cmd(void) {
    int c = getchar_timeout_us(0);
    if(c == PICO_ERROR_TIMEOUT) {
        return;
    }
    if (c=='d') {
        flashlog_dump();
    } else if (c == 'E') {
        printf("erasing \n");
        flashlog_erase();
        printf("erased\n");
    } else if(c == 'i') {
        printf(" log %lu / %lu bytes \n", (unsigned long)flashlog_used(), (unsigned long)flashlog_capacity());
    }
}


int main(void) {
    stdio_init_all();
    sleep_ms(3000);
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    link_init();

    batt_init();
    gpio_init(PIN_VBUS_SENSE);
    gpio_set_dir(PIN_VBUS_SENSE, GPIO_IN);
    bool on_usb = gpio_get(PIN_VBUS_SENSE);
    flashlog_init();
    logging = !on_usb;


    rb_init(&rb);
    builder_init(&builder);

    bool imu_alive = imu_init();
    printf("imu_init = %d\n" , imu_alive);

    if(imu_alive) {
        gpio_set_irq_enabled_with_callback(IMU_PIN_INT1, GPIO_IRQ_EDGE_RISE, true, &imu_irq);

    }

    add_repeating_timer_ms(-TICK_MS, tick_cb, NULL, &tick_timer);

    packet_t pkt;
    uint8_t buf[PKT_MAX_LEN];
    uint32_t tick = 0;
    int empty_streak = 0;
    uint32_t mv = batt_read_mv();

    while(true) {
        if(!(packet_due)) {
            __wfi();
            continue;
        }
        packet_due = false;
        tick++;

        if(tick % TICKS_PER_SEC == 0) {
            if (imu_alive) {
                imu_alive = imu_ok() && empty_streak < TICKS_PER_SEC;
                
            }
            if(!imu_alive) {
                gpio_set_irq_enabled(IMU_PIN_INT1, GPIO_IRQ_EDGE_RISE, false);
                imu_alive = imu_init();
                if (imu_alive) {
                    empty_streak = 0;
                    gpio_set_irq_enabled_with_callback(IMU_PIN_INT1, GPIO_IRQ_EDGE_RISE, true, &imu_irq);
                }
                printf("imu reinit: %d\n", imu_alive);
            }
                mv = batt_read_mv();
        }
    

        uint8_t extra = 0;
        if (!imu_alive) {
            extra |= PKT_FLAG_IMU_ERR;

        }


        if(mv < LOW_BATT_MV) {
            extra |= PKT_FLAG_LOW_BATT;
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());
        builder_build(&builder, &rb, now, batt_byte(mv), extra, &pkt);

        empty_streak = (pkt.n == 0) ? empty_streak + 1 : 0;
        if (empty_streak >=2 ){
            pkt.flags |= PKT_FLAG_IMU_ERR;
        }

        int len = packet_encode(&pkt, buf, sizeof(buf));
        if(len > 0) {
            send_frame(buf, (uint8_t)(len));
        }
        if(tick % 10 == 0) {
            gpio_put(PIN_LED, !gpio_get(PIN_LED));
        }

        if(tick % TICKS_PER_SEC == 0) {
            printf("seq = %u | n = %u | mv = %lu | flags = 0x%02x | ovr = %lu | isr_count = %lu\n", pkt.seq, pkt.n, (unsigned long)mv, pkt.flags, (unsigned long)rb_overruns(&rb), (unsigned long)(isr_count));
        }

        if (on_usb) {
            handle_cmd();
        }
    }
}