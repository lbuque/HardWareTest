#define LGFX_USE_V1
#include <LovyanGFX.hpp>

#include <Arduino.h>
#include <esp_log.h>
#include "esp_err.h"


#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#define BUZZER_PIN 6
#define BK_PIN 5
#define SCK_PIN 1
#define MOSI_PIN 3
#define MISO_PIN 3
#define CS_PIN 2
#define DC_PIN 0
#define RESET_PIN 4
#define LCD_HOST  SPI2_HOST
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (4 * 1000 * 1000)


// select one of the following options
#define USE_LGFX 1
#define USE_SPI_DRIVER 0

#if USE_LGFX
class C3GFX: public lgfx::LGFX_Device {
    lgfx::Panel_ST7789P3 _panel_instance;
    lgfx::Bus_SPI       _spi_bus_instance;
    lgfx::Light_PWM     _light_instance;

public:

    C3GFX() {

        {   
            auto cfg = _light_instance.config();
            cfg.pin_bl = BK_PIN;
            cfg.invert = false;
            cfg.freq = 1000;
            cfg.pwm_channel = 5;

            _light_instance.config(cfg);
        }

        {
            auto cfg = _spi_bus_instance.config();

            cfg.spi_host = LCD_HOST;
            cfg.spi_mode = 0;
            cfg.freq_write = 4 * 1000 * 1000;
            cfg.freq_read = 4 * 1000 * 1000;
            cfg.spi_3wire = true;
            // cfg.use_lock = true;
            // cfg.dma_channel = 3; // AUTO
            cfg.pin_sclk = SCK_PIN;
            cfg.pin_mosi = MOSI_PIN;
            cfg.pin_miso = -1;
            cfg.pin_dc = -1;

            _spi_bus_instance.config(cfg);
        }

        {
            auto cfg = _panel_instance.config();

            cfg.invert       = false;
            cfg.pin_cs       = CS_PIN;
            cfg.pin_rst      = RESET_PIN;
            cfg.pin_busy     = -1;
            cfg.panel_width  = 320;
            cfg.panel_height = 240;
            cfg.offset_x     = 0;
            cfg.offset_y     = 0;

            // cfg.offset_rotation = 0;
            // cfg.dummy_read_pixel = 8;
            // cfg.dummy_read_bits = 1;
            // cfg.readable = true;
            // cfg.invert = false;
            cfg.rgb_order = false;
            // cfg.dlen_16bit = false;
            // cfg.bus_shared = false;

            _panel_instance.setBus(&_spi_bus_instance);
            _panel_instance.config(cfg);
        }

        _panel_instance.setLight(&_light_instance);
        setPanel(&_panel_instance);
    };

    // override the init()
    inline bool init(void)
    {
        // HW_Reset();
        lgfx::pinMode(GPIO_NUM_4, lgfx::pin_mode_t::output);
        lgfx::gpio_hi(GPIO_NUM_4);
        lgfx::delay(100);
        lgfx::gpio_lo(GPIO_NUM_4);
        lgfx::delay(120);
        lgfx::gpio_hi(GPIO_NUM_4);
        delay(120); // ms

        /* Lgfx */
        return lgfx::LGFX_Device::init();
    }
};

C3GFX lcd;
#endif

#if USE_SPI_DRIVER
spi_device_handle_t spi;
#endif

void setup() {
    // put your setup code here, to run once:
    ledcSetup(0, 4000, 8);

    ledcAttachPin(BUZZER_PIN, 0);

    ledcWriteTone(0, 4000);
    delay(50);
    ledcWriteTone(0, 0);

#if USE_LGFX
    lcd.init();
    lcd.setBrightness(255);
    lcd.clear(TFT_RED);
#endif

#if USE_SPI_DRIVER
    pinMode(BK_PIN, OUTPUT);
    digitalWrite(BK_PIN, HIGH);


    SPI_3Wire_Interface_Init();

    // HW_Reset();
    pinMode(RESET_PIN, OUTPUT);
    digitalWrite(RESET_PIN, HIGH);
    delay(100);
    digitalWrite(RESET_PIN, LOW);
    delay(120); // ms
    digitalWrite(RESET_PIN, HIGH);

    delay(120); // ms

    lcd_write_cmd(0x11);

    delay(120); // ms

    lcd_write_cmd(0x36);
    lcd_write_data(0x00);

    lcd_write_cmd(0x3A);
    lcd_write_data(0x05);

    lcd_write_cmd(0xB2);
    lcd_write_data(0x0C);
    lcd_write_data(0x0C);
    lcd_write_data(0x00);
    lcd_write_data(0x33);
    lcd_write_data(0x33);

    lcd_write_cmd(0xB7);
    lcd_write_data(0x56);

    lcd_write_cmd(0xBB);
    lcd_write_data(0x1D);

    lcd_write_cmd(0xC0);
    lcd_write_data(0x2C);

    lcd_write_cmd(0xC2);
    lcd_write_data(0x01);

    lcd_write_cmd(0xC3);
    lcd_write_data(0x0F);

    lcd_write_cmd(0xC6);
    lcd_write_data(0x0F);

    lcd_write_cmd(0xD0);
    lcd_write_data(0xA7);

    lcd_write_cmd(0xD0);
    lcd_write_data(0xA4);
    lcd_write_data(0xA1);

    lcd_write_cmd(0xD6);
    lcd_write_data(0xA1);

    lcd_write_cmd(0xE0);
    lcd_write_data(0xF0);
    lcd_write_data(0x02);
    lcd_write_data(0x07);
    lcd_write_data(0x05);
    lcd_write_data(0x06);
    lcd_write_data(0x14);
    lcd_write_data(0x2F);
    lcd_write_data(0x54);
    lcd_write_data(0x46);
    lcd_write_data(0x38);
    lcd_write_data(0x13);
    lcd_write_data(0x11);
    lcd_write_data(0x2E);
    lcd_write_data(0x35);

    lcd_write_cmd(0xE1);
    lcd_write_data(0xF0);
    lcd_write_data(0x08);
    lcd_write_data(0x0C);
    lcd_write_data(0x0C);
    lcd_write_data(0x09);
    lcd_write_data(0x05);
    lcd_write_data(0x2F);
    lcd_write_data(0x43);
    lcd_write_data(0x46);
    lcd_write_data(0x36);
    lcd_write_data(0x10);
    lcd_write_data(0x12);
    lcd_write_data(0x2C);
    lcd_write_data(0x32);

    lcd_write_cmd(0x21);

    lcd_write_cmd(0x29);

    lcd_write_cmd(0x2C);

    delay(100);

    lcd_write_cmd(0x2A);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x01);
    lcd_write_data(0x3F);

    lcd_write_cmd(0x2B);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0xEF);

    lcd_write_cmd(0x2C);
    for (int i = 0; i < 240 * 320; i++) {
        lcd_write_data(0xF8);
        lcd_write_data(0x00);
    }
#endif
}


void loop() {
    // put your main code here, to run repeatedly:
    delay(10);
}

#if USE_SPI_DRIVER
void SPI_3Wire_Interface_Init()
{
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.miso_io_num = -1;
    buscfg.mosi_io_num = MOSI_PIN;
    buscfg.sclk_io_num = SCK_PIN;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 4096; //= 4096 Byte
    buscfg.flags = SPICOMMON_BUSFLAG_DUAL;

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.command_bits = 1;     //D/Cx位，0 cmd， 1 data
    devcfg.mode = 0;   //CPOL, CPHA xSPI_CPOL_CPHA_mode = 0
    devcfg.clock_speed_hz = 4 * 1000 * 1000; //=1MHz
    devcfg.spics_io_num = CS_PIN;
    devcfg.flags = SPI_DEVICE_3WIRE | SPI_DEVICE_HALFDUPLEX; //3线半双工
    devcfg.queue_size = 8;

    //Initialize the SPI bus
    esp_err_t ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    log_i("ret: %d", ret);
    //Attach the LCD to the SPI bus
    ret = spi_bus_add_device(LCD_HOST, &devcfg, &spi);
    log_i("ret: %d", ret);
}


void lcd_write_cmd(const uint8_t cmd)
{
    log_i("lcd_write_cmd");
    spi_transaction_t sendcfg;
    memset(&sendcfg, 0, sizeof(sendcfg));
    sendcfg.cmd = 0;                        //cmd = 0
    sendcfg.length=8;
    sendcfg.tx_buffer = &cmd;
    esp_err_t ret = spi_device_polling_transmit(spi, &sendcfg);
    log_i("s cmd ret: %d", ret);
}


void lcd_write_data(const uint8_t data)
{
    log_i("lcd_write_data");
    spi_transaction_t sendcfg;
    memset(&sendcfg, 0, sizeof(sendcfg));
    sendcfg.cmd = 1;                        //data = 1
    sendcfg.length=8;
    sendcfg.tx_buffer = &data;
    esp_err_t ret = spi_device_polling_transmit(spi, &sendcfg);
    log_i("s data ret: %d", ret);
}
#endif