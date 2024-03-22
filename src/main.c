#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/display.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <lvgl.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>

#define VEXT_PIN  DT_GPIO_PIN(DT_NODELABEL(vext), gpios)
#define OLED_RST  DT_GPIO_PIN(DT_NODELABEL(oledrst), gpios)

static int board_heltec_wifi_lora32_v2_init(void)
{
	const struct device *gpio;

	gpio = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	if (!device_is_ready(gpio)) {
		return -ENODEV;
	}

	/* turns external VCC on  */
	gpio_pin_configure(gpio, VEXT_PIN, GPIO_OUTPUT);
	gpio_pin_set_raw(gpio, VEXT_PIN, 0);

	return 0;
}

SYS_INIT(board_heltec_wifi_lora32_v2_init, PRE_KERNEL_2, CONFIG_GPIO_INIT_PRIORITY);


#define DISPLAY_BUFFER_PITCH 128

LOG_MODULE_REGISTER(display);

static const struct device *display = DEVICE_DT_GET(DT_NODELABEL(ssd1306));

static uint32_t count;

int main(void)
{
  char count_str[11] = {0};
  lv_obj_t *count_label;
  
  LOG_ERR("Hello World!");

  if (display == NULL) {
    LOG_ERR("device pointer is NULL");
    return -1;
  }

  if (!device_is_ready(display)) {
    LOG_ERR("display device is not ready");
    return -1;
  }

  lv_disp_t * lv_disp = lv_disp_get_default();
  lv_theme_default_init(lv_disp, lv_color_black(), lv_color_white(), 1, &lv_font_montserrat_14);

  const struct spi_dt_spec sx1278_dev =
                SPI_DT_SPEC_GET(DT_NODELABEL(sx1276), 0, 0);
                
  LOG_INF("sx1278_dev.bus = %p", sx1278_dev.bus);
  LOG_INF("sx1278_dev.config.cs.gpio.port = %p", sx1278_dev.config.cs.gpio.port);
  LOG_INF("sx1278_dev.config.cs.gpio.pin = %u", sx1278_dev.config.cs.gpio.pin);

  struct display_capabilities capabilities;
  display_get_capabilities(display, &capabilities);

  const uint16_t x_res = capabilities.x_resolution;
  const uint16_t y_res = capabilities.y_resolution;

  LOG_INF("x_resolution: %d", x_res);
  LOG_INF("y_resolution: %d", y_res);
  LOG_INF("supported pixel formats: %d", capabilities.supported_pixel_formats);
  LOG_INF("screen_info: %d", capabilities.screen_info);
  LOG_INF("current_pixel_format: %d", capabilities.current_pixel_format);
  LOG_INF("current_orientation: %d", capabilities.current_orientation);
	 
  count_label = lv_label_create(lv_scr_act());
	lv_obj_align(count_label, LV_ALIGN_BOTTOM_MID, 0, 0);

	lv_task_handler();
	display_blanking_off(display);

	while (1) {
		if ((count % 100) == 0U) {
			sprintf(count_str, "%d", count/100U);
			lv_label_set_text(count_label, count_str);
		}
		lv_task_handler();
		++count;
		k_sleep(K_MSEC(10));
	}

  return 0; 
}
