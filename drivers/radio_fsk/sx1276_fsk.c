#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <string.h>

#include <errno.h>

#include "radio_fsk.h"
#include "sx1276_registers.h"

#define DT_DRV_COMPAT semtech_sx1276_fsk

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sx1276_fsk, CONFIG_RADIO_FSK_LOG_LEVEL);

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(semtech_sx1276_fsk) <= 1,
	     "Multiple SX127x instances in DT");

#define SX127X_DIO_GPIO_LEN(inst) \
	DT_INST_PROP_LEN(inst, dio_gpios)

#define SX127X_DIO_GPIO_ELEM(idx, inst) \
	GPIO_DT_SPEC_INST_GET_BY_IDX(inst, dio_gpios, idx)

#define SX127X_DIO_GPIO_INIT(n) \
	LISTIFY(SX127X_DIO_GPIO_LEN(n), SX127X_DIO_GPIO_ELEM, (,), n)

static const struct gpio_dt_spec sx127x_dios[] = { SX127X_DIO_GPIO_INIT(0) };

#define SX127X_MAX_DIO ARRAY_SIZE(sx127x_dios)

struct sx127x_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec reset;
#if DT_INST_NODE_HAS_PROP(0, antenna_enable_gpios)
	struct gpio_dt_spec antenna_enable;
#endif
#if DT_INST_NODE_HAS_PROP(0, rfi_enable_gpios)
	struct gpio_dt_spec rfi_enable;
#endif
#if DT_INST_NODE_HAS_PROP(0, rfo_enable_gpios)
	struct gpio_dt_spec rfo_enable;
#endif
#if DT_INST_NODE_HAS_PROP(0, pa_boost_enable_gpios)
	struct gpio_dt_spec pa_boost_enable;
#endif
#if DT_INST_NODE_HAS_PROP(0, tcxo_power_gpios)
	struct gpio_dt_spec tcxo_power;
#endif
};

static const struct sx127x_config dev_config = {
	.bus = SPI_DT_SPEC_INST_GET(0, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),
	.reset = GPIO_DT_SPEC_INST_GET(0, reset_gpios),
#if DT_INST_NODE_HAS_PROP(0, antenna_enable_gpios)
	.antenna_enable = GPIO_DT_SPEC_INST_GET(0, antenna_enable_gpios),
#endif
#if DT_INST_NODE_HAS_PROP(0, rfi_enable_gpios)
	.rfi_enable = GPIO_DT_SPEC_INST_GET(0, rfi_enable_gpios),
#endif
#if DT_INST_NODE_HAS_PROP(0, rfo_enable_gpios)
	.rfo_enable = GPIO_DT_SPEC_INST_GET(0, rfo_enable_gpios),
#endif
#if DT_INST_NODE_HAS_PROP(0, pa_boost_enable_gpios)
	.pa_boost_enable = GPIO_DT_SPEC_INST_GET(0, pa_boost_enable_gpios),
#endif
#if DT_INST_NODE_HAS_PROP(0, tcxo_power_gpios)
	.tcxo_power = GPIO_DT_SPEC_INST_GET(0, tcxo_power_gpios),
#endif
};

int __sx1276_fsk_configure_pin(const struct gpio_dt_spec *gpio, gpio_flags_t flags)
{
	int err;

	if (!device_is_ready(gpio->port)) {
		LOG_ERR("GPIO device not ready %s", gpio->port->name);
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(gpio, flags);
	if (err) {
		LOG_ERR("Cannot configure gpio %s %d: %d", gpio->port->name,
			gpio->pin, err);
		return err;
	}

	return 0;
}

#define sx1276_fsk_configure_pin(_name, _flags)				\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(0, _name##_gpios),		\
		    (__sx1276_fsk_configure_pin(&dev_config._name, _flags)),\
		    (0))

struct sx1276_fsk_data
{

};

struct sx1276_fsk_conf
{
    struct sx1276_fsk_data *data;           // Pointer to runtime data.
    const struct device *spi_dev;       // SPI device.
    const struct gpio_dt_spec gpio_spec; // GPIO spec for pin used to start transmitting.
    struct gpio_callback gpio_cb;        // GPIO callback
};


static int sx1276_fsk_transceive(uint8_t reg, bool write, void *data, size_t length)
{
	const struct spi_buf buf[2] = {
		{
			.buf = &reg,
			.len = sizeof(reg)
		},
		{
			.buf = data,
			.len = length
		}
	};

	struct spi_buf_set tx = {
		.buffers = buf,
		.count = ARRAY_SIZE(buf),
	};

	if (!write) {
		const struct spi_buf_set rx = {
			.buffers = buf,
			.count = ARRAY_SIZE(buf)
		};

		return spi_transceive_dt(&dev_config.bus, &tx, &rx);
	}

	return spi_write_dt(&dev_config.bus, &tx);
}


int sx1276_fsk_read(uint8_t reg_addr, uint8_t *data, uint8_t len)
{
	return sx1276_fsk_transceive(reg_addr, false, data, len);
}

int sx1276_fsk_write(uint8_t reg_addr, uint8_t *data, uint8_t len)
{
	return sx1276_fsk_transceive(reg_addr | BIT(7), true, data, len);
}

//static inline int sx1276_fsk_set_bitrate(const struct device *dev, )

static int sx1276_fsk_init(const struct device *dev)
{
    int ret;
	uint8_t regval;

	if (!spi_is_ready_dt(&dev_config.bus)) {
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}

	ret = sx1276_fsk_configure_pin(reset, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		return ret;
	}

	k_sleep(K_MSEC(100));
	gpio_pin_set_dt(&dev_config.reset, 0);
	k_sleep(K_MSEC(100));

	uint8_t mode = 0; //sleep mode
	sx1276_fsk_read(REG_CHIP_REVISION, &mode, 1);
	mode = 0;
	sx1276_fsk_write(REG_OP_MODE, &mode, 1);
	mode = 0x8 | 0x01; // FSK | FSK Modulation | Low Frequency Mode | Standby
	sx1276_fsk_write(REG_OP_MODE, &mode, 1);
	
	uint8_t buffer[0x42];

	sx1276_fsk_read(REG_OP_MODE, &buffer[0], 0x42);


	return 0;

}

static const struct radio_fsk_api sx1276_api =
{
    .config = NULL,
    .start_receive = NULL
};



DEVICE_DT_INST_DEFINE(0, &sx1276_fsk_init, NULL, NULL,
		      NULL, POST_KERNEL, CONFIG_RADIO_FSK_INIT_PRIORITY,
		      NULL);