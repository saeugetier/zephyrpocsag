
#ifndef RADIO_FSK_H_
#define RADIO_FSK_H_

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>


enum fsk_baudrate
{
    BAUD_512,
    BAUD_1200,
    BAUD_2400
};

enum fsk_bandwith
{
    KHZ_12_5,
    KHZ_25,
    KHZ_50,
    KHZ_100
};

struct radio_fsk_config
{
    uint32_t frequency;

    enum fsk_baudrate baudrate;

    uint16_t preambleLen;

    uint8_t syncWord[8];

    uint8_t syncWordLen;
};


enum fsk_receive_callback_cause
{
    SIGNAL_DETECT,
    BIT_HIGH,
    BIT_LOW,
    TIMEOUT
};

typedef void (*radio_fsk_receive_cb)(const struct device *dev, enum fsk_receive_callback_cause cause);

typedef int (*radio_fsk_api_config)(const struct device *dev, struct radio_fsk_config *config);

typedef int (*radio_fsk_api_start_receive)(const struct device *dev, radio_fsk_receive_cb receive_cb);

struct radio_fsk_api
{
    radio_fsk_api_config config;
    radio_fsk_api_start_receive start_receive;
};


static inline int radio_fsk_config(const struct device *dev, struct radio_fsk_config *config)
{
	const struct radio_fsk_api *api =
		(const struct radio_fsk_api *)dev->api;

	return api->config(dev, config);
}


static inline int radio_fsk_start_receive(const struct device *dev, radio_fsk_receive_cb receive_cb)
{
	const struct radio_fsk_api *api =
		(const struct radio_fsk_api *)dev->api;

	return api->start_receive(dev, receive_cb);
}


#endif