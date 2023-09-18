// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * mcp4922.c
 *
 * Driver for Microchip Digital to Analog Converters.
 * Supports MCP4902, MCP4912, MCP4921, MCP4922, MCP4801, MCP4802, MCP4811,
 * MCP4812, MCP4821, and MCP4822.
 *
 * Copyright (c) 2014 EMAC Inc.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/regulator/consumer.h>
#include <linux/bitops.h>

#define MCP4922_NUM_CHANNELS		2
#define MCP4921_NUM_CHANNELS		1
#define MCP48XX_INTERNAL_VREF_MV	2048

enum mcp4922_supported_device_ids {
	ID_MCP4902,
	ID_MCP4912,
	ID_MCP4921,
	ID_MCP4922,
	ID_MCP4801,
	ID_MCP4802,
	ID_MCP4811,
	ID_MCP4812,
	ID_MCP4821,
	ID_MCP4822,
};

struct mcp4922_state {
	struct spi_device *spi;
	unsigned int value[MCP4922_NUM_CHANNELS];
	unsigned int vref_mv;
	struct regulator *vref_reg;
	struct regulator *vdd_reg;
	u8 mosi[2] __aligned(IIO_DMA_MINALIGN);
};

#define MCP4922_CHAN(chan, bits) {			\
	.type = IIO_VOLTAGE,				\
	.output = 1,					\
	.indexed = 1,					\
	.channel = chan,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.scan_type = {					\
		.sign = 'u',				\
		.realbits = (bits),			\
		.storagebits = 16,			\
		.shift = 12 - (bits),			\
	},						\
}

static bool mcp4922_needs_vref(int device_id)
{
	switch (device_id) {
	case ID_MCP4902:
	case ID_MCP4912:
	case ID_MCP4921:
	case ID_MCP4922:
		return true;
	default:
		return false;
	}
}

static int mcp4922_num_channels(int device_id)
{
	switch (device_id) {
	case ID_MCP4902:
	case ID_MCP4912:
	case ID_MCP4922:
	case ID_MCP4802:
	case ID_MCP4812:
	case ID_MCP4822:
		return MCP4922_NUM_CHANNELS;
	default:
		return MCP4921_NUM_CHANNELS;
	}

}

static int mcp4922_spi_write(struct mcp4922_state *state, u8 addr, u32 val)
{
	state->mosi[1] = val & 0xff;
	state->mosi[0] = (addr == 0) ? 0x00 : 0x80;
	state->mosi[0] |= 0x30 | ((val >> 8) & 0x0f);

	return spi_write(state->spi, state->mosi, 2);
}

static int mcp4922_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan,
		int *val,
		int *val2,
		long mask)
{
	struct mcp4922_state *state = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		*val = state->value[chan->channel];
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = state->vref_mv;
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int mcp4922_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan,
		int val,
		int val2,
		long mask)
{
	struct mcp4922_state *state = iio_priv(indio_dev);
	int ret;

	if (val2 != 0)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val < 0 || val > GENMASK(chan->scan_type.realbits - 1, 0))
			return -EINVAL;
		val <<= chan->scan_type.shift;

		ret = mcp4922_spi_write(state, chan->channel, val);
		if (!ret)
			state->value[chan->channel] = val;
		return ret;

	default:
		return -EINVAL;
	}
}

static const struct iio_chan_spec mcp4922_channels[10][MCP4922_NUM_CHANNELS] = {
	[ID_MCP4902] = { MCP4922_CHAN(0, 8),	MCP4922_CHAN(1, 8) },
	[ID_MCP4912] = { MCP4922_CHAN(0, 10),	MCP4922_CHAN(1, 10) },
	[ID_MCP4921] = { MCP4922_CHAN(0, 12),	{} },
	[ID_MCP4922] = { MCP4922_CHAN(0, 12),	MCP4922_CHAN(1, 12) },
	[ID_MCP4801] = { MCP4922_CHAN(0, 8),	{} },
	[ID_MCP4802] = { MCP4922_CHAN(0, 8),	MCP4922_CHAN(1, 8) },
	[ID_MCP4811] = { MCP4922_CHAN(0, 10),	{} },
	[ID_MCP4812] = { MCP4922_CHAN(0, 10),	MCP4922_CHAN(1, 10) },
	[ID_MCP4821] = { MCP4922_CHAN(0, 12),	{} },
	[ID_MCP4822] = { MCP4922_CHAN(0, 12),	MCP4922_CHAN(1, 12) },
};

static const struct iio_info mcp4922_info = {
	.read_raw = &mcp4922_read_raw,
	.write_raw = &mcp4922_write_raw,
};

static int mcp4922_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct mcp4922_state *state;
	const struct spi_device_id *id;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*state));
	if (indio_dev == NULL)
		return -ENOMEM;

	state = iio_priv(indio_dev);
	state->spi = spi;
	id = spi_get_device_id(spi);
	if (mcp4922_needs_vref(id->driver_data)) {
		state->vref_reg = devm_regulator_get(&spi->dev, "vref");
		if (IS_ERR(state->vref_reg))
			return dev_err_probe(&spi->dev, PTR_ERR(state->vref_reg),
					"Vref regulator not specified\n");

		ret = regulator_enable(state->vref_reg);
		if (ret) {
			dev_err(&spi->dev, "Failed to enable vref regulator: %d\n",
					ret);
			return ret;
		}

		ret = regulator_get_voltage(state->vref_reg);
		if (ret < 0) {
			dev_err(&spi->dev, "Failed to read vref regulator: %d\n",
					ret);
			goto error_disable_vref_reg;
		}
		state->vref_mv = ret / 1000;
	} else {
		state->vref_mv = MCP48XX_INTERNAL_VREF_MV;
	}


	state->vdd_reg = devm_regulator_get(&spi->dev, "vdd");
	if (IS_ERR(state->vdd_reg)) {
		ret = dev_err_probe(&spi->dev, PTR_ERR(state->vdd_reg),
				    "vdd regulator not specified\n");
		goto error_disable_vref_reg;
	}
	ret = regulator_enable(state->vdd_reg);
	if (ret) {
		dev_err(&spi->dev, "Failed to enable vdd regulator: %d\n",
			ret);
		goto error_disable_vref_reg;
	}

	spi_set_drvdata(spi, indio_dev);
	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &mcp4922_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = mcp4922_channels[id->driver_data];
	indio_dev->num_channels = mcp4922_num_channels(id->driver_data);
	indio_dev->name = id->name;

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&spi->dev, "Failed to register iio device: %d\n",
				ret);
		goto error_disable_vdd_reg;
	}

	return 0;
error_disable_vdd_reg:
	regulator_disable(state->vdd_reg);
error_disable_vref_reg:
	if (mcp4922_needs_vref(id->driver_data))
		regulator_disable(state->vref_reg);


	return ret;
}

static int mcp4922_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct mcp4922_state *state;
	const struct spi_device_id *id = spi_get_device_id(spi);

	iio_device_unregister(indio_dev);
	state = iio_priv(indio_dev);
	regulator_disable(state->vdd_reg);
	if (mcp4922_needs_vref(id->driver_data)) {
		regulator_disable(state->vref_reg);
	}

	return 0;
}

static const struct spi_device_id mcp4922_id[] = {
	{"mcp4902", ID_MCP4902},
	{"mcp4912", ID_MCP4912},
	{"mcp4921", ID_MCP4921},
	{"mcp4922", ID_MCP4922},
	{"mcp4801", ID_MCP4801},
	{"mcp4802", ID_MCP4802},
	{"mcp4811", ID_MCP4811},
	{"mcp4812", ID_MCP4812},
	{"mcp4821", ID_MCP4821},
	{"mcp4822", ID_MCP4822},
	{}
};
MODULE_DEVICE_TABLE(spi, mcp4922_id);

static struct spi_driver mcp4922_driver = {
	.driver = {
		   .name = "mcp4922",
		   },
	.probe = mcp4922_probe,
	.remove = mcp4922_remove,
	.id_table = mcp4922_id,
};
module_spi_driver(mcp4922_driver);

MODULE_AUTHOR("Michael Welling <mwelling@ieee.org>");
MODULE_DESCRIPTION("Microchip MCP49XX and MCP48XX DAC");
MODULE_LICENSE("GPL v2");
