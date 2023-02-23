// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Analog Devices AD7606X Parallel Interface ADC driver
 *
 * Copyright 2023 Analog Devices Inc.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/mod_devicetable.h>

#include "cf_axi_adc.h"

#define AD7606X_VREF		2500

#define KHz 1000
#define MHz (1000 * KHz)

#define AD7606X_MULTIPLE_CHAN(_idx, _storagebits, _realbits, _shift)	\
	{								\
		.type = IIO_VOLTAGE,					\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
		.indexed = 1,						\
		.channel = _idx,					\
		.scan_index = _idx,					\
		.scan_type = {						\
			.sign = 's',					\
			.storagebits = _storagebits,			\
			.realbits = _realbits,				\
			.shift = _shift,				\
		},							\
	}

enum ad7606x_id {
	ID_AD7606X_16,
};

struct ad7606x_info {
	struct iio_chan_spec channels[8];
	int num_channels;
	int resolution;
};

static const struct ad7606x_info ad7606x_infos[] = {
	[ID_AD7606X_16] = {
		.resolution = 16,
		.channels = {
			AD7606X_MULTIPLE_CHAN(0, 128, 16, 0),
			AD7606X_MULTIPLE_CHAN(1, 128, 16, 16),
			AD7606X_MULTIPLE_CHAN(2, 128, 16, 32),
			AD7606X_MULTIPLE_CHAN(3, 128, 16, 48),
			AD7606X_MULTIPLE_CHAN(4, 128, 16, 64),
			AD7606X_MULTIPLE_CHAN(5, 128, 16, 80),
			AD7606X_MULTIPLE_CHAN(6, 128, 16, 96),
			AD7606X_MULTIPLE_CHAN(7, 128, 16, 112),
		},
		.num_channels = 8,
	},
};

struct ad7606x_dev {
	const struct ad7606x_info *device_info;
/*	struct gpio_desc *adc_serpar;
	struct gpio_desc *adc_refsel;*/
	struct gpio_desc *adc_reset;
	struct gpio_desc *adc_standby;
	struct gpio_desc *adc_range;
	struct gpio_descs *adc_os;
	unsigned long ref_clk_rate;
	struct pwm_device *cnvst_n;
	struct regulator *vref;
	struct clk *ref_clk;

	unsigned int vref_mv;
	int sampling_freq;
};


static int ad7606x_set_sampling_freq(struct ad7606x_dev *ad7606x, int freq)
{
	unsigned long long target, ref_clk_period_ps;
	struct pwm_state cnvst_n_state;
	int ret;//, clk_en_time;

	target = DIV_ROUND_CLOSEST_ULL(ad7606x->ref_clk_rate, freq);
	ref_clk_period_ps = DIV_ROUND_CLOSEST_ULL(1000000000000ULL,
						  ad7606x->ref_clk_rate);
	cnvst_n_state.period = ref_clk_period_ps * target;
	cnvst_n_state.duty_cycle = ref_clk_period_ps;
	cnvst_n_state.phase = ref_clk_period_ps;
	cnvst_n_state.time_unit = PWM_UNIT_PSEC;
	cnvst_n_state.enabled = true;
	ret = pwm_apply_state(ad7606x->cnvst_n, &cnvst_n_state);
	if (ret < 0)
		return ret;

	ad7606x->sampling_freq = DIV_ROUND_CLOSEST_ULL(ad7606x->ref_clk_rate, target);

	return 0;
}

static int ad7606x_setup(struct iio_dev *indio_dev)
{
	struct ad7606x_dev *ad7606x = iio_priv(indio_dev);
	struct device *dev = indio_dev->dev.parent;

	return ad7606x_set_sampling_freq(ad7606x, 100 * KHz);
}

static int ad7606x_post_setup(struct iio_dev *indio_dev);
{
	struct axiadc_state *axi_adc_st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	int i;

	for (i=0; i < conv->chip_info->num_channels; i++)
		axiadc_write(axi_adc_st, ADI_REG_CHAN_CNTRL(i), ADI_ENABLE | ADI_FORMAT_ENABLE | ADI_FORMAT_SIGNEXT);

	return 0;
}

static int ad7606x_read_raw(struct iio_dev *indio_dev,
			    const struct iio_chan_spec *chan,
			    int *val, int *val2, long info)
{
	struct ad7606x_dev *ad7606x = iio_priv(indio_dev);
	unsigned int temp;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = ad7606x->sampling_freq;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		temp = regulator_get_voltage(ad7606x->vref);
		if (temp < 0)
			return temp;

		*val = (temp * 2) / 1000;
		*val2 = chan->scan_type.realbits;

		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int ad7606x_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long info)
{
	struct ad7606x_dev *ad7606x = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad7606x_set_sampling_freq(ad7606x, val);

	default:
		return -EINVAL;
	}
}

static int ad7606x_dma_submit(struct iio_dma_buffer_queue *queue,
			      struct iio_dma_buffer_block *block)
{
	return iio_dmaengine_buffer_submit_block(queue, block, DMA_DEV_TO_MEM);
}

static void ad7606x_pwm_diasble(void *data)
{
	pwm_disable(data);
}

static void ad7606x_regulator_disable(void *data)
{
	regulator_disable(data);
}

static void ad7606x_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static const struct iio_dma_buffer_ops ad7606x_dma_buffer_ops = {
	.submit = ad7606x_dma_submit,
	.abort = iio_dmaengine_buffer_abort,
};

static const struct iio_info ad7606x_info = {
	.read_raw = ad7606x_read_raw,
	.write_raw = ad7606x_write_raw,
};

static const struct of_device_id ad7606x_of_match[] = {
	{
		.compatible = "ad7606x-16",
		.data = &ad7606x_infos[ID_AD7606X_16]
	},
	{}
};
MODULE_DEVICE_TABLE(of, ad7606x_of_match);

static int ad7606x_probe(struct platform_device *pdev)
{
	struct iio_dev			*indio_dev;
	struct iio_buffer		*buffer;
	struct ad7606x_dev		*ad7606x;
	int				ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*ad7606x));
	if (!indio_dev)
		return -ENOMEM;

	ad7606x = iio_priv(indio_dev);
/*
	ad7606x->adc_serpar = devm_gpiod_get_optional(&pdev->dev, "adi,adc_serpar", GPIOD_OUT_LOW);
	if (IS_ERR(ad7606x->adc_serpar)) {
		return PTR_ERR(ad7606x->adc_serpar);
	}

	ad7606x->adc_refsel = devm_gpiod_get_optional(&pdev->dev, "adi,adc_refsel", GPIOD_OUT_LOW);
	if (IS_ERR(ad7606x->adc_refsel)) {
		return PTR_ERR(ad7606x->adc_refsel);
	}
*/
	ad7606x->adc_reset = devm_gpiod_get_optional(&pdev->dev, "adi,adc_reset", GPIOD_OUT_LOW);
	if (IS_ERR(ad7606x->adc_reset)) {
		return PTR_ERR(ad7606x->adc_reset);
	}

	ad7606x->adc_range = devm_gpiod_get_optional(&pdev->dev, "adi,adc_range", GPIOD_OUT_HIGH);
	if (IS_ERR(ad7606x->adc_range)) {
		return PTR_ERR(ad7606x->adc_range);
	}

	ad7606x->adc_standby = devm_gpiod_get_optional(&pdev->dev, "adi,adc_standby", GPIOD_OUT_HIGH);
	if (IS_ERR(ad7606x->adc_standby)) {
		return PTR_ERR(ad7606x->adc_standby);
	}

	ad7606x->adc_os = devm_gpiod_get_array_optional(&pdev->dev, "adi,adc_os", GPIOD_OUT_HIGH);
	if (IS_ERR(ad7606x->adc_os)) {
		return PTR_ERR(ad7606x->adc_os);
	}

	ad7606x->vref = devm_regulator_get_optional(&pdev->dev, "vref");
	if (!IS_ERR(ad7606x->vref)) {
		ret = regulator_enable(ad7606x->vref);
		if (ret) {
			dev_err(&pdev->dev, "Can't to enable vref regulator\n");
			return ret;
		}
		ret = regulator_get_voltage(ad7606x->vref);
		if (ret < 0)
			return ret;

		ad7606x->vref_mv = ret / 1000;
		ret = devm_add_action_or_reset(&pdev->dev,
					       ad7606x_regulator_disable,
					       ad7606x->vref);
		if (ret)
			return ret;
	} else {
		if (PTR_ERR(ad7606x->vref) != -ENODEV)
			return PTR_ERR(ad7606x->vref);

		ad7606x->vref_mv = AD7606X_VREF; /* Internal vref is used */
	}

	ad7606x->ref_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(ad7606x->ref_clk))
		return PTR_ERR(ad7606x->ref_clk);

	ret = clk_prepare_enable(ad7606x->ref_clk);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&pdev->dev, ad7606x_clk_disable,
				       ad7606x->ref_clk);
	if (ret)
		return ret;
	ad7606x->ref_clk_rate = clk_get_rate(ad7606x->ref_clk);

	ad7606x->cnvst_n = devm_pwm_get(&pdev->dev, "cnvst_n");
	if (IS_ERR(ad7606x->cnvst_n))
		return PTR_ERR(ad7606x->cnvst_n);

	ret = devm_add_action_or_reset(&pdev->dev, ad7606x_pwm_diasble,
				       ad7606x->cnvst_n);
	if (ret)
		return ret;

	ad7606x->device_info = device_get_match_data(&pdev->dev);
	if (!ad7606x->device_info)
		return -EINVAL;
	indio_dev->channels = ad7606x->device_info->channels;
	indio_dev->num_channels = ad7606x->device_info->num_channels;
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->info = &ad7606x_info;
	indio_dev->modes = INDIO_BUFFER_HARDWARE;

	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent,
						 "rx",
						 &ad7606x_dma_buffer_ops,
						 indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);
	ret = ad7606x_setup(indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "\nAD7606X setup failed\n");
		return ret;
	}

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static struct platform_driver ad7606x_driver = {
	.probe          = ad7606x_probe,
	.driver         = {
		.name   = "ad7606x",
		.of_match_table = ad7606x_of_match,
	},
};
module_platform_driver(ad7606x_driver);

MODULE_AUTHOR("Alin-Tudor Sferle <alin-tudor.sferle@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7606X Parallel Interface ADC");
MODULE_LICENSE("Dual BSD/GPL");
