// SPDX-License-Identifier: GPL-2.0-only

/*
 * Analog Devices AD3552R
 * Digital to Analog converter driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/crc8.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gcd.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/iio.h>
#include <linux/iio/sw_trigger.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/time64.h>
#include <linux/unaligned/be_byteshift.h>

#define AD3552R_NUM_CH					2

/* Register addresses */
/* Primary address space */
#define AD3552R_REG_ADDR_INTERFACE_CONFIG_A		0x00
#define AD3552R_REG_ADDR_INTERFACE_CONFIG_B		0x01
#define AD3552R_REG_ADDR_DEVICE_CONFIG			0x02
#define AD3552R_REG_ADDR_CHIP_TYPE			0x03
#define AD3552R_REG_ADDR_PRODUCT_ID_L			0x04
#define AD3552R_REG_ADDR_PRODUCT_ID_H			0x05
#define AD3552R_REG_ADDR_CHIP_GRADE			0x06
#define AD3552R_REG_ADDR_SCRATCH_PAD			0x0A
#define AD3552R_REG_ADDR_SPI_REVISION			0x0B
#define AD3552R_REG_ADDR_VENDOR_L			0x0C
#define AD3552R_REG_ADDR_VENDOR_H			0x0D
#define AD3552R_REG_ADDR_STREAM_MODE			0x0E
#define AD3552R_REG_ADDR_TRANSFER_REGISTER		0x0F
#define AD3552R_REG_ADDR_INTERFACE_CONFIG_C		0x10
#define AD3552R_REG_ADDR_INTERFACE_STATUS_A		0x11
#define AD3552R_REG_ADDR_INTERFACE_CONFIG_D		0x14
#define AD3552R_REG_ADDR_SH_REFERENCE_CONFIG		0x15
#define AD3552R_REG_ADDR_ERR_ALARM_MASK			0x16
#define AD3552R_REG_ADDR_ERR_STATUS			0x17
#define AD3552R_REG_ADDR_POWERDOWN_CONFIG		0x18
#define AD3552R_REG_ADDR_CH0_CH1_OUTPUT_RANGE		0x19
#define AD3552R_REG_ADDR_CH_OFFSET(ch)			(0x1B + (ch) * 2)
#define AD3552R_REG_ADDR_CH_GAIN(ch)			(0x1C + (ch) * 2)
/* Secondary region */
#define AD3552R_SECONDARY_REGION_START			0x28
#define AD3552R_REG_ADDR_HW_LDAC_16B			0x28
#define AD3552R_REG_ADDR_CH_DAC_16B(ch)			(0x29 + (ch) * 2)
#define AD3552R_REG_ADDR_DAC_PAGE_MASK_16B		0x2D
#define AD3552R_REG_ADDR_CH_SELECT_16B			0x2F
#define AD3552R_REG_ADDR_INPUT_PAGE_MASK_16B		0x30
#define AD3552R_REG_ADDR_SW_LDAC_16B			0x32
#define AD3552R_REG_ADDR_CH_INPUT_16B(ch)		(0x33 + (ch) * 2)
/* 3 bytes registers */
#define AD3552R_REG_START_24B				0x37
#define AD3552R_REG_ADDR_HW_LDAC_24B			0x37
#define AD3552R_REG_ADDR_CH_DAC_24B(ch)			(0x38 + (ch) * 3)
#define AD3552R_REG_ADDR_DAC_PAGE_MASK_24B		0x3E
#define AD3552R_REG_ADDR_CH_SELECT_24B			0x41
#define AD3552R_REG_ADDR_INPUT_PAGE_MASK_24B		0x42
#define AD3552R_REG_ADDR_SW_LDAC_24B			0x45
#define AD3552R_REG_ADDR_CH_INPUT_24B(ch)		(0x46 + (ch) * 3)

#define AD3552R_REG_ADDR_MAX				0x4B

/* AD3552R_REG_ADDR_INTERFACE_CONFIG_A */
#define AD3552R_MASK_SOFTWARE_RESET			(BIT(7) | BIT(0))
#define AD3552R_MASK_ADDR_ASCENSION			BIT(5)
#define AD3552R_MASK_SDO_ACTIVE				BIT(4)
/* AD3552R_REG_ADDR_INTERFACE_CONFIG_B */
#define AD3552R_MASK_SINGLE_INST			BIT(7)
#define AD3552R_MASK_SHORT_INSTRUCTION			BIT(3)
/* AD3552R_REG_ADDR_DEVICE_CONFIG */
#define AD3552R_MASK_DEVICE_STATUS(n)			BIT(4 + (n))
#define AD3552R_MASK_CUSTOM_MODES			(BIT(3) | BIT(2))
#define AD3552R_MASK_OPERATING_MODES			(BIT(1) | BIT(0))
/* AD3552R_REG_ADDR_CHIP_TYPE */
#define AD3552R_MASK_CLASS				0x0F
/* AD3552R_REG_ADDR_CHIP_GRADE */
#define AD3552R_MASK_GRADE				0xF0
#define AD3552R_MASK_DEVICE_REVISION			0x0F
/* AD3552R_REG_ADDR_STREAM_MODE */
#define AD3552R_MASK_LENGTH				0x0F
/* AD3552R_REG_ADDR_TRANSFER_REGISTER */
#define AD3552R_MASK_MULTI_IO_MODE			(BIT(7) | BIT(6))
#define AD3552R_MASK_STREAM_LENGTH_KEEP_VALUE		BIT(2)
/* AD3552R_REG_ADDR_INTERFACE_CONFIG_C */
#define AD3552R_MASK_CRC_ENABLE				(BIT(7) | BIT(6) |\
							 BIT(1) | BIT(0))
#define AD3552R_MASK_STRICT_REGISTER_ACCESS		BIT(5)
/* AD3552R_REG_ADDR_INTERFACE_STATUS_A */
#define AD3552R_MASK_INTERFACE_NOT_READY		BIT(7)
#define AD3552R_MASK_CLOCK_COUNTING_ERROR		BIT(5)
#define AD3552R_MASK_INVALID_OR_NO_CRC			BIT(3)
#define AD3552R_MASK_WRITE_TO_READ_ONLY_REGISTER	BIT(2)
#define AD3552R_MASK_PARTIAL_REGISTER_ACCESS		BIT(1)
#define AD3552R_MASK_REGISTER_ADDRESS_INVALID		BIT(0)
/* AD3552R_REG_ADDR_INTERFACE_CONFIG_D */
#define AD3552R_MASK_ALERT_ENABLE_PULLUP		BIT(6)
#define AD3552R_MASK_MEM_CRC_EN				BIT(4)
#define AD3552R_MASK_SDO_DRIVE_STRENGTH			(BIT(3) | BIT(2))
#define AD3552R_MASK_DUAL_SPI_SYNCHROUNOUS_EN		BIT(1)
#define AD3552R_MASK_SPI_CONFIG_DDR			BIT(0)
/* AD3552R_REG_ADDR_SH_REFERENCE_CONFIG */
#define AD3552R_MASK_IDUMP_FAST_MODE			BIT(6)
#define AD3552R_MASK_SAMPLE_HOLD_DIFFERENTIAL_USER_EN	BIT(5)
#define AD3552R_MASK_SAMPLE_HOLD_USER_TRIM		(BIT(4) | BIT(3))
#define AD3552R_MASK_SAMPLE_HOLD_USER_ENABLE		BIT(2)
#define AD3552R_MASK_REFERENCE_VOLTAGE_SEL		(BIT(1) | BIT(0))
/* AD3552R_REG_ADDR_ERR_ALARM_MASK */
#define AD3552R_MASK_REF_RANGE_ALARM			BIT(6)
#define AD3552R_MASK_CLOCK_COUNT_ERR_ALARM		BIT(5)
#define AD3552R_MASK_MEM_CRC_ERR_ALARM			BIT(4)
#define AD3552R_MASK_SPI_CRC_ERR_ALARM			BIT(3)
#define AD3552R_MASK_WRITE_TO_READ_ONLY_ALARM		BIT(2)
#define AD3552R_MASK_PARTIAL_REGISTER_ACCESS_ALARM	BIT(1)
#define AD3552R_MASK_REGISTER_ADDRESS_INVALID_ALARM	BIT(0)
/* AD3552R_REG_ADDR_ERR_STATUS */
#define AD3552R_MASK_REF_RANGE_ERR_STATUS			BIT(6)
#define AD3552R_MASK_DUAL_SPI_STREAM_EXCEEDS_DAC_ERR_STATUS	BIT(5)
#define AD3552R_MASK_MEM_CRC_ERR_STATUS				BIT(4)
#define AD3552R_MASK_RESET_STATUS				BIT(0)
/* AD3552R_REG_ADDR_POWERDOWN_CONFIG */
#define AD3552R_MASK_CH_DAC_POWERDOWN(ch)		BIT(4 + (ch))
#define AD3552R_MASK_CH_AMPLIFIER_POWERDOWN(ch)		BIT(ch)
/* AD3552R_REG_ADDR_CH0_CH1_OUTPUT_RANGE */
#define AD3552R_MASK_CH_OUTPUT_RANGE_SEL(ch)		((ch) ? 0xF0 : 0x0F)
/* AD3552R_REG_ADDR_CH_GAIN */
#define AD3552R_MASK_CH_RANGE_OVERRIDE			BIT(7)
#define AD3552R_MASK_CH_GAIN_SCALING_N			(BIT(6) | BIT(5))
#define AD3552R_MASK_CH_GAIN_SCALING_P			(BIT(4) | BIT(3))
#define AD3552R_MASK_CH_OFFSET_POLARITY			BIT(2)
#define AD3552R_MASK_CH_OFFSET_H			BIT(0)
/* AD3552R_REG_ADDR_CH_OFFSET */
#define AD3552R_MASK_CH_OFFSET_L			0xFF
/* Mask valid for multiple registers */
#define AD3552R_MASK_DAC_DATA24				0xFFFFC0
#define AD3552R_MASK_CH(ch)				BIT(ch)
#define AD3552R_MASK_ALL_CH				(BIT(0) | BIT(1))
#define AD3552R_REAL_BITS_PREC_MODE			16
#define AD3552R_STORAGE_BITS_PREC_MODE			24
#define AD3552R_REAL_BITS_FAST_MODE			12
#define AD3552R_STORAGE_BITS_FAST_MODE			16

#define AD3552R_ATTR_REG(attr) addr_mask_map[attr][0]
#define AD3552R_ATTR_MASK(attr) addr_mask_map[attr][1]
#define AD3552R_CH_ATTR_REG(attr) addr_mask_map_ch[attr][0]
#define AD3552R_CH_ATTR_MASK(ch, attr) addr_mask_map_ch[attr][(ch) + 1]

#define AD3552R_MAX_REG_SIZE				3
#define AD3552R_READ_BIT				(1 << 7)
#define AD3552R_ADDR_MASK				(~AD3552R_READ_BIT)
#define AD3552R_CRC_ENABLE_VALUE			(BIT(6) | BIT(1))
#define AD3552R_CRC_DISABLE_VALUE			(BIT(1) | BIT(0))
#define AD3552R_EXTERNAL_VREF_MASK			BIT(1)
#define AD3552R_CRC_POLY				0x07
#define AD3552R_CRC_SEED				0xA5
#define AD3552R_MASK_DAC_12B				0xFFF0
#define AD3552R_MASK_DAC_16B				0xFFFF
#define AD3552R_SECONDARY_REGION_ADDR			0x28
#define AD3552R_DEFAULT_CONFIG_B_VALUE			0x8
#define AD3552R_DATA_IDX(x)				(1 + (x))
#define AD3552R_DEFAULT_DAC_UPDATE_PERIOD		1000

#define AD3552R_PREC_CH_START				3
#define _TO_CH_DAC(ch)	(((ch) < AD3552R_PREC_CH_START) ? (ch) : \
			 (ch) - AD3552R_PREC_CH_START)

enum ad3552r_ch_output_range {
	/* Range from 0 V to 2.5 V. Requires Rfb1x connection */
	AD3552R_CH_OUTPUT_RANGE_0__2_5V,
	/* Range from 0 V to 5 V. Requires Rfb1x connection  */
	AD3552R_CH_OUTPUT_RANGE_0__5V,
	/* Range from 0 V to 10 V. Requires Rfb2x connection  */
	AD3552R_CH_OUTPUT_RANGE_0__10V,
	/* Range from -2.5 V to 7.5 V. Requires Rfb2x connection  */
	AD3552R_CH_OUTPUT_RANGE_NEG_5__5V,
	/* Range from -6.5 V to 3.5 V. Requires Rfb4x connection  */
	AD3552R_CH_OUTPUT_RANGE_NEG_10__10V,
};

enum ad3552r_ch_gain_scaling {
	/* Gain scaling of 1 */
	AD3552R_CH_GAIN_SCALING_1,
	/* Gain scaling of 0.5 */
	AD3552R_CH_GAIN_SCALING_0_5,
	/* Gain scaling of 0.25 */
	AD3552R_CH_GAIN_SCALING_0_25,
	/* Gain scaling of 0.125 */
	AD3552R_CH_GAIN_SCALING_0_125,
};

enum ad3552r_offset_polarity {
	/* Positive offset */
	AD3552R_OFFSET_POLARITY_POSITIVE,
	/* Negative offset */
	AD3552R_OFFSET_POLARITY_NEGATIVE,
};

enum ad3552r_dev_attributes {
	/* - Direct register values */
	/* From 0-3 */
	AD3552R_SDO_DRIVE_STRENGTH,
	/*
	 * 0 -> Internal Vref, vref_io pin floating (default)
	 * 1 -> Internal Vref, vref_io driven by internal vref
	 * 2 or 3 -> External Vref
	 */
	AD3552R_VREF_SELECT,
	/* Enable / Disable CRC */
	AD3552R_CRC_ENABLE,
	/* Spi mode: Strandard, Dual or Quad */
	AD3552R_SPI_MULTI_IO_MODE,
	/* Spi data rate: Single or dual */
	AD3552R_SPI_DATA_RATE,
	/* Dual spi synchronous mode */
	AD3552R_SPI_SYNCHRONOUS_ENABLE,

	/* - Direct register values (Private) */
	/* Read registers in ascending order if set. Else descending */
	AD3552R_ADDR_ASCENSION,
	/* Single instruction mode if set. Else, stream mode */
	AD3552R_SINGLE_INST,
	/* Number of addresses to loop on when stream writing. */
	AD3552R_STREAM_MODE,
	/* Keep stream value if set. */
	AD3552R_STREAM_LENGTH_KEEP_VALUE,
};

enum ad3552r_ch_attributes {
	/* DAC powerdown */
	AD3552R_CH_DAC_POWERDOWN,
	/* DAC amplifier powerdown */
	AD3552R_CH_AMPLIFIER_POWERDOWN,
	/* Select the output range. Select from enum ad3552r_ch_output_range */
	AD3552R_CH_OUTPUT_RANGE_SEL,
	/*
	 * Over-rider the range selector in order to manually set the output
	 * voltage range
	 */
	AD3552R_CH_RANGE_OVERRIDE,
	/* Manually set the offset voltage */
	AD3552R_CH_GAIN_OFFSET,
	/* Sets the polarity of the offset. */
	AD3552R_CH_GAIN_OFFSET_POLARITY,
	/* PDAC gain scaling */
	AD3552R_CH_GAIN_SCALING_P,
	/* NDAC gain scaling */
	AD3552R_CH_GAIN_SCALING_N,
	/* Trigger a software LDAC */
	AD3552R_CH_TRIGGER_SOFTWARE_LDAC,
	/* Hardware LDAC Mask */
	AD3552R_CH_HW_LDAC_MASK,
	/* Rfb value */
	AD3552R_CH_RFB,
	/* Code value when voltage is minimum. Used to calculate voltage */
	AD3552R_CH_SELECT,
	/* Raw value to be set to dac */
	AD3552R_CH_CODE
};

struct ad3552_transfer_data {
	/* Starting address for transfer */
	u8	addr;
	/* Data to transfer */
	u8	*data;
	/* Size of data to transfer */
	u32	len;
	/* Read transaction if true, write transfer otherwise */
	u8	is_read : 1;
};

struct ad3552r_ch_data {
	u16	gain_offset : 9;
	u16	range_override : 1;
	u16	n : 2;
	u16	p : 2;
	u16	offset_polarity : 1;
	u16	rfb;
	u8	range;
	s32	scale_int;
	s32	scale_dec;
	s32	offset_int;
	s32	offset_dec;
};

struct ad3552r_desc {
	struct iio_dev			*indio_dev;
	struct mutex			lock;
	struct gpio_desc		*gpio_reset;
	struct spi_device		*spi;
	struct ad3552r_ch_data		ch_data[AD3552R_NUM_CH];
	u32	mask;
	u32	bytes_per_datum;
	u8	crc_table[CRC8_TABLE_SIZE];
	u8	crc_en : 1;
	unsigned int num_ch;
};

static const u16 addr_mask_map[][2] = {
	[AD3552R_ADDR_ASCENSION] = {
			AD3552R_REG_ADDR_INTERFACE_CONFIG_A,
			AD3552R_MASK_ADDR_ASCENSION
	},
	[AD3552R_SINGLE_INST] = {
			AD3552R_REG_ADDR_INTERFACE_CONFIG_B,
			AD3552R_MASK_SINGLE_INST
	},
	[AD3552R_STREAM_MODE] = {
			AD3552R_REG_ADDR_STREAM_MODE,
			AD3552R_MASK_LENGTH
	},
	[AD3552R_STREAM_LENGTH_KEEP_VALUE] = {
			AD3552R_REG_ADDR_TRANSFER_REGISTER,
			AD3552R_MASK_STREAM_LENGTH_KEEP_VALUE
	},
	[AD3552R_SDO_DRIVE_STRENGTH] = {
			AD3552R_REG_ADDR_INTERFACE_CONFIG_D,
			AD3552R_MASK_SDO_DRIVE_STRENGTH
	},
	[AD3552R_VREF_SELECT] = {
			AD3552R_REG_ADDR_SH_REFERENCE_CONFIG,
			AD3552R_MASK_REFERENCE_VOLTAGE_SEL
	},
	[AD3552R_CRC_ENABLE] = {
			AD3552R_REG_ADDR_INTERFACE_CONFIG_C,
			AD3552R_MASK_CRC_ENABLE
	},
	[AD3552R_SPI_MULTI_IO_MODE] = {
			AD3552R_REG_ADDR_TRANSFER_REGISTER,
			AD3552R_MASK_MULTI_IO_MODE
	},
	[AD3552R_SPI_DATA_RATE] = {
			AD3552R_REG_ADDR_INTERFACE_CONFIG_D,
			AD3552R_MASK_SPI_CONFIG_DDR
	},
	[AD3552R_SPI_SYNCHRONOUS_ENABLE] = {
			AD3552R_REG_ADDR_INTERFACE_CONFIG_D,
			AD3552R_MASK_DUAL_SPI_SYNCHROUNOUS_EN
	},
};

/* 0 -> reg addr, 1->ch0 mask, 2->ch1 mask */
static const u16 addr_mask_map_ch[][3] = {
	[AD3552R_CH_DAC_POWERDOWN] = {
			AD3552R_REG_ADDR_POWERDOWN_CONFIG,
			AD3552R_MASK_CH_DAC_POWERDOWN(0),
			AD3552R_MASK_CH_DAC_POWERDOWN(1)
	},
	[AD3552R_CH_AMPLIFIER_POWERDOWN] = {
			AD3552R_REG_ADDR_POWERDOWN_CONFIG,
			AD3552R_MASK_CH_AMPLIFIER_POWERDOWN(0),
			AD3552R_MASK_CH_AMPLIFIER_POWERDOWN(1)
	},
	[AD3552R_CH_OUTPUT_RANGE_SEL] = {
			AD3552R_REG_ADDR_CH0_CH1_OUTPUT_RANGE,
			AD3552R_MASK_CH_OUTPUT_RANGE_SEL(0),
			AD3552R_MASK_CH_OUTPUT_RANGE_SEL(1)
	},
	/*
	 * This attributes are update by the chip on 16B and 24B no matter to
	 * what register the write is done
	 */
	[AD3552R_CH_TRIGGER_SOFTWARE_LDAC] = {
			AD3552R_REG_ADDR_SW_LDAC_16B,
			AD3552R_MASK_CH(0),
			AD3552R_MASK_CH(1)
	},
	[AD3552R_CH_HW_LDAC_MASK] = {
			AD3552R_REG_ADDR_HW_LDAC_16B,
			AD3552R_MASK_CH(0),
			AD3552R_MASK_CH(1)
	},
	[AD3552R_CH_SELECT] = {
			AD3552R_REG_ADDR_CH_SELECT_16B,
			AD3552R_MASK_CH(0),
			AD3552R_MASK_CH(1)
	}
};

static const s32 ch_ranges[][2] = {
	[AD3552R_CH_OUTPUT_RANGE_0__2_5V]	= {0, 2500},
	[AD3552R_CH_OUTPUT_RANGE_0__5V]		= {0, 5000},
	[AD3552R_CH_OUTPUT_RANGE_0__10V]	= {0, 10000},
	[AD3552R_CH_OUTPUT_RANGE_NEG_5__5V]	= {-5000, 5000},
	[AD3552R_CH_OUTPUT_RANGE_NEG_10__10V]	= {-10000, 10000}
};

static const u32 defaut_rfbs[] = {
	[AD3552R_CH_OUTPUT_RANGE_0__2_5V]	= 1,
	[AD3552R_CH_OUTPUT_RANGE_0__5V]		= 1,
	[AD3552R_CH_OUTPUT_RANGE_0__10V]	= 2,
	[AD3552R_CH_OUTPUT_RANGE_NEG_5__5V]	= 2,
	[AD3552R_CH_OUTPUT_RANGE_NEG_10__10V]	= 4
};

static const float gains_scaling_table[] = {
	[AD3552R_CH_GAIN_SCALING_1]		= 1,
	[AD3552R_CH_GAIN_SCALING_0_5]		= 0.5,
	[AD3552R_CH_GAIN_SCALING_0_25]		= 0.25,
	[AD3552R_CH_GAIN_SCALING_0_125]		= 0.125
};

static u8 _ad3552r_reg_len(u8 addr)
{
	if (addr > AD3552R_REG_ADDR_MAX)
		return 0;

	switch (addr) {
	case AD3552R_REG_ADDR_HW_LDAC_16B:
	case AD3552R_REG_ADDR_CH_SELECT_16B:
	case AD3552R_REG_ADDR_SW_LDAC_16B:
	case AD3552R_REG_ADDR_HW_LDAC_24B:
	case AD3552R_REG_ADDR_CH_SELECT_24B:
	case AD3552R_REG_ADDR_SW_LDAC_24B:
		return 1;
	default:
		break;
	}

	if (addr > AD3552R_REG_ADDR_HW_LDAC_24B)
		return 3;
	if (addr > AD3552R_REG_ADDR_HW_LDAC_16B)
		return 2;

	return 1;
}

/* Transfer data using CRC */
static int _ad3552r_transfer_with_crc(struct ad3552r_desc *desc,
				   struct ad3552_transfer_data *data,
				   u8 instr)
{
	struct spi_transfer	msg = { 0 };
	int	ret;
	u8	out[AD3552R_MAX_REG_SIZE + 2];
	u8	in[AD3552R_MAX_REG_SIZE + 2];
	u8	*pbuf;
	u8	addr;
	u8	reg_len;
	u8	crc_init;
	s32	inc;
	u32	i;

	msg.rx_buf = in;
	msg.tx_buf = out;
	inc = 0;
	i = 0;
	mutex_lock(&desc->lock);
	do {
		/* Get next address to for which CRC value will be calculated */
		addr = data->addr + inc;
		addr %= AD3552R_REG_ADDR_MAX;
		reg_len = _ad3552r_reg_len(addr);

		/* Prepare CRC to send */
		msg.len = reg_len + 1;
		if (i > 0)
			crc_init = addr;
		else
			crc_init = crc8(desc->crc_table, &instr, 1,
					AD3552R_CRC_SEED);

		if (data->is_read && i > 0) {
			/* CRC is not needed for continuous read transaction */
			memset(out, 0xFF, reg_len + 1);
		} else {
			pbuf = out;
			if (i == 0) {
				/* Take in consideration instruction for CRC */
				pbuf[0] = instr;
				++pbuf;
				++msg.len;
			}
			memcpy(pbuf, data->data + i, reg_len);
			pbuf[reg_len] = crc8(desc->crc_table, pbuf, reg_len,
					     crc_init);
		}

		/* Send message */
		msg.cs_change = !(i + reg_len == data->len);
		ret = spi_sync_transfer(desc->spi, &msg, 1);
		if (ret < 0)
			return ret;

		/* Check received CRC */
		if (data->is_read) {
			pbuf = in;
			if (i == 0)
				pbuf++;
			/* Save received data */
			memcpy(data->data + i, pbuf, reg_len);
			if (pbuf[reg_len] !=
			    crc8(desc->crc_table, pbuf, reg_len, crc_init))
				return -EBADMSG;
		} else {
			if (in[reg_len + (i == 0)] != out[reg_len + (i == 0)])
				return -EBADMSG;
		}
		inc -= reg_len;
		i += reg_len;
	} while (i < data->len);
	mutex_unlock(&desc->lock);

	return 0;
}

/* SPI transfer to device */
static int ad3552r_transfer(struct ad3552r_desc *desc,
			     struct ad3552_transfer_data *data)
{
	struct spi_transfer	msgs[2] = { 0 };
	u8			instr;

	instr = data->addr & AD3552R_ADDR_MASK;
	instr |= data->is_read ? AD3552R_READ_BIT : 0;

	if (desc->crc_en)
		return _ad3552r_transfer_with_crc(desc, data, instr);

	msgs[0].tx_buf = &instr;
	msgs[0].len = 1;
	msgs[1].len = data->len;
	if (data->is_read)
		msgs[1].rx_buf = data->data;
	else
		msgs[1].tx_buf = data->data;

	return spi_sync_transfer(desc->spi, msgs, ARRAY_SIZE(msgs));
}

static int _ad3552r_access_reg(struct ad3552r_desc *desc, u8 addr,
				   u16 in, u16 *out)
{
	struct ad3552_transfer_data	msg = { 0 };
	int err;
	u32 tmp;
	u8  buf[3] = {0};
	u8  reg_len;

	reg_len = _ad3552r_reg_len(addr);
	if (!reg_len)
		return -EINVAL;

	if (out) {
		tmp = 0;
		msg.is_read = 1;
	} else {
		msg.is_read = 0;
		if (reg_len == 2)
			in &= AD3552R_MASK_DAC_12B;
		if (reg_len == 1)
			buf[0] = in & 0xFF;
		else
			__put_unaligned_be16(in, buf);
	}

	msg.addr = addr + reg_len - 1;
	msg.data = buf;
	msg.len = reg_len;
	err = ad3552r_transfer(desc, &msg);
	if (err)
		return err;

	if (out) {
		if (reg_len == 1)
			*out = buf[0];
		else
			*out = (buf[0] << 8) | buf[1];
	}

	return 0;
}

static int ad3552r_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct ad3552r_desc	*desc = iio_priv(indio_dev);
	int	err;
	u16	val;

	if (readval) {
		err = _ad3552r_access_reg(desc, reg, writeval, &val);
		*readval = val;
		return err;
	} else {
		return _ad3552r_access_reg(desc, reg, writeval, NULL);
	}

}

static int ad3552r_write_reg(struct ad3552r_desc *desc, u8 addr,
				 u16 val)
{
	return _ad3552r_access_reg(desc, addr, val, NULL);
}

static int ad3552r_read_reg(struct ad3552r_desc *desc, u8 addr,
				u16 *val)
{
	return _ad3552r_access_reg(desc, addr, 0, val);
}

static inline int _ad3552r_write_reg_mask(struct ad3552r_desc *desc,
						u8 addr, u16 mask,
						u16 val)
{
	int ret;
	u16 reg;
	u16 reg_full_mask;

	reg_full_mask = GENMASK(8 * _ad3552r_reg_len(addr), 0);
	if (mask == reg_full_mask) {
		reg = val;
	} else {
		ret = ad3552r_read_reg(desc, addr, &reg);
		if (ret < 0)
			return ret;

		reg = (reg & ~mask) | (val << __ffs(mask));
	}

	return ad3552r_write_reg(desc, addr, reg);
}

static inline int _ad3552r_read_reg_mask(struct ad3552r_desc *desc,
					u8 addr, u16 mask,
					u16 *val)
{
	int ret;

	ret = ad3552r_read_reg(desc, addr, val);
	*val = (*val & mask) >> __ffs(mask);

	return ret;
}

static inline int _ad3552r_get_reg_attr(struct ad3552r_desc *desc,
					    u16 attr,
					    u16 *val)
{
	return _ad3552r_read_reg_mask(desc, AD3552R_ATTR_REG(attr),
			AD3552R_ATTR_MASK(attr), val);
}

static inline int _ad3552r_set_reg_attr(struct ad3552r_desc *desc,
						u16 attr,
						u16 val)
{
	return _ad3552r_write_reg_mask(desc, AD3552R_ATTR_REG(attr),
			AD3552R_ATTR_MASK(attr), val);
}

static int _ad3552r_set_crc_enable(struct ad3552r_desc *desc, u16 en)
{
	int ret;
	u16 reg;

	reg = en ? AD3552R_CRC_ENABLE_VALUE : AD3552R_CRC_DISABLE_VALUE;
	ret = ad3552r_write_reg(desc, AD3552R_ATTR_REG(AD3552R_CRC_ENABLE),
				reg);
	if (ret < 0)
		return ret;

	desc->crc_en = en;

	return 0;
}

s32 ad3552r_set_dev_value(struct ad3552r_desc *desc,
			  enum ad3552r_dev_attributes attr,
			  u16 val)
{
	switch (attr) {
	case AD3552R_SPI_MULTI_IO_MODE:
	case AD3552R_SPI_DATA_RATE:
	case AD3552R_SPI_SYNCHRONOUS_ENABLE:
		/* TODO. Only standard spi implemented for the moment */
		return -EINVAL;
	case AD3552R_CRC_ENABLE:
		return _ad3552r_set_crc_enable(desc, val);
	default:
		return _ad3552r_set_reg_attr(desc, attr, val);
	}

	return 0;
}

static int _ad3552r_set_gain_value(struct ad3552r_desc *desc,
					enum ad3552r_ch_attributes attr,
					u8 ch,
					int val)
{
	struct ad3552_transfer_data	msg = {0};
	int reg_mask;
	int ret;
	u16 regs;

	msg.addr = AD3552R_REG_ADDR_CH_GAIN(ch);
	msg.data = (u8 *)&regs;
	if (attr == AD3552R_CH_GAIN_OFFSET)
		msg.len = 2;
	else
		msg.len = 1;

	msg.is_read = true;
	ret = ad3552r_transfer(desc, &msg);
	if (ret < 0)
		return ret;

	switch (attr) {
	case AD3552R_CH_GAIN_OFFSET:
		reg_mask = (AD3552R_MASK_CH_OFFSET_L << 8) &
			    AD3552R_MASK_CH_OFFSET_H;
		desc->ch_data[ch].gain_offset = val;
		val = ((val >> 8) & 0xFF) | ((val & 0xFF) << 8);
		break;
	case AD3552R_CH_RANGE_OVERRIDE:
		desc->ch_data[ch].range_override = !!val;
		reg_mask = AD3552R_MASK_CH_RANGE_OVERRIDE;
		break;
	case AD3552R_CH_GAIN_OFFSET_POLARITY:
		desc->ch_data[ch].offset_polarity = !!val;
		reg_mask = AD3552R_MASK_CH_OFFSET_POLARITY;
		break;
	case AD3552R_CH_GAIN_SCALING_P:
		desc->ch_data[ch].p = val;
		reg_mask = AD3552R_MASK_CH_GAIN_SCALING_P;
		break;
	case AD3552R_CH_GAIN_SCALING_N:
		desc->ch_data[ch].n = val;
		reg_mask = AD3552R_MASK_CH_GAIN_SCALING_N;
		break;
	default:
		return -EINVAL;
	}
	regs = (regs & ~reg_mask) | (val << __ffs(reg_mask));

	msg.is_read = false;
	return ad3552r_transfer(desc, &msg);
}

static int _ad3552r_get_gain_value(struct ad3552r_desc *desc,
				       enum ad3552r_ch_attributes attr,
				       u8 ch,
				       u16 *val)
{
	struct ad3552_transfer_data	msg = {0};
	u16 reg_mask;
	u16 regs;
	int ret;

	msg.addr = AD3552R_REG_ADDR_CH_GAIN(ch);
	msg.data = (u8 *)&regs;
	if (attr == AD3552R_CH_GAIN_OFFSET)
		msg.len = 2;
	else
		msg.len = 1;

	msg.is_read = 1;
	ret = ad3552r_transfer(desc, &msg);
	if (ret < 0)
		return ret;

	switch (attr) {
	case AD3552R_CH_GAIN_OFFSET:
		reg_mask =  (AD3552R_MASK_CH_OFFSET_L << 8) &
			    AD3552R_MASK_CH_OFFSET_H;
		break;
	case AD3552R_CH_RANGE_OVERRIDE:
		reg_mask = AD3552R_MASK_CH_RANGE_OVERRIDE;
		break;
	case AD3552R_CH_GAIN_OFFSET_POLARITY:
		reg_mask = AD3552R_MASK_CH_OFFSET_POLARITY;
		break;
	case AD3552R_CH_GAIN_SCALING_P:
		reg_mask = AD3552R_MASK_CH_GAIN_SCALING_P;
		break;
	case AD3552R_CH_GAIN_SCALING_N:
		reg_mask = AD3552R_MASK_CH_GAIN_SCALING_N;
		break;
	default:
		return -EINVAL;
	}

	*val = (regs & reg_mask) >> __ffs(reg_mask);
	if (attr == AD3552R_CH_GAIN_OFFSET)
		*val = ((*val >> 8) & 0xFF) | ((*val & 0xFF) << 8);

	return 0;
}

static u8 _ch_to_addr(u8 ch)
{
	/* Based on ad3552r_channels order */
	switch (ch) {
	case 0:
	case 1:
		return AD3552R_REG_ADDR_CH_DAC_16B(ch);
	case 2:
		return AD3552R_REG_ADDR_DAC_PAGE_MASK_16B;
	case 3:
	case 4:
		return AD3552R_REG_ADDR_CH_DAC_24B(ch - AD3552R_PREC_CH_START);
	case 5:
		return AD3552R_REG_ADDR_DAC_PAGE_MASK_24B;
	default:
		return -EINVAL;
	}
}

/* Iterate over mask and write required bytes */
static int _ad3552r_write_codes(struct ad3552r_desc *desc, u32 mask,
				    u8 *vals)
{
	struct ad3552_transfer_data	msg = {0};
	int				err;
	s32				cnt;
	s32				i;
	s32				val_idx;
	u8				all_dac;
	u8				all_dac_prec;

	all_dac = mask == AD3552R_MASK_ALL_CH;
	all_dac_prec = (mask >> AD3552R_PREC_CH_START) == AD3552R_MASK_ALL_CH;
	if (all_dac || all_dac_prec) {
		if (all_dac)
			msg.addr = AD3552R_REG_ADDR_CH_DAC_16B(1) + 1;
		else
			msg.addr = AD3552R_REG_ADDR_CH_DAC_24B(1) + 2;
		msg.len = _ad3552r_reg_len(msg.addr) * 2;
		msg.data = (u8 *)vals;

		return ad3552r_transfer(desc, &msg);
	}


	/* Mask for nonconsecutive channels. Write them individually. */
	cnt = AD3552R_NUM_CH;
	val_idx = 0;
	for (i = 0; cnt && i < 6; i++) {
		if (mask & BIT(i)) {
			err = ad3552r_write_reg(desc, _ch_to_addr(i),
						vals[val_idx]);
			if (err)
				return err;
			if (i > AD3552R_PREC_CH_START)
				val_idx += 3;
			else
				val_idx += 2;
			cnt--;
		}
	}

	return 0;
}

static int ad3552r_get_ch_value(struct ad3552r_desc *desc,
				enum ad3552r_ch_attributes attr,
				u8 ch,
				u16 *val)
{
	int ret;
	u16 reg;
	u8  addr;
	u16 mask;

	/* Attributes not defined in addr_mask_map_ch */
	switch (attr) {
	case AD3552R_CH_CODE:
		return ad3552r_read_reg(desc, _ch_to_addr(ch), val);
	case AD3552R_CH_RFB:
		*val = desc->ch_data[_TO_CH_DAC(ch)].rfb;
		return 0;
	default:
		break;
	}

	if (attr >= AD3552R_CH_RANGE_OVERRIDE &&
	    attr <= AD3552R_CH_GAIN_SCALING_N)
		return _ad3552r_get_gain_value(desc, attr, _TO_CH_DAC(ch), val);

	addr = AD3552R_CH_ATTR_REG(attr);
	if (addr == AD3552R_REG_ADDR_SW_LDAC_24B ||
	    addr == AD3552R_REG_ADDR_SW_LDAC_16B) {
		dev_dbg(&desc->indio_dev->dev, "Write only registers\n");
		/* LDAC are write only registers */
		return -EINVAL;
	}

	ret = ad3552r_read_reg(desc, addr, &reg);
	if (ret < 0)
		return ret;

	mask = AD3552R_CH_ATTR_MASK(_TO_CH_DAC(ch), attr);
	*val = (reg & mask) >> __ffs(mask);

	return 0;
}

static int ad3552r_set_ch_value(struct ad3552r_desc *desc,
			     enum ad3552r_ch_attributes attr,
			     u8 ch,
			     u16 val)
{
	int ret;

	/* Attributes not defined in addr_mask_map_ch */
	switch (attr) {
	case AD3552R_CH_CODE:
		return ad3552r_write_reg(desc, _ch_to_addr(ch), val);
	case AD3552R_CH_RFB:
		desc->ch_data[_TO_CH_DAC(ch)].rfb = val;
		return 0;
	default:
		break;
	}
	if (attr >= AD3552R_CH_RANGE_OVERRIDE &&
	    attr <= AD3552R_CH_GAIN_SCALING_N)
		return _ad3552r_set_gain_value(desc, attr, _TO_CH_DAC(ch), val);

	/* Update register related to attributes in chip */
	ret = _ad3552r_write_reg_mask(desc,
				      AD3552R_CH_ATTR_REG(attr),
				      AD3552R_CH_ATTR_MASK(_TO_CH_DAC(ch),
							   attr),
				      val);
	if (ret < 0)
		return ret;

	/* Update software structures */
	if (attr == AD3552R_CH_OUTPUT_RANGE_SEL) {
		val %= AD3552R_CH_OUTPUT_RANGE_NEG_10__10V + 1;
		desc->ch_data[_TO_CH_DAC(ch)].range = val;
		desc->ch_data[_TO_CH_DAC(ch)].rfb = defaut_rfbs[val];
	}

	return ret;
}

static const char * const ad3552r_ch_output_range[] = {
	[AD3552R_CH_OUTPUT_RANGE_0__2_5V]	= "0_+2.5V",
	[AD3552R_CH_OUTPUT_RANGE_0__5V]		= "0_+5V",
	[AD3552R_CH_OUTPUT_RANGE_0__10V]	= "0_+10V",
	[AD3552R_CH_OUTPUT_RANGE_NEG_5__5V]	= "-5_+5V",
	[AD3552R_CH_OUTPUT_RANGE_NEG_10__10V]	= "-10_+10V"
};

static const char * const ad3552r_ch_gain_scaling_p[] = {
	[AD3552R_CH_GAIN_SCALING_1]	= "1",
	[AD3552R_CH_GAIN_SCALING_0_5]	= "0.5",
	[AD3552R_CH_GAIN_SCALING_0_25]	= "0.25",
	[AD3552R_CH_GAIN_SCALING_0_125]	= "0.125"
};

static const char * const ad3552r_ch_gain_scaling_n[] = {
	[AD3552R_CH_GAIN_SCALING_1]	= "1",
	[AD3552R_CH_GAIN_SCALING_0_5]	= "0.5",
	[AD3552R_CH_GAIN_SCALING_0_25]	= "0.25",
	[AD3552R_CH_GAIN_SCALING_0_125]	= "0.125"
};

#define AD3552R_CH_DAC(_idx) {		\
	.type = IIO_VOLTAGE,		\
	.output = true,			\
	.indexed = true,		\
	.channel = _idx,		\
	.scan_index = _idx,		\
	.scan_type = {			\
		.sign = 'u',		\
		.realbits = 16,		\
		.storagebits = 16,	\
		.endianness = IIO_LE,	\
	},				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_ENABLE) |	\
				BIT(IIO_CHAN_INFO_OFFSET),	\
}

#define AD3552R_CH_DAC_PAGE(_idx) {	\
	.type = IIO_VOLTAGE,		\
	.output = true,			\
	.indexed = true,		\
	.channel = _idx,		\
	.scan_index = _idx,	\
	.scan_type = {			\
		.sign = 'u',		\
		.realbits = 16,		\
		.storagebits = 16,	\
		.endianness = IIO_LE,	\
	},				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.modified = 1,						\
	.channel2 = IIO_MOD_X_AND_Z,	\
}

static const struct iio_chan_spec ad3552r_channels[] = {
	AD3552R_CH_DAC(0),
	AD3552R_CH_DAC(1),
	AD3552R_CH_DAC_PAGE(2)
};

static int ad3552r_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val,
			      int *val2,
			      long mask)
{
	struct ad3552r_desc	*dac = iio_priv(indio_dev);
	u16		tmp_val;
	int		err;
	u8		ch;

	ch = _TO_CH_DAC(chan->channel);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		err = ad3552r_get_ch_value(dac, AD3552R_CH_CODE, chan->channel,
					   &tmp_val);
		if (err < 0)
			return err;

		*val = tmp_val;
		break;
	case IIO_CHAN_INFO_ENABLE:
		err = ad3552r_get_ch_value(dac, AD3552R_CH_DAC_POWERDOWN,
					   _TO_CH_DAC(ch), &tmp_val);
		if (err < 0)
			return err;
		*val = tmp_val;
		break;
	case IIO_CHAN_INFO_SCALE:
		*val = dac->ch_data[_TO_CH_DAC(ch)].scale_int;
		*val2 = dac->ch_data[_TO_CH_DAC(ch)].scale_dec;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_OFFSET:
		*val = dac->ch_data[_TO_CH_DAC(ch)].offset_int;
		*val2 = dac->ch_data[_TO_CH_DAC(ch)].offset_dec;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

static int ad3552r_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct ad3552r_desc *dac = iio_priv(indio_dev);
	enum ad3552r_ch_attributes attr;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		attr = AD3552R_CH_CODE;
		break;
	case IIO_CHAN_INFO_ENABLE:
		attr = AD3552R_CH_DAC_POWERDOWN;
		val = !!val;
		break;
	default:
		return -EINVAL;
	}

	return ad3552r_set_ch_value(dac, attr, chan->channel, val);
}

static const unsigned long single_masks[] = {
	AD3552R_MASK_ALL_CH,
	AD3552R_MASK_ALL_CH << AD3552R_PREC_CH_START,
	BIT(2),
	BIT(5)
};

static int ad3552r_update_scan_mode(struct iio_dev *indio_dev,
				    const unsigned long *scan_mask)
{
	struct ad3552r_desc	*dac = iio_priv(indio_dev);
	unsigned long		mask;
	u32		len;
	int			i;

	mask = *scan_mask;
	/* If writing to both channels. Can't write to any other channel */
	for (i = 0; i < ARRAY_SIZE(single_masks); ++i)
		if ((mask & single_masks[i]) == single_masks[i])
			if (mask & ~single_masks[i])
				return -EINVAL;

	/* If same channel active for prec and fast mask is invalid */
	for (i = 0; i < AD3552R_NUM_CH; ++i)
		if ((mask & BIT(i)) && (mask & BIT(i + AD3552R_PREC_CH_START)))
			return -EINVAL;
	len = 0;
	if ((mask & single_masks[0]) == single_masks[0])
		len = AD3552R_NUM_CH * 2;
	else if ((mask & single_masks[1]) == single_masks[1])
		len = AD3552R_NUM_CH * 3;
	else if ((mask & single_masks[2]) == single_masks[2])
		len = 2;
	else if ((mask & single_masks[3]) == single_masks[3])
		len = 3;
	else {
		if (mask & AD3552R_MASK_ALL_CH)
			len += 2;
		if (mask & (AD3552R_MASK_ALL_CH << AD3552R_PREC_CH_START))
			len += 3;
	}

	dac->bytes_per_datum = len;
	dac->mask = *scan_mask;

	return 0;
}

/*
 * Device type specific information.
 */
static const struct iio_info ad3552r_iio_info = {
	.read_raw = ad3552r_read_raw,
	.write_raw = ad3552r_write_raw,
	.debugfs_reg_access = ad3552r_reg_access,
	.update_scan_mode = ad3552r_update_scan_mode
};

static irqreturn_t ad3552r_trigger_handler(int irq, void *p)
{
	struct iio_poll_func	*pf = p;
	struct iio_dev		*indio_dev = pf->indio_dev;
	struct iio_buffer	*buf = indio_dev->buffer;
	struct ad3552r_desc	*dac = iio_priv(indio_dev);
	char			buff[AD3552R_NUM_CH * 3];
	int			err;

	mutex_lock(&dac->lock);

	err = iio_buffer_remove_sample(buf, buff);
	if (err)
		goto end;

	err = _ad3552r_write_codes(dac, dac->mask, buff);
	if (err)
		goto end;

end:
	iio_trigger_notify_done(indio_dev->trig);
	mutex_unlock(&dac->lock);

	return IRQ_HANDLED;
}

static int ad3552r_setup_trigger_buffer(struct device *dev,
					struct iio_dev *indio_dev, int irq)
{
	struct ad3552r_desc	*dac = iio_priv(indio_dev);
	struct iio_trigger	*hwtrig;
	int			err;

	/* Configure trigger buffer */
	err = devm_iio_triggered_buffer_setup(dev, indio_dev, NULL,
					      &ad3552r_trigger_handler, NULL);
	if (err)
		return err;

	if (irq) {
		hwtrig = devm_iio_trigger_alloc(dev, "%s-ldac-dev%d",
						indio_dev->name, indio_dev->id);
		if (!hwtrig)
			return -ENOMEM;

		hwtrig->dev.parent = dev;
		iio_trigger_set_drvdata(hwtrig, dac);
		err = devm_iio_trigger_register(dev, hwtrig);
		if (err < 0)
			return err;

		err = devm_request_threaded_irq(dev, irq,
					iio_trigger_generic_data_rdy_poll,
					NULL,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					indio_dev->name,
					hwtrig);
		if (err < 0)
			return err;
	}

	return 0;
}

static int _ad3552r_check_scratch_pad(struct ad3552r_desc *desc)
{
	const u16 val1 = 0x3A;
	const u16 val2 = 0xB2;
	u16 val;
	int err;

	err = ad3552r_write_reg(desc, AD3552R_REG_ADDR_SCRATCH_PAD, val1);
	if (err < 0)
		return err;

	err = ad3552r_read_reg(desc, AD3552R_REG_ADDR_SCRATCH_PAD, &val);
	if (err < 0)
		return err;

	if (val1 != val)
		return -ENODEV;

	err = ad3552r_write_reg(desc, AD3552R_REG_ADDR_SCRATCH_PAD, val2);
	if (err < 0)
		return err;

	err = ad3552r_read_reg(desc, AD3552R_REG_ADDR_SCRATCH_PAD, &val);
	if (err < 0)
		return err;

	if (val2 != val)
		return -ENODEV;

	return 0;
}

static int _ad3552r_reset(struct ad3552r_desc *dac)
{
	int timeout_ms = 1000;
	int ret;
	u16 val;

	if (dac->gpio_reset) {
		/* Perform hardwer reset */
		gpiod_set_value_cansleep(dac->gpio_reset, 0);
		usleep_range(10, 20);
		gpiod_set_value_cansleep(dac->gpio_reset, 1);
	} else {
		/* Perform software reset if no GPIO provided */
		ret = _ad3552r_write_reg_mask(dac,
					AD3552R_REG_ADDR_INTERFACE_CONFIG_A,
					AD3552R_MASK_SOFTWARE_RESET,
					AD3552R_MASK_SOFTWARE_RESET);
		if (ret < 0)
			return ret;

	}

	/*
	 * Wait for default value in register otherwise random values can
	 * be found in AD3552R_REG_ADDR_INTERFACE_STATUS_A
	 */
	do {
		ret = ad3552r_read_reg(dac, AD3552R_REG_ADDR_INTERFACE_CONFIG_B,
				       &val);
		if (ret < 0)
			return ret;
		usleep_range(100, 1000);
		timeout_ms--;
	} while (val != AD3552R_DEFAULT_CONFIG_B_VALUE && timeout_ms);

	timeout_ms = 1000;
	/* Wait for interface to be ready */
	do {
		ret = ad3552r_read_reg(dac, AD3552R_REG_ADDR_INTERFACE_STATUS_A,
				       &val);
		if (ret < 0)
			return ret;
		usleep_range(100, 1000);
		timeout_ms--;
	} while (val & AD3552R_MASK_INTERFACE_NOT_READY && timeout_ms);

	ret = ad3552r_set_dev_value(dac, AD3552R_ADDR_ASCENSION, 0);
	if (ret < 0)
		return ret;

	return 0;
}

static void _ad3552r_set_gain_and_offset(struct ad3552r_desc *dac)
{
	const s64 TO_MICROS = 1000000;
	s32 i, idx, v_max, v_min, span, rem;
	s64 tmp;

	for (i = 0; i < AD3552R_NUM_CH; i++) {
		if (dac->ch_data[i].range_override) {
			//TODO
			v_max = 2500;
			v_min = 0;
		} else {
			/* Normal range */
			idx = dac->ch_data[i].range;
			v_max = ch_ranges[idx][1];
			v_min = ch_ranges[idx][0];
		}
		span = v_max - v_min;
		dac->ch_data[i].scale_int = div_s64_rem(span, 65536, &rem);
		dac->ch_data[i].scale_dec = DIV_ROUND_CLOSEST(
						(s64)rem * TO_MICROS, 65536);

		dac->ch_data[i].offset_int = div_s64_rem(v_min * 65536, span,
							 &rem);
		tmp = (s64)rem * TO_MICROS;
		dac->ch_data[i].offset_dec = div_s64(tmp, span);
	}
}

static int _ad3552r_configure_device(struct ad3552r_desc *dac)
{
	static const enum ad3552r_ch_attributes gain_attrs[] = {
		AD3552R_CH_GAIN_SCALING_P,
		AD3552R_CH_GAIN_SCALING_N,
		AD3552R_CH_RFB
	};
	static const char * const gain_dts_names[] = {
		"adi,gain-sacling-p",
		"adi,gain-sacling-n",
		"adi,rfb"
	};
	struct fwnode_handle	*child;
	struct fwnode_handle	*custom_gain_child;
	int		i;
	int		err;
	u32		val;
	u32		ch;
	bool		is_custom;

	err = device_property_read_u32(&dac->spi->dev, "adi,vref-select", &val);
	if (!err) {
		if (val > 2)
			return -EINVAL;
		err = ad3552r_set_dev_value(dac, AD3552R_VREF_SELECT, val);
		if (err)
			return err;
	}

	val = device_property_read_bool(&dac->spi->dev, "crc-en");
	err = ad3552r_set_dev_value(dac, AD3552R_CRC_ENABLE, val);
	if (err)
		return err;

	dac->num_ch = device_get_child_node_count(&dac->spi->dev);
	if (!dac->num_ch) {
		dev_err(&dac->spi->dev, "No channel children\n");
		return -ENODEV;
	}

	device_for_each_child_node(&dac->spi->dev, child) {
		err = fwnode_property_read_u32(child, "num", &ch);
		if (err)
			return err;
		if (ch >= AD3552R_NUM_CH)
			return -EINVAL;

		val = fwnode_property_read_bool(child, "adi,dac-powerdown");
		err = ad3552r_set_ch_value(dac, AD3552R_CH_AMPLIFIER_POWERDOWN,
					   ch, val);
		if (err)
			return err;

		if (fwnode_property_present(child, "adi,output-range")) {
			is_custom = false;
			err = fwnode_property_read_u32(child,
						       "adi,output-range",
						       &val);
			if (err)
				return err;
			if (val > AD3552R_CH_OUTPUT_RANGE_NEG_10__10V)
				return -EINVAL;

			err = ad3552r_set_ch_value(dac,
						   AD3552R_CH_OUTPUT_RANGE_SEL,
						   ch, val);
			if (err)
				return err;
		} else {
			is_custom = true;
			custom_gain_child = fwnode_get_named_child_node(child,
						"custom-output-range-config");
			if (IS_ERR(custom_gain_child))
				return PTR_ERR(custom_gain_child);

			err = fwnode_property_read_u32(custom_gain_child,
						       "adi,gain-offset", &val);
			if (err)
				return err;

			err = ad3552r_set_ch_value(dac,
						   AD3552R_CH_GAIN_OFFSET,
						   ch, abs(val));
			if (err)
				return err;

			err = ad3552r_set_ch_value(dac,
						AD3552R_CH_GAIN_OFFSET_POLARITY,
						ch, val < 0);
			if (err)
				return err;

			for (i = 0; i < ARRAY_SIZE(gain_attrs); ++i) {
				err = fwnode_property_read_u32(
							custom_gain_child,
							gain_dts_names[i],
							&val);
				if (err)
					return err;

				err = ad3552r_set_ch_value(dac, gain_attrs[i],
							   ch, val);
				if (err)
					return err;
			}
		}
		err = ad3552r_set_ch_value(dac, AD3552R_CH_RANGE_OVERRIDE, ch,
					   is_custom);
		if (err)
			return err;
	}

	_ad3552r_set_gain_and_offset(dac);

	return 0;
}

static int _ad3552r_init(struct ad3552r_desc *dac)
{
	int err;

	dac->gpio_reset = devm_gpiod_get_optional(&dac->spi->dev, "reset",
						 GPIOD_OUT_HIGH);
	if (IS_ERR(dac->gpio_reset))
		return PTR_ERR(dac->gpio_reset);

	err = _ad3552r_reset(dac);
	if (err)
		return err;

	err = _ad3552r_check_scratch_pad(dac);
	if (err) {
		dev_err(&dac->spi->dev, "Scratch pad test failed\n");
		return err;
	}

	return _ad3552r_configure_device(dac);
}

static int ad3552r_probe(struct spi_device *spi)
{
	struct ad3552r_desc	*dac;
	struct iio_dev		*indio_dev;
	int			err;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*dac));
	if (!indio_dev)
		return -ENOMEM;

	dac = iio_priv(indio_dev);
	dac->indio_dev = indio_dev;
	dac->spi = spi;

	mutex_init(&dac->lock);

	crc8_populate_msb(dac->crc_table, AD3552R_CRC_POLY);

	err = _ad3552r_init(dac);
	if (err)
		return err;

	/* Config triggered buffer device */
	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = "ad3552r";
	indio_dev->channels = ad3552r_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad3552r_channels);
	indio_dev->info = &ad3552r_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->direction = IIO_DEVICE_DIRECTION_OUT;
	err = ad3552r_setup_trigger_buffer(&spi->dev, indio_dev, spi->irq);
	if (err)
		return err;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ad3552r_of_match[] = {
	{ .compatible = "adi,ad3552r" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad3552r_of_match);

static struct spi_driver ad3552r_driver = {
	.driver = {
		.name = "ad3552r",
		.of_match_table = ad3552r_of_match,
	},
	.probe = ad3552r_probe
};
module_spi_driver(ad3552r_driver);

MODULE_AUTHOR("Mihail Chindris <mihail.chindris@analog.com>");
MODULE_DESCRIPTION("Analog Device AD3552R DAC");
MODULE_LICENSE("Dual BSD/GPL");
