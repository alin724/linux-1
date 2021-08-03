/*
 * Altera FPLL driver
 *
 * Copyright 2021 Analog Devices Inc.
 *  Author: Liviu Adace <liviu.adace@analog.com>
 *
 * Licensed under the GPL-2.
 *
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define C5_FPLL_MODE_REG_ADDR                      0x00
#define C5_FPLL_STATUS_REG_ADDR                    0x01
#define C5_FPLL_START_REG_ADDR                     0x02
#define C5_FPLL_N_CNTR_ADDR                        0x03
#define C5_FPLL_M_CNTR_ADDR                        0x04
#define C5_FPLL_C_CNTR_ADDR                        0x05
#define C5_FPLL_DYNAMIC_PHASE_SHIFT_ADDR           0x06
#define C5_FPLL_M_CNTR_FRACTIONAL_VAL_ADDR         0x07
#define C5_FPLL_BANDWITH_SETTINGS_ADDR             0x08
#define C5_FPLL_CHARGE_PUMP_SETTINGS_ADDR          0x09
#define C5_FPLL_VCO_POST_DEVIDE_CNTR_SETTINGS_ADDR 0x1C
#define C5_FPLL_MIF_BASE_ADDR                      0x1F

#define C5_FPLL_REG_C_CNTR(x)				((x) << 18)
#define C5_FPLL_VCO_MAX						1600000000ull
#define C5_FPLL_NUM_CHAN				    9

struct altera_c5_fpll_chan {
	struct altera_c5_fpll	*fpll;
	unsigned int			num;
	unsigned long			rate;
	struct clk_hw			hw;
};

#define to_channel(_hw) container_of(_hw, struct altera_c5_fpll_chan, hw)

struct altera_c5_fpll {
	void __iomem *base;
	struct device *dev;
	unsigned long rate;
	struct clk *ref_clk;
	unsigned int num_channels;
	const char *clk_out_names[C5_FPLL_NUM_CHAN];
	struct altera_c5_fpll_chan	channels[C5_FPLL_NUM_CHAN];
	struct clk					*clks[C5_FPLL_NUM_CHAN];
	struct clk_onecell_data		clk_data;
};

static long altera_c5_fpll_round_rate(struct clk_hw *hw,
	unsigned long rate, unsigned long *parent_rate)
{
	 struct altera_c5_fpll_chan *chan = to_channel(hw);
	 struct altera_c5_fpll *fpll = chan->fpll;

	return rate;
}

static int altera_c5_fpll_set_rate(struct clk_hw *hw, unsigned long rate,
	unsigned long parent_rate)
{

	struct altera_c5_fpll_chan *chan = to_channel(hw);
	struct altera_c5_fpll *fpll = chan->fpll;
	uint64_t vco;
	uint32_t in;
	uint8_t c;
	uint32_t reg;
	uint32_t m_frac;
	uint32_t m_int;
	uint64_t m;
	uint8_t i;
	u32 status;

	c = DIV_ROUND_CLOSEST_ULL(C5_FPLL_VCO_MAX, rate);
	vco = rate * c;
	if (vco > C5_FPLL_VCO_MAX) {
		c--;
		vco = rate * c;
	}

	in = clk_get_rate(fpll->ref_clk);

	m = (vco << 32) + (in / 2);
	do_div(m, in);
	m_frac = m & 0xffffffff;
	m_int = m >> 32;

	reg = m_int / 2;
	reg |= ((m_int / 2) + (m_int % 2)) << 8;

	if (m_int % 2)
		reg |= (1 << 17);

 	writel(0x01u, fpll->base + C5_FPLL_MODE_REG_ADDR * 4);
	writel(0x10101u, fpll->base + C5_FPLL_N_CNTR_ADDR * 4); 
	writel(reg, fpll->base + C5_FPLL_M_CNTR_ADDR * 4);
 	writel(m_frac, fpll->base + C5_FPLL_M_CNTR_FRACTIONAL_VAL_ADDR * 4);

	chan->rate = rate;

	for (i = 0; i < fpll->num_channels; i++) {
		c = DIV_ROUND_CLOSEST_ULL(vco, fpll->channels[i].rate);
		fpll->channels[i].rate = DIV_ROUND_CLOSEST_ULL(vco, c);

		reg = c / 2;
		reg |= ((c / 2) + (c % 2)) << 8;
		reg |= C5_FPLL_REG_C_CNTR(fpll->channels[i].num);

		if (c % 2)
			reg |= (1 << 17);

		writel(reg, fpll->base + C5_FPLL_C_CNTR_ADDR * 4);
	}

 	writel(0x00u, fpll->base + C5_FPLL_START_REG_ADDR * 4);

	do {
		status = readl(fpll->base + C5_FPLL_STATUS_REG_ADDR * 4);
	} while (status == 0);

	return 0;
}

static unsigned long altera_c5_fpll_recalc_rate(struct clk_hw *hw,
	unsigned long parent_rate)
{
	 struct altera_c5_fpll_chan *chan = to_channel(hw);
	 struct altera_c5_fpll *fpll = chan->fpll;

	return chan->rate;
}

static const struct clk_ops altera_c5_fpll_ops = {
	.recalc_rate = altera_c5_fpll_recalc_rate,
	.round_rate = altera_c5_fpll_round_rate,
	.set_rate = altera_c5_fpll_set_rate,
};

static int altera_c5_fpll_parse_dt(struct altera_c5_fpll *fpll)
{
	struct device_node *np = fpll->dev->of_node, *chan_np;
	unsigned int cnt = 0;
	u32 tmp;
	int ret;

	fpll->ref_clk = devm_clk_get(fpll->dev, NULL);
	if (IS_ERR(fpll->ref_clk))
		return PTR_ERR(fpll->ref_clk);

	for_each_child_of_node(np, chan_np)
		fpll->num_channels++;
	if (fpll->num_channels > C5_FPLL_NUM_CHAN)
		return -EINVAL;

	for_each_child_of_node(np, chan_np) {
		of_property_read_u32(chan_np, "reg",
				    &fpll->channels[cnt].num);
		of_property_read_u32(chan_np, "adi,rate",
					&tmp);
		fpll->channels[cnt].rate = tmp;
		cnt++;
	};

	ret = of_property_read_string_array(np, "clock-output-names",
			fpll->clk_out_names, ARRAY_SIZE(fpll->clk_out_names));
	if (ret < 0)
		return ret;

	return 0;
};

static int altera_c5_fpll_outputs_setup(struct altera_c5_fpll *fpll)
{
	struct clk_init_data init;
	struct clk *clk;
	int i;

	for (i = 0; i < fpll->num_channels; i++) {
		init.name = fpll->clk_out_names[i];
		init.ops = &altera_c5_fpll_ops;
		init.flags = 0;
		init.parent_names = NULL;
		init.num_parents = 0;

		fpll->channels[i].fpll = fpll;
		fpll->channels[i].hw.init = &init;
		clk = devm_clk_register(fpll->dev, &fpll->channels[i].hw);
		if (IS_ERR(clk))
			return PTR_ERR(clk);

		fpll->clks[i] = clk;
	}

	return 0;
}

static const struct of_device_id altera_c5_fpll_ids[] = {
	{ .compatible = "altr,c5-fpll", },
	{ },
};
MODULE_DEVICE_TABLE(of, altera_c5_fpll_ids);

static int altera_c5_fpll_probe(struct platform_device *pdev)
{
	struct altera_c5_fpll *fpll;
	struct resource *mem;
	int ret;

	fpll = devm_kzalloc(&pdev->dev, sizeof(*fpll), GFP_KERNEL);
	if (!fpll)
		return -ENOMEM;

	fpll->dev = &pdev->dev;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	fpll->base = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(fpll->base))
		return PTR_ERR(fpll->base);

	ret = altera_c5_fpll_parse_dt(fpll);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(fpll->ref_clk);
	if (ret < 0)
		return ret;

	ret = altera_c5_fpll_outputs_setup(fpll);
	if (ret < 0)
		return ret;

	fpll->clk_data.clks = fpll->clks;
	fpll->clk_data.clk_num = fpll->num_channels;

	ret = of_clk_add_provider(pdev->dev.of_node, of_clk_src_onecell_get,
				    &fpll->clk_data);
	if (ret < 0)
		return ret;

	return 0;
}

static int altera_c5_fpll_remove(struct platform_device *pdev)
{
	of_clk_del_provider(pdev->dev.of_node);

	return 0;
}

static struct platform_driver altera_c5_fpll_driver = {
	.driver = {
		.name = "altera-c5-fpll",
		.of_match_table = altera_c5_fpll_ids,
	},
	.probe = altera_c5_fpll_probe,
	.remove = altera_c5_fpll_remove,
};

module_platform_driver(altera_c5_fpll_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Liviu Adace <liviu.adace@analog.com>");
MODULE_DESCRIPTION("Driver for the Altera Cyclone5 FPLL");
