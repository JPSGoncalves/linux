// SPDX-License-Identifier: GPL-2.0
/*
 * Sophgo CV18xx/SG200x security subsystem system controller (SEC_SYS) driver
 *
 * Copyright (C) 2026 João Paulo Gonçalves <jpaulo.silvagoncalves@gmail.com>
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

static const struct regmap_config cv1800_secsys_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x24,
};

static int cv1800_secsys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regmap *regmap;
	void __iomem *base;
	struct clk *clk;
	int ret;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	clk = devm_clk_get_enabled(dev, "apb");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "failed to enable apb clock\n");

	clk = devm_clk_get_enabled(dev, "efuse");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "failed to enable efuse clock\n");

	regmap = devm_regmap_init_mmio(dev, base, &cv1800_secsys_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "failed to init regmap\n");

	/* The syscon core cannot unregister, so this device must never unbind */
	ret = of_syscon_register_regmap(dev->of_node, regmap);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register syscon\n");

	return 0;
}

static const struct of_device_id cv1800_secsys_of_match[] = {
	{ .compatible = "sophgo,cv1800b-sec-sys" },
	{ /* sentinel */ }
};

static struct platform_driver cv1800_secsys_driver = {
	.probe = cv1800_secsys_probe,
	.driver = {
		.name = "cv1800-secsys",
		.of_match_table = cv1800_secsys_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(cv1800_secsys_driver);
