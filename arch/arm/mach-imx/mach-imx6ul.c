// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 */
#include <linux/irqchip.h>
#include <linux/of_platform.h>
#include <asm/mach/arch.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/phy.h>
#include <linux/regmap.h>

#include "common.h"
#include "cpuidle.h"
#include "hardware.h"

static int lan8831_phy_fixup(struct phy_device *dev)
{
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6ul-iomuxc-gpr");
	if (!IS_ERR(gpr))
		regmap_update_bits(gpr, IOMUXC_GPR1, 3 << 13, 3 << 13);
	else
		pr_err("failed to find fsl,imx6ul-iomux-gpr regmap\n");

	return 0;
}

#define PHY_ID_LAN8831 0x221652

static void __init imx6ul_init_machine(void)
{
	imx_print_silicon_rev(cpu_is_imx6ull() ? "i.MX6ULL" : "i.MX6UL",
		imx_get_soc_revision());

	of_platform_default_populate(NULL, NULL, NULL);
	imx_anatop_init();
	imx6ul_pm_init();
	phy_register_fixup_for_uid(PHY_ID_LAN8831, 0xffffffff, lan8831_phy_fixup);
}

static void __init imx6ul_init_irq(void)
{
	imx_init_revision_from_anatop();
	imx_src_init();
	irqchip_init();
	imx6_pm_ccm_init("fsl,imx6ul-ccm");
}

static void __init imx6ul_init_late(void)
{
	imx6sx_cpuidle_init();

	if (IS_ENABLED(CONFIG_ARM_IMX6Q_CPUFREQ))
		platform_device_register_simple("imx6q-cpufreq", -1, NULL, 0);
}

static const char * const imx6ul_dt_compat[] __initconst = {
	"fsl,imx6ul",
	"fsl,imx6ull",
	"fsl,imx6ulz",
	NULL,
};

DT_MACHINE_START(IMX6UL, "Freescale i.MX6 Ultralite (Device Tree)")
	.init_irq	= imx6ul_init_irq,
	.init_machine	= imx6ul_init_machine,
	.init_late	= imx6ul_init_late,
	.dt_compat	= imx6ul_dt_compat,
MACHINE_END
