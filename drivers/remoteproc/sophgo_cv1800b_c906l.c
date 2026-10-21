// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2025 Junhui Liu <junhui.liu@pigmoral.tech>
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/mailbox_client.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/reset.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#include "remoteproc_internal.h"

#define CV1800B_SYS_C906L_CTRL_REG	0x04
#define CV1800B_SYS_C906L_CTRL_EN	BIT(13)

#define CV1800B_SYS_C906L_BOOTADDR_REG	0x20

/* XXX debug: the delay lets the line reach the console before a hang */
#define C906L_DBG(dev)						\
	do {							\
		dev_info(dev, "%s:%d\n", __func__, __LINE__);	\
		mdelay(50);					\
	} while (0)

/* XXX debug: IPC paths run per message, so no delay here */
#define C906L_IPC_DBG(dev, fmt, ...) \
	dev_info(dev, "%s: " fmt, __func__, ##__VA_ARGS__)

struct cv1800b_c906l {
	struct rproc *rproc;
	struct reset_control *reset;
	struct regmap *syscon;
	struct clk_bulk_data *clks;
	int num_clks;
	struct mbox_client cl;
	struct mbox_chan *tx_ch;
	struct mbox_chan *rx_ch;
	struct work_struct rx_work;
	void __iomem *rsc_table;
	size_t rsc_table_sz;
};

static int cv1800b_c906l_add_carveout(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct device_node *np = dev->of_node;
	struct rproc_mem_entry *mem;
	struct resource res;
	int ret;

	C906L_DBG(dev);
	/* Second memory-region is the firmware region */
	ret = of_reserved_mem_region_to_resource(np, 1, &res);
	if (ret)
		return ret;

	mem = rproc_mem_entry_init(dev, NULL,
				   (dma_addr_t)res.start,
				   resource_size(&res), res.start,
				   rproc_mem_entry_ioremap_wc,
				   rproc_mem_entry_iounmap,
				   "firmware");
	if (!mem)
		return -ENOMEM;

	rproc_add_carveout(rproc, mem);

	C906L_DBG(dev);
	return 0;
}

static int cv1800b_c906l_prepare(struct rproc *rproc)
{
	return cv1800b_c906l_add_carveout(rproc);
}

static int cv1800b_c906l_start(struct rproc *rproc)
{
	struct cv1800b_c906l *priv = rproc->priv;
	struct device *dev = &rproc->dev;
	u32 bootaddr[2];
	int ret;

	bootaddr[0] = lower_32_bits(rproc->bootaddr);
	bootaddr[1] = upper_32_bits(rproc->bootaddr);

	dev_info(dev, "%s: bootaddr 0x%llx\n", __func__, (u64)rproc->bootaddr);
	C906L_IPC_DBG(dev, "rsc table %p (%zu bytes), max notifyid %d\n",
		      rproc->table_ptr, rproc->table_sz, rproc->max_notifyid);
	C906L_DBG(dev);

	ret = reset_control_assert(priv->reset);
	if (ret)
		return ret;

	C906L_DBG(dev);

	/*
	 * This control bit must be set to enable the C906L remote processor.
	 * Note that once the remote processor is running, merely clearing
	 * this bit will not stop its execution.
	 */
	ret = regmap_update_bits(priv->syscon, CV1800B_SYS_C906L_CTRL_REG,
				  CV1800B_SYS_C906L_CTRL_EN,
				  CV1800B_SYS_C906L_CTRL_EN);
	if (ret)
		return ret;

	C906L_DBG(dev);

	ret = regmap_bulk_write(priv->syscon, CV1800B_SYS_C906L_BOOTADDR_REG,
				bootaddr, ARRAY_SIZE(bootaddr));
	if (ret)
		return ret;

	C906L_DBG(dev);

	ret = reset_control_deassert(priv->reset);

	C906L_DBG(dev);

	return ret;
}

static int cv1800b_c906l_stop(struct rproc *rproc)
{
	struct cv1800b_c906l *priv = rproc->priv;

	return reset_control_assert(priv->reset);
}

static int cv1800b_c906l_attach(struct rproc *rproc)
{
	/* The C906L was started by the bootloader, nothing to do */
	C906L_IPC_DBG(&rproc->dev, "");
	return 0;
}

static struct resource_table *
cv1800b_c906l_get_loaded_rsc_table(struct rproc *rproc, size_t *table_sz)
{
	struct cv1800b_c906l *priv = rproc->priv;
	struct resource_table *table;

	if (!priv->rsc_table)
		return NULL;

	C906L_IPC_DBG(&rproc->dev, "");
	table = (struct resource_table __force *)priv->rsc_table;

	/*
	 * The firmware copies its resource table here when it boots. Treat
	 * anything that does not look like one as no resource table at all.
	 */
	if (table->ver != 1 ||
	    struct_size(table, offset, table->num) > priv->rsc_table_sz) {
		dev_warn(&rproc->dev, "invalid resource table, ignoring it\n");
		return NULL;
	}

	*table_sz = priv->rsc_table_sz;

	return table;
}

static struct resource_table *
cv1800b_c906l_find_loaded_rsc_table(struct rproc *rproc,
				    const struct firmware *fw)
{
	struct cv1800b_c906l *priv = rproc->priv;

	/* No resource table in the firmware */
	if (!rproc->table_ptr)
		return NULL;

	/*
	 * Use the shared resource table region, so the same table is used
	 * whether Linux or the bootloader started the C906L.
	 */
	if (priv->rsc_table)
		return (struct resource_table __force *)priv->rsc_table;

	return rproc_elf_find_loaded_rsc_table(rproc, fw);
}

static int cv1800b_c906l_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	rproc_elf_load_rsc_table_optional(rproc, fw, dev_info,
					  "no resource table in elf\n");
	return 0;
}

static void cv1800b_c906l_kick(struct rproc *rproc, int vqid)
{
	struct cv1800b_c906l *priv = rproc->priv;
	u64 msg = vqid;
	int ret;

	if (!priv->tx_ch) {
		dev_warn_once(&rproc->dev, "no mailbox, cannot kick vq %d\n",
			      vqid);
		return;
	}

	C906L_IPC_DBG(&rproc->dev, "kick vq %d\n", vqid);
	ret = mbox_send_message(priv->tx_ch, &msg);
	C906L_IPC_DBG(&rproc->dev, "kick vq %d done: %d\n", vqid, ret);
	if (ret < 0)
		dev_err(&rproc->dev, "failed to kick vq %d: %d\n", vqid, ret);
}

/* XXX debug */
static int cv1800b_c906l_load(struct rproc *rproc, const struct firmware *fw)
{
	int ret;

	C906L_DBG(&rproc->dev);
	ret = rproc_elf_load_segments(rproc, fw);
	C906L_DBG(&rproc->dev);

	return ret;
}

static const struct rproc_ops cv1800b_c906l_ops = {
	.prepare = cv1800b_c906l_prepare,
	.start = cv1800b_c906l_start,
	.stop = cv1800b_c906l_stop,
	.attach = cv1800b_c906l_attach,
	.kick = cv1800b_c906l_kick,
	.load = cv1800b_c906l_load,
	.parse_fw = cv1800b_c906l_parse_fw,
	.find_loaded_rsc_table = cv1800b_c906l_find_loaded_rsc_table,
	.get_loaded_rsc_table = cv1800b_c906l_get_loaded_rsc_table,
	.sanity_check = rproc_elf_sanity_check,
	.get_boot_addr = rproc_elf_get_boot_addr,
};

static int cv1800b_c906l_vq_notify(int id, void *p, void *data)
{
	struct rproc *rproc = data;
	irqreturn_t irq;

	irq = rproc_vq_interrupt(rproc, id);
	C906L_IPC_DBG(&rproc->dev, "vq %d: %s\n", id,
		      irq == IRQ_HANDLED ? "handled" : "nothing pending");

	return 0;
}

static void cv1800b_c906l_mbox_rx_work(struct work_struct *work)
{
	struct cv1800b_c906l *priv = container_of(work, struct cv1800b_c906l,
						  rx_work);

	C906L_IPC_DBG(&priv->rproc->dev, "rx work\n");
	idr_for_each(&priv->rproc->notifyids, cv1800b_c906l_vq_notify,
		     priv->rproc);
}

static void cv1800b_c906l_mbox_rx_callback(struct mbox_client *cl, void *msg)
{
	struct cv1800b_c906l *priv = container_of(cl, struct cv1800b_c906l, cl);

	C906L_IPC_DBG(cl->dev, "rx msg 0x%llx\n", *(u64 *)msg);
	schedule_work(&priv->rx_work);
}

static void cv1800b_c906l_free_mbox(void *data)
{
	struct cv1800b_c906l *priv = data;

	mbox_free_channel(priv->rx_ch);
	mbox_free_channel(priv->tx_ch);
}

static int cv1800b_c906l_mbox_init(struct device *dev,
				   struct cv1800b_c906l *priv)
{
	struct mbox_client *cl = &priv->cl;
	int ret;

	if (!of_property_present(dev->of_node, "mboxes")) {
		C906L_IPC_DBG(dev, "no mboxes, IPC disabled\n");
		return 0;
	}

	ret = devm_work_autocancel(dev, &priv->rx_work,
				   cv1800b_c906l_mbox_rx_work);
	if (ret)
		return ret;

	cl->dev = dev;
	cl->rx_callback = cv1800b_c906l_mbox_rx_callback;
	cl->tx_block = true;
	cl->tx_tout = 50;
	cl->knows_txdone = false;

	priv->tx_ch = mbox_request_channel_byname(cl, "tx");
	if (IS_ERR(priv->tx_ch))
		return dev_err_probe(dev, PTR_ERR(priv->tx_ch),
				     "failed to request tx mailbox channel\n");

	priv->rx_ch = mbox_request_channel_byname(cl, "rx");
	if (IS_ERR(priv->rx_ch)) {
		mbox_free_channel(priv->tx_ch);
		return dev_err_probe(dev, PTR_ERR(priv->rx_ch),
				     "failed to request rx mailbox channel\n");
	}

	C906L_IPC_DBG(dev, "mailbox tx/rx channels ready\n");

	return devm_add_action_or_reset(dev, cv1800b_c906l_free_mbox, priv);
}

static int cv1800b_c906l_mem_init(struct device *dev,
				  struct cv1800b_c906l *priv)
{
	struct resource res;
	int count;
	int ret;

	count = of_reserved_mem_region_count(dev->of_node);
	if (count < 0)
		return count;

	if (count != 2 && count != 3)
		return -EINVAL;

	/*
	 * The DMA pool is the first memory-region. Assign it to this device so
	 * the vrings are allocated from it. The virtio buffers come from the
	 * same pool, through the core's fallback to the parent's first
	 * memory-region.
	 */
	ret = devm_of_reserved_mem_device_init(dev);
	if (ret)
		return ret;

	/* The optional third memory-region is the resource table region */
	if (count < 3)
		return 0;

	ret = of_reserved_mem_region_to_resource(dev->of_node, 2, &res);
	if (ret)
		return ret;

	priv->rsc_table = devm_ioremap_resource_wc(dev, &res);
	if (IS_ERR(priv->rsc_table))
		return PTR_ERR(priv->rsc_table);

	priv->rsc_table_sz = resource_size(&res);

	return 0;
}

static bool cv1800b_c906l_is_running(struct cv1800b_c906l *priv)
{
	u32 ctrl;
	int ret;

	ret = reset_control_status(priv->reset);
	if (ret)
		return false;

	ret = regmap_read(priv->syscon, CV1800B_SYS_C906L_CTRL_REG, &ctrl);
	if (ret)
		return false;

	return ctrl & CV1800B_SYS_C906L_CTRL_EN;
}

static int cv1800b_c906l_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct cv1800b_c906l *priv;
	const char *fw_name = NULL;
	struct rproc *rproc;
	int ret;

	ret = rproc_of_parse_firmware(dev, 0, &fw_name);
	if (ret && ret != -EINVAL)
		return dev_err_probe(dev, ret, "failed to get firmware filename\n");

	rproc = devm_rproc_alloc(dev, dev_name(dev), &cv1800b_c906l_ops,
				 fw_name, sizeof(*priv));
	if (!rproc)
		return -ENOMEM;

	priv = rproc->priv;
	priv->rproc = rproc;

	ret = cv1800b_c906l_mem_init(dev, priv);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to init reserved memory regions\n");

	C906L_DBG(dev);

	priv->num_clks = devm_clk_bulk_get_all_enabled(dev, &priv->clks);
	if (priv->num_clks < 0)
		return dev_err_probe(dev, priv->num_clks,
				     "failed to get clocks\n");
	if (!priv->num_clks)
		return dev_err_probe(dev, -ENOENT, "no clocks found\n");

	priv->syscon = syscon_regmap_lookup_by_phandle(np, "sophgo,sec-syscon");
	if (IS_ERR(priv->syscon))
		return dev_err_probe(dev, PTR_ERR(priv->syscon),
				     "failed to get system controller\n");

	priv->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(priv->reset))
		return dev_err_probe(dev, PTR_ERR(priv->reset),
				     "failed to get reset control handle\n");

	ret = cv1800b_c906l_mbox_init(dev, priv);
	if (ret)
		return ret;

	if (cv1800b_c906l_is_running(priv)) {
		dev_info(dev, "rproc already running, attaching\n");
		rproc->state = RPROC_DETACHED;
		rproc->auto_boot = true;
	} else {
		rproc->auto_boot = false;
	}

	C906L_DBG(dev);

	platform_set_drvdata(pdev, rproc);

	ret = devm_rproc_add(dev, rproc);
	if (ret)
		return dev_err_probe(dev, ret, "rproc_add failed\n");

	C906L_DBG(dev);

	return 0;

}

static const struct of_device_id cv1800b_c906l_of_match[] = {
	{ .compatible = "sophgo,cv1800b-c906l" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, cv1800b_c906l_of_match);

static struct platform_driver cv1800b_c906l_driver = {
	.probe = cv1800b_c906l_probe,
	.driver = {
		.name = "cv1800b-c906l",
		.of_match_table = cv1800b_c906l_of_match,
	},
};

module_platform_driver(cv1800b_c906l_driver);

MODULE_AUTHOR("Junhui Liu <junhui.liu@pigmoral.tech>");
MODULE_DESCRIPTION("Sophgo CV1800B C906L remote processor control driver");
MODULE_LICENSE("GPL");
