/*
 * drivers/spi/spi_tegra.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *     Erik Gilling <konkers@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#undef DEBUG

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <linux/spi/spi.h>

#include <mach/dma.h>

#define DRIVER_NAME		"spi_tegra"

#define BUSY_TIMEOUT		1000

#define SLINK_COMMAND		0x000
#define   BIT_LENGTH(x)			(((x) & 0x1f) << 0)
#define   WORD_SIZE(x)			(((x) & 0x1f) << 5)
#define   BOTH_EN			(1 << 10)
#define   CS_SW				(1 << 11)
#define   CS_VALUE			(1 << 12)
#define   CS_POLARITY			(1 << 13)
#define   IDLE_SDA_DRIVE_LOW		(0 << 16)
#define   IDLE_SDA_DRIVE_HIGH		(1 << 16)
#define   IDLE_SDA_PULL_LOW		(2 << 16)
#define   IDLE_SDA_PULL_HIGH		(3 << 16)
#define   IDLE_SDA_MASK			(3 << 16)
#define   CS_POLARITY1			(1 << 20)
#define   CK_SDA			(1 << 21)
#define   CS_POLARITY2			(1 << 22)
#define   CS_POLARITY3			(1 << 23)
#define   IDLE_SCLK_DRIVE_LOW		(0 << 24)
#define   IDLE_SCLK_DRIVE_HIGH		(1 << 24)
#define   IDLE_SCLK_PULL_LOW		(2 << 24)
#define   IDLE_SCLK_PULL_HIGH		(3 << 24)
#define   IDLE_SCLK_MASK		(3 << 24)
#define   M_S				(1 << 28)
#define   WAIT				(1 << 29)
#define   GO				(1 << 30)
#define   ENB				(1 << 31)

#define SLINK_COMMAND2		0x004
#define   LSBFE				(1 << 0)
#define   SSOE				(1 << 1)
#define   SPIE				(1 << 4)
#define   BIDIROE			(1 << 6)
#define   MODFEN			(1 << 7)
#define   INT_SIZE(x)			(((x) & 0x1f) << 8)
#define   CS_ACTIVE_BETWEEN		(1 << 17)
#define   SS_EN_CS(x)			(((x) & 0x3) << 18)
#define   SS_SETUP(x)			(((x) & 0x3) << 20)
#define   FIFO_REFILLS_0		(0 << 22)
#define   FIFO_REFILLS_1		(1 << 22)
#define   FIFO_REFILLS_2		(2 << 22)
#define   FIFO_REFILLS_3		(3 << 22)
#define   FIFO_REFILLS_MASK		(3 << 22)
#define   WAIT_PACK_INT(x)		(((x) & 0x7) << 26)
#define   SPC0				(1 << 29)
#define   TXEN				(1 << 30)
#define   RXEN				(1 << 31)

#define SLINK_STATUS		0x008
#define   COUNT(val)			(((val) >> 0) & 0x1f)
#define   WORD(val)			(((val) >> 5) & 0x1f)
#define   BLK_CNT(val)			(((val) >> 0) & 0xffff)
#define   MODF				(1 << 16)
#define   RX_UNF			(1 << 18)
#define   TX_OVF			(1 << 19)
#define   TX_FULL			(1 << 20)
#define   TX_EMPTY			(1 << 21)
#define   RX_FULL			(1 << 22)
#define   RX_EMPTY			(1 << 23)
#define   TX_UNF			(1 << 24)
#define   RX_OVF			(1 << 25)
#define   TX_FLUSH			(1 << 26)
#define   RX_FLUSH			(1 << 27)
#define   SCLK				(1 << 28)
#define   ERR				(1 << 29)
#define   RDY				(1 << 30)
#define   BSY				(1 << 31)

#define SLINK_MAS_DATA		0x010
#define SLINK_SLAVE_DATA	0x014

#define SLINK_DMA_CTL		0x018
#define   DMA_BLOCK_SIZE(x)		(((x) & 0xffff) << 0)
#define   TX_TRIG_1			(0 << 16)
#define   TX_TRIG_4			(1 << 16)
#define   TX_TRIG_8			(2 << 16)
#define   TX_TRIG_16			(3 << 16)
#define   TX_TRIG_MASK			(3 << 16)
#define   RX_TRIG_1			(0 << 18)
#define   RX_TRIG_4			(1 << 18)
#define   RX_TRIG_8			(2 << 18)
#define   RX_TRIG_16			(3 << 18)
#define   RX_TRIG_MASK			(3 << 18)
#define   PACKED			(1 << 20)
#define   PACK_SIZE_4			(0 << 21)
#define   PACK_SIZE_8			(1 << 21)
#define   PACK_SIZE_16			(2 << 21)
#define   PACK_SIZE_32			(3 << 21)
#define   PACK_SIZE_MASK		(3 << 21)
#define   IE_TXC			(1 << 26)
#define   IE_RXC			(1 << 27)
#define   DMA_EN			(1 << 31)

#define SLINK_STATUS2		0x01c
#define   TX_FIFO_EMPTY_COUNT(val)	(((val) & 0x3f) >> 0)
#define   RX_FIFO_FULL_COUNT(val)	(((val) & 0x3f) >> 16)

#define SLINK_TX_FIFO		0x100
#define SLINK_RX_FIFO		0x180

static const unsigned long spi_tegra_req_sels[] = {
	TEGRA_DMA_REQ_SEL_SL2B1,
	TEGRA_DMA_REQ_SEL_SL2B2,
	TEGRA_DMA_REQ_SEL_SL2B3,
	TEGRA_DMA_REQ_SEL_SL2B4,
};

static inline unsigned bytes_per_word(u8 bits)
{
	WARN_ON((bits < 1) || bits > 32);
	if (bits <= 8)
		return 1;
	else if (bits <= 16)
		return 2;
	else if (bits <= 24)
		return 3;
	else
		return 4;
}

#define BB_LEN			32

struct spi_tegra_data {
	struct spi_master	*master;
	struct platform_device	*pdev;
	spinlock_t		lock;

	struct clk		*clk;
	void __iomem		*base;
	unsigned long		phys;

	u32			cur_speed;

	struct list_head	queue;
	struct spi_transfer	*cur;
	unsigned		cur_pos;
	unsigned		cur_len;
	unsigned		cur_bytes_per_word;

	/* The tegra spi controller has a bug which causes the first word
	 * in PIO transactions to be garbage.  Since packed DMA transactions
	 * require transfers to be 4 byte aligned we need a bounce buffer
	 * for the generic case.
	 */
	struct tegra_dma_req	rx_dma_req;
	struct tegra_dma_channel *rx_dma;
	u32			*rx_bb;
	dma_addr_t		rx_bb_phys;
};


static unsigned long spi_readl(struct spi_tegra_data *tspi, unsigned long reg)
{
	return readl(tspi->base + reg);
}

static void spi_writel(struct spi_tegra_data *tspi, unsigned long val,
		       unsigned long reg)
{
	writel(val, tspi->base + reg);
}

static void spi_tegra_go(struct spi_tegra_data *tspi)
{
	unsigned long val;

	wmb();

	val = spi_readl(tspi, SLINK_DMA_CTL);
	val &= ~DMA_BLOCK_SIZE(~0) & ~DMA_EN;
	val |= DMA_BLOCK_SIZE(tspi->rx_dma_req.size / 4 - 1);
	spi_writel(tspi, val, SLINK_DMA_CTL);

	tegra_dma_enqueue_req(tspi->rx_dma, &tspi->rx_dma_req);

	val |= DMA_EN;
	spi_writel(tspi, val, SLINK_DMA_CTL);
}

static unsigned spi_tegra_fill_tx_fifo(struct spi_tegra_data *tspi,
				  struct spi_transfer *t)
{
	unsigned len = min(t->len - tspi->cur_pos, BB_LEN *
			   tspi->cur_bytes_per_word);
	u8 *tx_buf = (u8 *)t->tx_buf + tspi->cur_pos;
	int i, j;
	unsigned long val;

	val = spi_readl(tspi, SLINK_COMMAND);
	val &= ~WORD_SIZE(~0);
	val |= WORD_SIZE(len / tspi->cur_bytes_per_word - 1);
	spi_writel(tspi, val, SLINK_COMMAND);

	for (i = 0; i < len; i += tspi->cur_bytes_per_word) {
		val = 0;
		for (j = 0; j < tspi->cur_bytes_per_word; j++)
			val |= tx_buf[i + j] << j * 8;

		spi_writel(tspi, val, SLINK_TX_FIFO);
	}

	tspi->rx_dma_req.size = len / tspi->cur_bytes_per_word * 4;

	return len;
}

static unsigned spi_tegra_drain_rx_fifo(struct spi_tegra_data *tspi,
				  struct spi_transfer *t)
{
	unsigned len = tspi->cur_len;
	u8 *rx_buf = (u8 *)t->rx_buf + tspi->cur_pos;
	int i, j;
	unsigned long val;

	for (i = 0; i < len; i += tspi->cur_bytes_per_word) {
		val = tspi->rx_bb[i / tspi->cur_bytes_per_word];
		for (j = 0; j < tspi->cur_bytes_per_word; j++)
			rx_buf[i + j] = (val >> (j * 8)) & 0xff;
	}

	return len;
}

static int spi_tegra_start_transfer(struct spi_device *spi,
				    struct spi_transfer *t)
{
	struct spi_tegra_data *tspi = spi_master_get_devdata(spi->master);
	u32 speed;
	u8 bits_per_word;
	unsigned long val;

	speed = t->speed_hz ? t->speed_hz : spi->max_speed_hz;
	bits_per_word = t->bits_per_word ? t->bits_per_word  :
		spi->bits_per_word;

	if (bits_per_word < 1 || bits_per_word > 32)
		return -EINVAL;

	if (t->len == 0)
		return -EINVAL;

	if (!t->rx_buf && !t->tx_buf)
		return -EINVAL;

	tspi->cur_bytes_per_word = bytes_per_word(bits_per_word);

	if (speed != tspi->cur_speed)
		clk_set_rate(tspi->clk, speed);

	if (tspi->cur_speed == 0)
		clk_enable(tspi->clk);

	tspi->cur_speed = speed;

	val = spi_readl(tspi, SLINK_COMMAND2);
	if (t->rx_buf)
		val |= RXEN;
	else
		val &= ~RXEN;

	if (t->tx_buf)
		val |= TXEN;
	else
		val &= ~TXEN;

	val &= ~SS_EN_CS(~0);
	val |= SS_EN_CS(spi->chip_select);
	val |= SPIE;

	spi_writel(tspi, val, SLINK_COMMAND2);

	val = spi_readl(tspi, SLINK_COMMAND);
	val &= ~BIT_LENGTH(~0);
	val |= BIT_LENGTH(bits_per_word - 1);

	/* FIXME: should probably control CS manually so that we can be sure
	 * it does not go low between transfer and to support delay_usecs
	 * correctly.
	 */
	val &= ~IDLE_SCLK_MASK & ~CK_SDA & ~CS_SW;

	if (spi->mode & SPI_CPHA)
		val |= CK_SDA;

	if (spi->mode & SPI_CPOL)
		val |= IDLE_SCLK_DRIVE_HIGH;
	else
		val |= IDLE_SCLK_DRIVE_LOW;

	spi_writel(tspi, val, SLINK_COMMAND);

	spi_writel(tspi, RX_FLUSH | TX_FLUSH, SLINK_STATUS);

	tspi->cur = t;
	tspi->cur_pos = 0;
	tspi->cur_len = spi_tegra_fill_tx_fifo(tspi, t);

	spi_tegra_go(tspi);

	return 0;
}

static void spi_tegra_start_message(struct spi_device *spi,
				    struct spi_message *m)
{
	struct spi_transfer *t;

	m->actual_length = 0;

	t = list_first_entry(&m->transfers, struct spi_transfer, transfer_list);
	spi_tegra_start_transfer(spi, t);
}

static void tegra_spi_rx_dma_complete(struct tegra_dma_req *req, int err)
{
	struct spi_tegra_data *tspi = req->dev;
	unsigned long flags;
	struct spi_message *m;
	struct spi_device *spi;
	int complete = 0;
	int timeout = 0;
	unsigned long val;

	/* the SPI controller may come back with both the BSY and RDY bits
	 * set.  In this case we need to wait for the BSY bit to clear so
	 * that we are sure the DMA is finished.
	 */
	while (timeout++ < BUSY_TIMEOUT) {
		if (!(spi_readl(tspi, SLINK_STATUS) & BSY))
			break;
	}

	val = spi_readl(tspi, SLINK_STATUS);
	val |= RDY;
	spi_writel(tspi, val, SLINK_STATUS);

	spin_lock_irqsave(&tspi->lock, flags);

	m = list_first_entry(&tspi->queue, struct spi_message, queue);
	spi = m->state;

	tspi->cur_pos += spi_tegra_drain_rx_fifo(tspi, tspi->cur);
	m->actual_length += tspi->cur_pos;

	if (tspi->cur_pos < tspi->cur->len) {
		tspi->cur_len = spi_tegra_fill_tx_fifo(tspi, tspi->cur);
		spi_tegra_go(tspi);
	} else if (!list_is_last(&tspi->cur->transfer_list,
				 &m->transfers)) {
		tspi->cur =  list_first_entry(&tspi->cur->transfer_list,
					      struct spi_transfer,
					      transfer_list);
		spi_tegra_start_transfer(spi, tspi->cur);
	} else {
		complete = 1;
	}

	if (complete) {
		list_del(&m->queue);

		m->status = 0;
		m->complete(m->context);

		if (!list_empty(&tspi->queue)) {
			m = list_first_entry(&tspi->queue, struct spi_message,
					     queue);
			spi = m->state;
			spi_tegra_start_message(spi, m);
		} else {
			clk_disable(tspi->clk);
			tspi->cur_speed = 0;
		}
	}

	spin_unlock_irqrestore(&tspi->lock, flags);
}

static int spi_tegra_setup(struct spi_device *spi)
{
	struct spi_tegra_data *tspi = spi_master_get_devdata(spi->master);
	unsigned long cs_bit;
	unsigned long val;
	unsigned long flags;

	dev_dbg(&spi->dev, "setup %d bpw, %scpol, %scpha, %dHz\n",
		spi->bits_per_word,
		spi->mode & SPI_CPOL ? "" : "~",
		spi->mode & SPI_CPHA ? "" : "~",
		spi->max_speed_hz);


	switch (spi->chip_select) {
	case 0:
		cs_bit = CS_POLARITY;
		break;

	case 1:
		cs_bit = CS_POLARITY1;
		break;

	case 2:
		cs_bit = CS_POLARITY2;
		break;

	case 4:
		cs_bit = CS_POLARITY3;
		break;

	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&tspi->lock, flags);

	val = spi_readl(tspi, SLINK_COMMAND);
	if (spi->mode & SPI_CS_HIGH)
		val |= cs_bit;
	else
		val &= ~cs_bit;
	spi_writel(tspi, val, SLINK_COMMAND);

	spin_unlock_irqrestore(&tspi->lock, flags);

	return 0;
}

static int spi_tegra_transfer(struct spi_device *spi, struct spi_message *m)
{
	struct spi_tegra_data *tspi = spi_master_get_devdata(spi->master);
	unsigned long flags;
	int was_empty;

	if (list_empty(&m->transfers) || !m->complete)
		return -EINVAL;

	m->state = spi;

	spin_lock_irqsave(&tspi->lock, flags);
	was_empty = list_empty(&tspi->queue);
	list_add_tail(&m->queue, &tspi->queue);

	if (was_empty)
		spi_tegra_start_message(spi, m);

	spin_unlock_irqrestore(&tspi->lock, flags);

	return 0;
}

static void spi_tegra_cleanup(struct spi_device *spi)
{
	dev_dbg(&spi->dev, "cleanup\n");
}

static int __init spi_tegra_probe(struct platform_device *pdev)
{
	struct spi_master	*master;
	struct spi_tegra_data	*tspi;
	struct resource		*r;
	int ret;

	master = spi_alloc_master(&pdev->dev, sizeof *tspi);
	if (master == NULL) {
		dev_err(&pdev->dev, "master allocation failed\n");
		return -ENOMEM;
	}

	/* the spi->mode bits understood by this driver: */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;

	if (pdev->id != -1)
		master->bus_num = pdev->id;

	master->setup = spi_tegra_setup;
	master->transfer = spi_tegra_transfer;
	master->cleanup = spi_tegra_cleanup;
	master->num_chipselect = 4;

	dev_set_drvdata(&pdev->dev, master);
	tspi = spi_master_get_devdata(master);
	tspi->master = master;
	tspi->pdev = pdev;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		ret = -ENODEV;
		goto err0;
	}

	if (!request_mem_region(r->start, (r->end - r->start) + 1,
			dev_name(&pdev->dev))) {
		ret = -EBUSY;
		goto err0;
	}

	tspi->phys = r->start;
	tspi->base = ioremap(r->start, r->end - r->start + 1);
	if (!tspi->base) {
		dev_err(&pdev->dev, "can't ioremap iomem\n");
		ret = -ENOMEM;
		goto err1;
	}

	tspi->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR_OR_NULL(tspi->clk)) {
		dev_err(&pdev->dev, "can not get clock\n");
		ret = PTR_ERR(tspi->clk);
		goto err2;
	}

	INIT_LIST_HEAD(&tspi->queue);

	tspi->rx_dma = tegra_dma_allocate_channel(TEGRA_DMA_MODE_ONESHOT);
	if (IS_ERR(tspi->rx_dma)) {
		dev_err(&pdev->dev, "can not allocate rx dma channel\n");
		ret = PTR_ERR(tspi->rx_dma);
		goto err3;
	}

	tspi->rx_bb = dma_alloc_coherent(&pdev->dev, sizeof(u32) * BB_LEN,
					 &tspi->rx_bb_phys, GFP_KERNEL);
	if (!tspi->rx_bb) {
		dev_err(&pdev->dev, "can not allocate rx bounce buffer\n");
		ret = -ENOMEM;
		goto err4;
	}

	tspi->rx_dma_req.complete = tegra_spi_rx_dma_complete;
	tspi->rx_dma_req.to_memory = 1;
	tspi->rx_dma_req.dest_addr = tspi->rx_bb_phys;
	tspi->rx_dma_req.dest_bus_width = 32;
	tspi->rx_dma_req.source_addr = tspi->phys + SLINK_RX_FIFO;
	tspi->rx_dma_req.source_bus_width = 32;
	tspi->rx_dma_req.source_wrap = 4;
	tspi->rx_dma_req.req_sel = spi_tegra_req_sels[pdev->id];
	tspi->rx_dma_req.dev = tspi;

	ret = spi_register_master(master);

	if (ret < 0)
		goto err5;

	return ret;

err5:
	dma_free_coherent(&pdev->dev, sizeof(u32) * BB_LEN,
			  tspi->rx_bb, tspi->rx_bb_phys);
err4:
	tegra_dma_free_channel(tspi->rx_dma);
err3:
	clk_put(tspi->clk);
err2:
	iounmap(tspi->base);
err1:
	release_mem_region(r->start, (r->end - r->start) + 1);
err0:
	spi_master_put(master);
	return ret;
}

static int __exit spi_tegra_remove(struct platform_device *pdev)
{
	struct spi_master	*master;
	struct spi_tegra_data	*tspi;
	struct resource		*r;

	master = dev_get_drvdata(&pdev->dev);
	tspi = spi_master_get_devdata(master);

	tegra_dma_free_channel(tspi->rx_dma);

	dma_free_coherent(&pdev->dev, sizeof(u32) * BB_LEN,
			  tspi->rx_bb, tspi->rx_bb_phys);

	clk_put(tspi->clk);
	iounmap(tspi->base);

	spi_master_put(master);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(r->start, (r->end - r->start) + 1);

	return 0;
}


MODULE_ALIAS("platform:" DRIVER_NAME);

static struct platform_driver spi_tegra_driver = {
	.driver = {
		.name =		DRIVER_NAME,
		.owner =	THIS_MODULE,
	},
	.remove =	__exit_p(spi_tegra_remove),
};


static int __init spi_tegra_init(void)
{
	return platform_driver_probe(&spi_tegra_driver, spi_tegra_probe);
}
subsys_initcall(spi_tegra_init);

static void __exit spi_tegra_exit(void)
{
	platform_driver_unregister(&spi_tegra_driver);
}
module_exit(spi_tegra_exit);

MODULE_LICENSE("GPL");
