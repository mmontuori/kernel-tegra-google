/*
 * drivers/video/tegrafb.c
 *
 * Copyright (C) 2009 Palm, Inc.
 * Author: Travis Geiselbrecht <travis@palm.com>
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

#include <linux/fb.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <asm/cacheflush.h>

#define NV_FB_TEST 0

#define DC_CMD_GENERAL_INCR_SYNCPT		0x000
#define DC_CMD_GENERAL_INCR_SYNCPT_CNTRL	0x001
#define DC_CMD_GENERAL_INCR_SYNCPT_ERROR	0x002
#define DC_CMD_WIN_A_INCR_SYNCPT		0x008
#define DC_CMD_WIN_A_INCR_SYNCPT_CNTRL		0x009
#define DC_CMD_WIN_A_INCR_SYNCPT_ERROR		0x00a
#define DC_CMD_WIN_B_INCR_SYNCPT		0x010
#define DC_CMD_WIN_B_INCR_SYNCPT_CNTRL		0x011
#define DC_CMD_WIN_B_INCR_SYNCPT_ERROR		0x012
#define DC_CMD_WIN_C_INCR_SYNCPT		0x018
#define DC_CMD_WIN_C_INCR_SYNCPT_CNTRL		0x019
#define DC_CMD_WIN_C_INCR_SYNCPT_ERROR		0x01a
#define DC_CMD_CONT_SYNCPT_VSYNC		0x028
#define DC_CMD_DISPLAY_COMMAND_OPTION0		0x031
#define DC_CMD_DISPLAY_COMMAND			0x032
#define DC_CMD_SIGNAL_RAISE			0x033
#define DC_CMD_INT_STATUS			0x037
#define DC_CMD_INT_MASK				0x038
#define DC_CMD_INT_ENABLE			0x039
#define DC_CMD_INT_TYPE				0x03a
#define DC_CMD_INT_POLARITY			0x03b
#define DC_CMD_SIGNAL_RAISE1			0x03c
#define DC_CMD_SIGNAL_RAISE2			0x03d
#define DC_CMD_SIGNAL_RAISE3			0x03e
#define DC_CMD_STATE_ACCESS			0x040
#define DC_CMD_STATE_CONTROL			0x041
#define DC_CMD_DISPLAY_WINDOW_HEADER		0x042
#define DC_CMD_REG_ACT_CONTROL			0x043
#define DC_WINC_A_COLOR_PALETTE			0x500
#define DC_WINC_A_PALETTE_COLOR_EXT		0x600
#define DC_WIN_A_WIN_OPTIONS			0x700
#define DC_WIN_A_BYTE_SWAP			0x701
#define DC_WIN_A_BUFFER_CONTROL			0x702
#define DC_WIN_A_COLOR_DEPTH			0x703
#define DC_WIN_A_POSITION			0x704
#define DC_WIN_A_SIZE				0x705
#define DC_WIN_A_PRESCALED_SIZE			0x706
#define DC_WIN_A_LINE_STRIDE			0x70a
#define DC_WIN_A_BUF_STRIDE			0x70b
#define DC_WINBUF_A_START_ADDR			0x800
#define DC_WINBUF_A_ADDR_H_OFFSET		0x806
#define DC_WINBUF_A_ADDR_V_OFFSET		0x808


#define DC_INT_CTXSW				(1<<0)
#define DC_INT_FRAME_END			(1<<1)
#define DC_INT_V_BLANK				(1<<2)
#define DC_INT_H_BLANK				(1<<3)
#define DC_INT_V_PULSE3				(1<<4)

struct tegrafb_info {
	struct clk *clk;
	struct resource *reg_mem;
	struct resource *fb_mem;
	void __iomem *reg_base;
	wait_queue_head_t frame_wq;
	spinlock_t lock;
	unsigned int wait_condition;
	int irq;
	u32 irq_mask;
};

static unsigned long s_fb_disable = 0;

#define tegrafb_writel( priv, reg, val ) writel((val), (priv)->reg_base + ((reg)*sizeof(u32)))
#define tegrafb_readl( priv, reg ) readl((priv)->reg_base + ((reg)*sizeof(u32)))

/* palette attary used by the fbcon */
u32 pseudo_palette[16];

/* fb_ops kernel interface */

int tegra_fb_open(struct fb_info *info, int user);
int tegra_fb_release(struct fb_info *info, int user);
int tegra_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info);
int tegra_fb_set_par(struct fb_info *info);
int tegra_fb_setcolreg(unsigned regno, unsigned red, unsigned green,
	unsigned blue, unsigned transp, struct fb_info *info);
int tegra_fb_blank(int blank, struct fb_info *info);
int tegra_fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info);
void tegra_fb_fillrect(struct fb_info *info, const struct fb_fillrect *rect);
void tegra_fb_copyarea(struct fb_info *info, const struct fb_copyarea *region);
void tegra_fb_imageblit(struct fb_info *info, const struct fb_image *image);
int tegra_fb_cursor(struct fb_info *info, struct fb_cursor *cursor);
int tegra_fb_sync(struct fb_info *info);

static struct fb_ops tegra_fb_ops = {
	.owner = THIS_MODULE,
	.fb_open = tegra_fb_open,
	.fb_release = tegra_fb_release,
	.fb_check_var = tegra_fb_check_var,
	.fb_set_par = tegra_fb_set_par,
	.fb_setcolreg = tegra_fb_setcolreg,
	.fb_pan_display = tegra_fb_pan_display,
	.fb_blank = tegra_fb_blank,
	.fb_fillrect = tegra_fb_fillrect,
	.fb_copyarea = tegra_fb_copyarea,
	.fb_imageblit =tegra_fb_imageblit,
	.fb_cursor = tegra_fb_cursor,
	.fb_sync = tegra_fb_sync,
};

irqreturn_t tegrafb_irq(int irq, void *ptr)
{
	struct fb_info *fb = ptr;
	struct tegrafb_info *tegrafb = fb->par;
	unsigned long flags;

	u32 reg = tegrafb_readl(tegrafb, DC_CMD_INT_STATUS);
	tegrafb_writel(tegrafb, DC_CMD_INT_STATUS, reg);

	spin_lock_irqsave(&tegrafb->lock, flags);
	tegrafb->wait_condition = 1;
	spin_unlock_irqrestore(&tegrafb->lock, flags);

	wake_up(&tegrafb->frame_wq);
	return IRQ_HANDLED;
}

static int wait_for_vsync(struct fb_info *fb, unsigned long timeout)
{
	struct tegrafb_info *tegrafb = fb->par;
	unsigned long flags;

	spin_lock_irqsave(&tegrafb->lock, flags);
	tegrafb->wait_condition = 0;
	spin_unlock_irqrestore(&tegrafb->lock, flags);

	tegrafb_writel(tegrafb, DC_CMD_INT_STATUS, tegrafb->irq_mask);
	tegrafb_writel(tegrafb, DC_CMD_INT_ENABLE, tegrafb->irq_mask);

	wait_event_interruptible_timeout(tegrafb->frame_wq, tegrafb->wait_condition, timeout);

	tegrafb_writel(tegrafb, DC_CMD_INT_ENABLE, 0);

	if (!tegrafb->wait_condition) {
		pr_warning("%s: wait for vsync timed out\n", __FUNCTION__);
		return 1;
	}
	return 0;
}

int tegra_fb_open(struct fb_info *info, int user)
{
	// printk( "tegra_fb_open\n");
	return 0;
}

int tegra_fb_release(struct fb_info *info, int user)
{
	// printk( "tegra_fb_release\n");
	return 0;
}

int tegra_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	//printk( "tegra_fb_check_var res: %dx%d virt: %dx%d bpp:%d "
	//	"offset: %d %d\n",
	//	var->xres, var->yres,
	//	var->xres_virtual, var->yres_virtual,
	//	var->bits_per_pixel,
	//	var->xoffset, var->yoffset
	//);

	return 0;
}

int tegra_fb_set_par(struct fb_info *info)
{
	//printk( "tegra_fb_set_par: %dx%d %dx%d\n", info->var.xres,
	//	info->var.yres, info->var.xres_virtual, info->var.yres_virtual );

	return 0;
}

int tegra_fb_setcolreg(unsigned regno, unsigned red, unsigned green,
	unsigned blue, unsigned transp, struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;

	if( info->fix.visual == FB_VISUAL_TRUECOLOR ||
		info->fix.visual == FB_VISUAL_DIRECTCOLOR ) {
		u32 v;

		if( regno >= 16 ) {
			return -EINVAL;
		}

		v = (red << var->red.offset) |
			(green << var->green.offset) |
			(blue << var->blue.offset);

		((u32 *)info->pseudo_palette)[regno] = v;

		//printk("Adding palette entry 0x%x at index %d\n", v, regno);
	}

	return 0;
}

int tegra_fb_blank(int blank, struct fb_info *info)
{
	return 0;
}

int tegra_fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct tegrafb_info *tegrafb = info->par;
	uint8_t *pFlushStart;
	uint8_t *pFlushEnd;
	u32 addr;
	int count = 0;

	if( s_fb_disable ) {
		return -1;
	}

	pFlushStart = info->screen_base + (var->yoffset * info->fix.line_length);
	pFlushEnd = pFlushStart + (var->yres * info->fix.line_length);
	dmac_clean_range(pFlushStart, pFlushEnd);

	//printk( "pan: %d %d, %d %d\n", var->xoffset, var->yoffset,
	//	info->var.xoffset, info->var.yoffset );

	info->var.xoffset = var->xoffset;
	info->var.yoffset = var->yoffset;

	addr = info->fix.smem_start + (var->yoffset * info->fix.line_length) +
		(var->xoffset * (var->bits_per_pixel/8));

	tegrafb_writel(tegrafb, DC_CMD_DISPLAY_WINDOW_HEADER, (1 << 4) );
	tegrafb_writel(tegrafb, DC_WINBUF_A_START_ADDR, addr );
	tegrafb_writel(tegrafb, DC_CMD_STATE_CONTROL, (1 << 8) | (1 << 9) );
	tegrafb_writel(tegrafb, DC_CMD_STATE_CONTROL, (1 << 0) | (1 << 1) );
	while (tegrafb_readl(tegrafb, DC_CMD_STATE_CONTROL) & 3) {
		wait_for_vsync(info, HZ/10);
		count++;
	}
	if (unlikely(count > 1))
		pr_warning("%s: waited for %d vsyncs\n", __FUNCTION__, count);

	return 0;
}

void tegra_fb_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
	cfb_fillrect( info, rect );
}

void tegra_fb_copyarea(struct fb_info *info, const struct fb_copyarea *region)
{
	cfb_copyarea( info, region );
}

void tegra_fb_imageblit(struct fb_info *info, const struct fb_image *image)
{
	//printk("tegra_fb_imageblit dx = %d  dy = %d width = %d height = %d "
	//	"color=%d\n", image->dx, image->dy, image->width, image->height,
	//	image->depth);

	cfb_imageblit( info, image );
}

int tegra_fb_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	//printk( "tegra_fb_cursor\n" );
	return 0;
}

int tegra_fb_sync(struct fb_info *info)
{
	//printk( "tegra_fb_sync\n" );
	return 0;
}

int tegra_fb_control(void *in, void *out)
{
	s_fb_disable = 1;
	return 0;
}

#define DUMP_REG(a) printk("%-32s\t%03x\t%08x\n", #a, a, tegrafb_readl(tegrafb, a));
static void dump_regs(struct tegrafb_info *tegrafb)
{
	DUMP_REG(DC_CMD_GENERAL_INCR_SYNCPT);
	DUMP_REG(DC_CMD_GENERAL_INCR_SYNCPT_CNTRL);
	DUMP_REG(DC_CMD_GENERAL_INCR_SYNCPT_ERROR);
	DUMP_REG(DC_CMD_WIN_A_INCR_SYNCPT);
	DUMP_REG(DC_CMD_WIN_A_INCR_SYNCPT_CNTRL);
	DUMP_REG(DC_CMD_WIN_A_INCR_SYNCPT_ERROR);
	DUMP_REG(DC_CMD_WIN_B_INCR_SYNCPT);
	DUMP_REG(DC_CMD_WIN_B_INCR_SYNCPT_CNTRL);
	DUMP_REG(DC_CMD_WIN_B_INCR_SYNCPT_ERROR);
	DUMP_REG(DC_CMD_WIN_C_INCR_SYNCPT);
	DUMP_REG(DC_CMD_WIN_C_INCR_SYNCPT_CNTRL);
	DUMP_REG(DC_CMD_WIN_C_INCR_SYNCPT_ERROR);
	DUMP_REG(DC_CMD_CONT_SYNCPT_VSYNC);
	DUMP_REG(DC_CMD_DISPLAY_COMMAND_OPTION0);
	DUMP_REG(DC_CMD_DISPLAY_COMMAND);
	DUMP_REG(DC_CMD_SIGNAL_RAISE);
	DUMP_REG(DC_CMD_INT_STATUS);
	DUMP_REG(DC_CMD_INT_MASK);
	DUMP_REG(DC_CMD_INT_ENABLE);
	DUMP_REG(DC_CMD_INT_TYPE);
	DUMP_REG(DC_CMD_INT_POLARITY);
	DUMP_REG(DC_CMD_SIGNAL_RAISE1);
	DUMP_REG(DC_CMD_SIGNAL_RAISE2);
	DUMP_REG(DC_CMD_SIGNAL_RAISE3);
	DUMP_REG(DC_CMD_STATE_ACCESS);
	DUMP_REG(DC_CMD_STATE_CONTROL);
	DUMP_REG(DC_CMD_DISPLAY_WINDOW_HEADER);
	DUMP_REG(DC_CMD_REG_ACT_CONTROL);
	tegrafb_writel(tegrafb, DC_CMD_DISPLAY_WINDOW_HEADER, (1 << 4) );
	DUMP_REG(DC_CMD_DISPLAY_WINDOW_HEADER);
	DUMP_REG(DC_WINC_A_COLOR_PALETTE);
	DUMP_REG(DC_WINC_A_PALETTE_COLOR_EXT);
	DUMP_REG(DC_WIN_A_WIN_OPTIONS);
	DUMP_REG(DC_WIN_A_BYTE_SWAP);
	DUMP_REG(DC_WIN_A_BUFFER_CONTROL);
	DUMP_REG(DC_WIN_A_COLOR_DEPTH);
	DUMP_REG(DC_WIN_A_POSITION);
	DUMP_REG(DC_WIN_A_SIZE);
	DUMP_REG(DC_WIN_A_PRESCALED_SIZE);
	DUMP_REG(DC_WIN_A_LINE_STRIDE);
	DUMP_REG(DC_WIN_A_BUF_STRIDE);
	DUMP_REG(DC_WINBUF_A_START_ADDR);
	DUMP_REG(DC_WINBUF_A_ADDR_H_OFFSET);
        DUMP_REG(DC_WINBUF_A_ADDR_V_OFFSET);
}

static int tegra_plat_probe( struct platform_device *pdev )
{
	struct fb_info *fb;
	struct tegrafb_info *tegrafb;
	struct clk *clk;
	struct resource	*res;
	struct resource *reg_mem;
	struct resource *fb_mem;
	int ret = 0;
	void __iomem *reg_base;
	void __iomem *fb_base;
	unsigned long fb_size;
	unsigned long fb_phys;
	int irq;

	fb = framebuffer_alloc(sizeof(struct tegrafb_info), &pdev->dev);
	if (!fb) {
		ret = -ENOMEM;
		goto err;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		pr_debug("%s: no irq\n", pdev->name);
		ret = -ENOENT;
		goto err_free;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_debug("%s: no mem resource\n", pdev->name);
		ret = -ENOENT;
		goto err_free;
	}

	reg_mem = request_mem_region(res->start, resource_size(res), pdev->name);
	if (!reg_mem) {
		pr_debug("%s: request_mem_region failed\n", pdev->name);
		ret = -EBUSY;
		goto err_free;
	}

	reg_base = ioremap(res->start, resource_size(res));
	if (!reg_base) {
		pr_debug("%s: registers can't be mapped\n", pdev->name);
		ret = -EBUSY;
		goto err_release_resource_reg;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		pr_debug("%s: no mem resource\n", pdev->name);
		ret = -ENOENT;
		goto err_iounmap_reg;
	}

	fb_mem = request_mem_region(res->start, resource_size(res), pdev->name);
	if (!fb_mem) {
		pr_debug("%s: request_mem_region failed\n", pdev->name);
		ret = -EBUSY;
		goto err_iounmap_reg;
	}

	fb_size = resource_size(res);
	fb_phys = res->start;
	fb_base = ioremap_cached(fb_phys, fb_size);
	if (!fb_base) {
		pr_debug("%s: fb can't be mapped\n", pdev->name);
		ret = -EBUSY;
		goto err_release_resource_fb;
	}

	clk = clk_get(&pdev->dev, NULL);
	if (!clk) {
		pr_debug("%s: can't get clock\n", pdev->name);
		ret = -ENOENT;
		goto err_iounmap_fb;
	}
	clk_enable(clk);

	tegrafb = fb->par;
	tegrafb->clk = clk;
	tegrafb->fb_mem = fb_mem;
	tegrafb->reg_mem = reg_mem;
	tegrafb->reg_base = reg_base;
	tegrafb->irq = irq;

	fb->fbops = &tegra_fb_ops;
	fb->pseudo_palette = pseudo_palette;
	fb->screen_base = fb_base;
	fb->screen_size = fb_size;

	strncpy(fb->fix.id, "tegrafb", 16);
	fb->fix.type = FB_TYPE_PACKED_PIXELS;
	fb->fix.visual = FB_VISUAL_TRUECOLOR;
	fb->fix.xpanstep   = 1;
	fb->fix.ypanstep   = 1;
	fb->fix.accel	  = FB_ACCEL_NONE;
	fb->fix.line_length = 1024 * 2;
	fb->fix.smem_start = fb_phys;
	fb->fix.smem_len = fb_size;

	fb->var.xres		= 1024;
	fb->var.yres		= 600;
	fb->var.xres_virtual   = 1024;
	fb->var.yres_virtual   = 1200;
	fb->var.bits_per_pixel = 16;
	fb->var.red.offset	= 11;
	fb->var.red.length	= 5;
	fb->var.red.msb_right	= 0;
	fb->var.green.offset	= 5;
	fb->var.green.length	= 6;
	fb->var.green.msb_right	= 0;
	fb->var.blue.offset	= 0;
	fb->var.blue.length	= 5;
	fb->var.blue.msb_right	= 0;
	fb->var.transp.offset	= 0;
	fb->var.transp.length	= 0;
	fb->var.transp.msb_right= 0;
	fb->var.activate	= FB_ACTIVATE_NOW;
	fb->var.height		= -1;
	fb->var.width		= -1;
	fb->var.pixclock	= 24500;
	fb->var.left_margin	= 0;
	fb->var.right_margin   = 0;
	fb->var.upper_margin   = 0;
	fb->var.lower_margin   = 0;
	fb->var.hsync_len	= 0;
	fb->var.vsync_len	= 0;
	fb->var.vmode		= FB_VMODE_NONINTERLACED;

	if (request_irq(irq, tegrafb_irq, IRQF_DISABLED,
			dev_name(&pdev->dev), fb)) {
		pr_debug("%s: request_irq %d failed\n",
			pdev->name, irq);
		ret = -EBUSY;
		goto err_clk_disable;
	}

	init_waitqueue_head(&tegrafb->frame_wq);
	spin_lock_init(&tegrafb->lock);

	tegrafb->irq_mask = DC_INT_FRAME_END;

	tegrafb_writel(tegrafb, DC_CMD_INT_MASK,
		tegrafb_readl(tegrafb, DC_CMD_INT_MASK) | tegrafb->irq_mask);

	printk( "tegrafb: base address: %08x (%08x)\n",
		(unsigned int)fb->fix.smem_start,
		(unsigned int)fb->screen_base );

	//dump_regs(tegrafb);

	register_framebuffer( fb );

	return 0;

err_clk_disable:
	clk_disable(clk);
err_iounmap_fb:
	iounmap(fb_base);
err_release_resource_fb:
	release_resource(fb_mem);
err_iounmap_reg:
	iounmap(reg_base);
err_release_resource_reg:
	release_resource(reg_mem);
err_free:
	framebuffer_release(fb);
err:
	return ret;
}

struct platform_driver tegra_platform_driver = {
	.probe = tegra_plat_probe,
	.driver = {
		.name = "tegrafb",
		.owner = THIS_MODULE,
	},
};

static int __init tegra_fb_init( void )
{
	int e;
	e = platform_driver_register( &tegra_platform_driver );
	if( e ) {
		printk( "nvtegrafb: platform_driver_register failed\n" );
		return e;
	}
	return e;
}

static void __exit tegra_exit( void )
{
	platform_driver_unregister(&tegra_platform_driver);
}
module_exit( tegra_exit );

module_init( tegra_fb_init );
