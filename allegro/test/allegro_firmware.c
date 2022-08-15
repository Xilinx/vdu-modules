/*
 * allegro_firmware.c driver used to do firmware tests
 *
 * Copyright (C) 2019, Allegro DVT (www.allegrodvt.com)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/debugfs.h>
#include <linux/fcntl.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define ALLEGRO_MCU_INTERRUPT_MASK 0x9104
#define ALLEGRO_MCU_INTERRUPT 0x9100
#define ALLEGRO_MCU_CLR 0x9108
#define ALLEGRO_MCU_STA 0x910c

#define ALLEGRO_FIRMWARE "altest.fw"
#define ALLEGRO_BOOTLOADER_FIRMWARE "altest_b.fw"
#define ALLEGRO_ICACHE_SIZE                 (1 << 15)   /* 32 kB */
#define ALLEGRO_DCACHE_SIZE                 (1 << 10)   /* 1  kB */
#define MCU_SRAM_SIZE                       (1 << 15)   /* 32 kB */

/* MCU Control Regs */
#define ALLEGRO_MCU_RESET                      0x9000
#define ALLEGRO_MCU_RESET_MODE                 0x9004
#define ALLEGRO_MCU_WAKEUP                     0x900c
#define ALLEGRO_ICACHE_ADDR_OFFSET_MSB         0x9010
#define ALLEGRO_ICACHE_ADDR_OFFSET_LSB         0x9014
#define ALLEGRO_DCACHE_ADDR_OFFSET_MSB         0x9018
#define ALLEGRO_DCACHE_ADDR_OFFSET_LSB         0x901c

#define MAILBOX_STATUS                         0x7c00

#define allegro_writel(val, reg) iowrite32(val, codec->regs + reg)
#define allegro_readl(reg) ioread32(codec->regs + reg)

struct allegro_dma_buffer {
	unsigned long size;
	dma_addr_t dma_handle;
	void *cpu_handle;
};

struct allegro_codec_desc {
	struct device *device;

	void __iomem *regs;     /* Base addr for regs of the MCU */
	unsigned long regs_size;

	struct allegro_dma_buffer *icache;
	struct allegro_dma_buffer *dcache;

	struct dentry *firm_debug_dir;
};

static void allegro_free_dma_buffers(struct allegro_codec_desc *codec)
{
	dma_free_coherent(codec->device, codec->icache->size,
			  codec->icache->cpu_handle, codec->icache->dma_handle);
	dma_free_coherent(codec->device, codec->dcache->size,
			  codec->dcache->cpu_handle, codec->dcache->dma_handle);
}

static int allegro_alloc_dma_buffers(struct allegro_codec_desc *codec)
{
	int ret;
	char *dcache_addr;

	dev_info(codec->device, "allocating icache struct\n");

	/* alloc the icache and the dcache */
	codec->icache = devm_kzalloc(codec->device,
				     sizeof(struct allegro_dma_buffer),
				     GFP_KERNEL);
	if (!codec->icache) {
		ret = -ENOMEM;
		goto fail_icache;
	}

	codec->dcache = devm_kzalloc(codec->device,
				     sizeof(struct allegro_dma_buffer),
				     GFP_KERNEL);
	if (!codec->dcache) {
		ret = -ENOMEM;
		goto fail_dcache;
	}

	dev_info(codec->device, "allocating dma buffer for icache");

	codec->icache->cpu_handle =
		dma_alloc_coherent(codec->device,
				   ALLEGRO_ICACHE_SIZE,
				   &codec->icache->dma_handle,
				   GFP_KERNEL | GFP_DMA);
	if (!codec->icache->cpu_handle) {
		ret = -ENOMEM;
		goto fail_alloc_dma_icache;
	}
	codec->icache->size = ALLEGRO_ICACHE_SIZE;

	codec->dcache->cpu_handle =
		dma_alloc_coherent(codec->device,
				   ALLEGRO_DCACHE_SIZE,
				   &codec->dcache->dma_handle,
				   GFP_KERNEL | GFP_DMA);
	if (!codec->dcache->cpu_handle) {
		ret = -ENOMEM;
		goto fail_alloc_dma_dcache;
	}
	codec->dcache->size = ALLEGRO_DCACHE_SIZE;

	dcache_addr = codec->dcache->cpu_handle;

	dcache_addr[0] = 1;
	dcache_addr[4] = 2;
	dcache_addr[8] = 3;
	dcache_addr[12] = 4;

	dev_info(codec->device, "dcache phy is at %p",
		 (void *)codec->dcache->dma_handle);
	dev_info(codec->device, "icache phy is at %p",
		 (void *)codec->icache->dma_handle);

	return 0;

fail_alloc_dma_dcache:
	dma_free_coherent(codec->device, codec->icache->size,
			  codec->icache->cpu_handle, codec->icache->dma_handle);
fail_alloc_dma_icache:
	devm_kfree(codec->device, codec->dcache);
fail_dcache:
	devm_kfree(codec->device, codec->icache);
fail_icache:
	return ret;
}

static void set_icache_offset(struct allegro_codec_desc *codec)
{
	u32 icache_offset_msb;

	/* XXX walkaround mb cache: addr - 0x80000000 */
	allegro_writel((u32)codec->icache->dma_handle - 0x80000000,
		       ALLEGRO_ICACHE_ADDR_OFFSET_LSB);
	icache_offset_msb = (sizeof(codec->icache->dma_handle) == 4) ? 0
			    : codec->icache->dma_handle >> 32;
	allegro_writel(icache_offset_msb, ALLEGRO_ICACHE_ADDR_OFFSET_MSB);
}

static void set_dcache_offset(struct allegro_codec_desc *codec)
{
	u32 dcache_offset_msb;
	unsigned long msb = 0;

	/* XXX walkaround mb cache: addr - 0x8000000 */
	if (codec->dcache->dma_handle > 0x80000000)
		msb = 0xffffffff;
	allegro_writel((u32)codec->dcache->dma_handle - 0x80000000,
		       ALLEGRO_DCACHE_ADDR_OFFSET_LSB);
	dcache_offset_msb = (sizeof(codec->dcache->dma_handle) == 4) ? msb
			    : codec->dcache->dma_handle >>  32;
	allegro_writel(dcache_offset_msb, ALLEGRO_DCACHE_ADDR_OFFSET_MSB);
}

static int copy_firmware(struct allegro_codec_desc *codec,
			 const struct firmware *fw,
			 const struct firmware *bl_fw)
{
	int i;
	unsigned long sram_value, sram_true_value;

	if (fw->size > ALLEGRO_ICACHE_SIZE) {
		dev_err(codec->device, "firmware is too big\n");
		return -EINVAL;
	}

	if (bl_fw->size > MCU_SRAM_SIZE) {
		dev_err(codec->device, "bootloader firmware is too big\n");
		return -EINVAL;
	}

	dev_info(codec->device, "copy firmware in icache\n");
	dev_info(codec->device, "firmware size is %zx", fw->size);

	memcpy(codec->icache->cpu_handle, fw->data, fw->size);

	dev_info(codec->device, "copy bootloader firmware in sram\n");
	dev_info(codec->device, "bootloader firmware size is %zx", bl_fw->size);

	for (i = 0; i < bl_fw->size / 4; ++i) {
		sram_true_value = *((u32 *)bl_fw->data + i);
		iowrite32(sram_true_value, codec->regs + 4 * i);
	}

	for (i = 0; i < bl_fw->size / 4; ++i) {
		sram_value = ioread32((__u32 *)codec->regs + i);
		sram_true_value = *((__u32 *)bl_fw->data + i);
		if (sram_value != sram_true_value) {
			dev_alert(codec->device,
				  "Corruption while loading the bootloader firmware");
			dev_alert(codec->device,
				  "sram %x:%02lx should be %02lx", i,
				  sram_value, sram_true_value);
			break;
		}
	}

	return 0;

}

static void reset_mcu(struct allegro_codec_desc *codec)
{
	dev_info(codec->device, "reset MCU\n");

	/* Mode 1 Reset MCU (sleep) */
	allegro_writel(1, ALLEGRO_MCU_RESET_MODE);
	allegro_writel(0, ALLEGRO_MCU_WAKEUP);

	/* Reset MCU */
	allegro_writel(1, ALLEGRO_MCU_RESET);
}

static void start_mcu(struct allegro_codec_desc *codec)
{
	allegro_writel(1, ALLEGRO_MCU_WAKEUP);
}

static void set_mcu_interrupt_mask(struct allegro_codec_desc *codec)
{
	allegro_writel(1, ALLEGRO_MCU_INTERRUPT_MASK);
}

static int allegro_start_ip(struct allegro_codec_desc *codec,
			    const struct firmware *fw,
			    const struct firmware *bl_fw)
{
	int ret;

	reset_mcu(codec);

	ret = copy_firmware(codec, fw, bl_fw);
	if (ret)
		return ret;

	set_icache_offset(codec);
	set_dcache_offset(codec);
	set_mcu_interrupt_mask(codec);

	return 0;
}

irqreturn_t allegro_codec_hardirq_handler(int irq, void *data)
{
	struct allegro_codec_desc *codec = (struct allegro_codec_desc *)data;

	/* ack interrupt */
	allegro_writel(1, ALLEGRO_MCU_CLR);

	return IRQ_WAKE_THREAD;
}

static void read_test_status(struct allegro_codec_desc *codec)
{

	u32 status = allegro_readl(MAILBOX_STATUS);

	dev_info(codec->device, "Test status is %x\n", status);
}

irqreturn_t allegro_codec_irq_handler(int irq, void *data)
{
	struct allegro_codec_desc *codec = (struct allegro_codec_desc *)data;

	read_test_status(codec);
	dev_info(codec->device, "Received interrupt\n");
	return IRQ_HANDLED;
}

ssize_t launch_write(struct file *filp, const char __user *buf,
		     size_t count, loff_t *offp)
{
	struct allegro_codec_desc *codec = (struct allegro_codec_desc *)
					   filp->f_inode->i_private;

	start_mcu(codec);
	return count;
}

const struct file_operations launch_fops = {
	.write	= launch_write,
};

int allegro_codec_probe(struct platform_device *pdev)
{
	struct resource *res;
	int err;
	int irq;
	struct allegro_codec_desc *codec
		= devm_kzalloc(&pdev->dev,
			       sizeof(struct allegro_codec_desc),
			       GFP_KERNEL);

	const struct firmware *fw = NULL;
	const struct firmware *bl_fw = NULL;
	char *fw_file = ALLEGRO_FIRMWARE;
	char *bl_fw_file = ALLEGRO_BOOTLOADER_FIRMWARE;


	if (!codec) {
		dev_err(&pdev->dev, "codec couldn't be initialized\n");
		return -ENOMEM;
	}

	codec->device = &pdev->dev;
	dev_info(codec->device, "PROBING ALLEGRO FIRMWARE\n");

	err = request_firmware(&fw, fw_file, codec->device);
	if (err) {
		dev_err(&pdev->dev, "firmware file '%s' not found\n", fw_file);
		goto out_failed_firmware;
	}

	dev_info(&pdev->dev, "downloading firmware from file '%s'\n", fw_file);

	err = request_firmware(&bl_fw, bl_fw_file, codec->device);
	if (err) {
		dev_err(&pdev->dev,
			"bootloader firmware file '%s' not found\n",
			bl_fw_file);
		goto out_failed_firmware;
	}

	dev_info(&pdev->dev,
		 "downloading firmware from file '%s'\n", bl_fw_file);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Can't get resource\n");
		err = -ENODEV;
		goto out_no_resource;
	}

	codec->regs = devm_ioremap_resource(&pdev->dev, res);
	codec->regs_size = res->end - res->start;

	if (IS_ERR(codec->regs)) {
		dev_err(&pdev->dev, "Can't map registers\n");
		err = PTR_ERR(codec->regs);
		goto out_map_register;
	}

	err = allegro_alloc_dma_buffers(codec);
	if (err) {
		dev_err(&pdev->dev, "icache failed to be allocated\n");
		goto out_map_register;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to get IRQ\n");
		err = irq;
		goto out_no_irq;
	}

	enable_irq(irq);

	err = devm_request_threaded_irq(codec->device,
					irq,
					allegro_codec_hardirq_handler,
					allegro_codec_irq_handler,
					IRQF_SHARED, "firm_test", codec);
	if (err) {
		dev_err(&pdev->dev, "Failed to request IRQ #%d -> :%d\n",
			irq, err);
		goto out_failed_request_irq;
	}

	err = allegro_start_ip(codec, fw, bl_fw);
	if (err) {
		dev_err(&pdev->dev, "failed to start microblaze\n");
		goto out_map_register;
	}

	platform_set_drvdata(pdev, codec);

	dev_info(codec->device, "Probed Allegro Codec board\n");

	release_firmware(fw);
	release_firmware(bl_fw);
	fw = NULL;
	bl_fw = NULL;

	codec->firm_debug_dir = debugfs_create_dir("mcu_firmware_test", NULL);
	debugfs_create_file("launch", 0777, codec->firm_debug_dir,
			    codec, &launch_fops);

	dev_info(codec->device, "END ALLEGRO FIRMWARE\n\n");

	return 0;

out_map_register:
out_failed_request_irq:
out_no_irq:
out_no_resource:
	release_firmware(fw);
out_failed_firmware:
	devm_kfree(&pdev->dev, codec);
	return err;

}

int allegro_codec_remove(struct platform_device *pdev)
{
	struct allegro_codec_desc *codec = platform_get_drvdata(pdev);

	allegro_free_dma_buffers(codec);
	debugfs_remove_recursive(codec->firm_debug_dir);
	return 0;
}

static const struct of_device_id allegro_codec_of_match[] = {
	{ .compatible = "allegroIP" },
	{ /* sentinel */ },
};

static struct platform_driver allegro_platform_driver = {
	.driver			=	{
		.name		= "allegroIP",
		.of_match_table = of_match_ptr(allegro_codec_of_match),
	},
	.probe			= allegro_codec_probe,
	.remove			= allegro_codec_remove,
};

module_platform_driver(allegro_platform_driver);

MODULE_FIRMWARE(ALLEGRO_FIRMWARE);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin Grandemange");
MODULE_AUTHOR("Sebastien Alaiwan");
MODULE_DESCRIPTION("Allegro IP driver");
