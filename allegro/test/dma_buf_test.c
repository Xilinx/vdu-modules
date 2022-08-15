/*
 * dma_buf_test.c driver used to do dma_buf tests
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
#include <linux/cdev.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/fcntl.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/stddef.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/dma-buf.h>

#include "al_alloc.h"
#include "al_alloc_ioctl.h"
#include "al_ioctl.h"

static int debug;
module_param(debug, int, 0644);
#define TEST_DMABUF_API        _IOWR('q', 20, struct al5_dma_info)
#define dprintk(level, fmt, arg ...)                            \
	do {                                                    \
		if (debug >= level) {                           \
			pr_debug("al5t: " fmt, ## arg)          \
		}                                               \
	} while (0)

static struct device *dev;
int al5t_major;
int al5t_num_devs = 10;
static struct cdev al5t_cdev;

static long al5t_compat_ioctl(struct file *file, unsigned int cmd,
			      unsigned long arg)
{
	long ret = -ENOIOCTLCMD;

	if (file->f_op->unlocked_ioctl)
		ret = file->f_op->unlocked_ioctl(file, cmd, arg);

	return ret;
}

static int al5t_open(struct inode *inode, struct file *filp)
{
	dprintk(1, "al5t: open\n");
	return 0;
}

static int al5t_release(struct inode *inode, struct file *filp)
{
	dprintk(1, "al5t: release\n");
	return 0;
}

int __test_dmabuf_api(struct device *dev, void *info)
{
	struct dma_buf *dbuf;
	struct al5_dma_info *user_info = info;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	char *vaddr;
	int i;
	int err = 0;
	int ret = 1;

	dbuf = dma_buf_get(user_info->fd);
	if (IS_ERR(dbuf)) {
		dprintk(1, "dbuf err %lx", (unsigned long)dbuf);
		return -EINVAL;
	}

	dprintk(1, "al5t: dbuf->size: %lx\n", (unsigned long)dbuf->size);

	attach = dma_buf_attach(dbuf, dev);
	if (IS_ERR(attach)) {
		dprintk(1, "attach err %lx", (unsigned long)attach);
		err = -EINVAL;
		goto fail_attach;
	}
	sgt = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		dprintk(1, "map err %lx", (unsigned long)sgt);
		err = -EINVAL;
		goto fail_map;
	}

	ret &= (sg_phys(sgt->sgl) == user_info->phy_addr);

	user_info->phy_addr = sg_dma_address(sgt->sgl);
	dprintk(1, "phy: %lx, virt: %lx\n", (unsigned long)sg_phys(sgt->sgl),
		(unsigned long)sg_virt(sgt->sgl));

	vaddr = dma_buf_kmap(dbuf, 0);
	for (i = 0; i < 5; ++i)
		ret &= (i == vaddr[i]);
	dma_buf_kunmap(dbuf, 0, vaddr);

	vaddr = dma_buf_vmap(dbuf);
	for (i = 0; i < 5; ++i)
		ret &= (i == vaddr[i]);
	dma_buf_vunmap(dbuf, vaddr);

	dma_buf_unmap_attachment(attach, sgt, DMA_BIDIRECTIONAL);

	if (!ret)
		err = -EINVAL;
fail_map:
	dma_buf_detach(dbuf, attach);
fail_attach:
	dma_buf_put(dbuf);
	return err;
}

int test_dmabuf_api(struct device *dev, unsigned long arg)
{
	struct al5_dma_info info;
	int err;

	if (copy_from_user(&info, (struct al5_dma_info *)arg, sizeof(info)))
		return -EFAULT;

	err = __test_dmabuf_api(dev, (void *)&info);
	if (err)
		return err;

	if (copy_to_user((void *)arg, &info, sizeof(info)))
		return -EFAULT;

	return 0;
}


static long al5t_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	dprintk(1, "al5t: ioctl\n");

	switch (cmd) {
	case GET_DMA_FD:
		dprintk(2, "GET_DMA_FD\n");
		return al5_ioctl_get_dma_fd(NULL, arg);
	case GET_DMA_PHY:
		dprintk(2, "GET_DMA_PHY\n");
		return al5_ioctl_get_dmabuf_dma_addr(dev, arg);
	case TEST_DMABUF_API:
		dprintk(2, "TEST_DMABUF_API\n");
		return test_dmabuf_api(dev, arg);
	default:
		return -EINVAL;
	}
}


const struct file_operations al5t_fops = {
	.owner		= THIS_MODULE,
	.open		= al5t_open,
	.release	= al5t_release,
	.unlocked_ioctl = al5t_ioctl,
	.compat_ioctl	= al5t_compat_ioctl,
};


int setup_chrdev_region(void)
{
	dev_t dev = 0;
	int err;

	if (al5t_major == 0) {
		err = alloc_chrdev_region(&dev, 0, al5t_num_devs, "al5t");
		al5t_major = MAJOR(dev);
		dprintk(2, "al5t: got major %d\n", al5t_major);

		if (err)
			return err;
	}
	return 0;
}


int al5t_probe(struct platform_device *pdev)
{
	int err, devno;

	dev = &pdev->dev;
	dprintk(1, "al5t: probe!\n");

	err = setup_chrdev_region();
	if (err) {
		dprintk(1, "setup_chrdev_region error:%d", err);
		return err;
	}
	devno = MKDEV(al5t_major, 0);
	cdev_init(&al5t_cdev, &al5t_fops);
	al5t_cdev.owner = THIS_MODULE;
	err = cdev_add(&al5t_cdev, devno, 1);
	if (err) {
		dprintk(1, "cdev_add error:%d", err);
		unregister_chrdev_region(devno, al5t_num_devs);
		return err;
	}
	dprintk(1, "al5t: minor is %d\n", MINOR(devno));
	return 0;
}

int al5t_remove(struct platform_device *pdev)
{
	dev_t devno = MKDEV(al5t_major, 0);

	dprintk(1, "al5t: remove\n");

	cdev_del(&al5t_cdev);
	unregister_chrdev_region(devno, al5t_num_devs);
	return 0;
}

static void al5t_pdev_release(struct device *dev)
{
}

static struct platform_device al5t_pdev = {
	.name		= "al5t",
	.dev.release	= al5t_pdev_release,
};

static struct platform_driver al5t_pdrv = {
	.probe		= al5t_probe,
	.remove		= al5t_remove,
	.driver		= {
		.name	= "al5t",
	},
};

static int __init al5t_init(void)
{
	int ret;

	dprintk(1, "al5t: init\n");

	ret = platform_device_register(&al5t_pdev);
	if (ret) {
		dprintk(1, "platform_device_register failed with %d\n", ret);
		return ret;
	}

	dprintk(1, "al5t: pdev is init\n");
	ret = platform_driver_register(&al5t_pdrv);
	if (ret) {
		dprintk(1, "platform_driver_register failed with %d\n", ret);
		platform_device_unregister(&al5t_pdev);
		return ret;
	}

	dprintk(1, "al5t: init retvalue: %d\n", ret);

	return ret;
}

static void __exit al5t_exit(void)
{
	dprintk(1, "al5t: exit\n");
	platform_driver_unregister(&al5t_pdrv);
	platform_device_unregister(&al5t_pdev);
}

module_init(al5t_init);
module_exit(al5t_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin Grandemange");
MODULE_DESCRIPTION("Test dmabuf interface");


