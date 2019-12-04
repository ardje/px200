/*
 * e2c PX Series PCIe Driver
 *
 * Copyright (C) 2014, e2c Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * This driver registers a misc device that exports ioctl() and mmap() access
 * for controlling px series logic and mapping the SRAM into userspace. It also
 * registers the px200 UARTs.
 */

/* comment out to disable debug statements - i.e. dev_dbg() */
#define DEBUG

#include <linux/pci.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>
#include <linux/serial_8250.h>
//#include "8250.h"

/* PCI ID */
#define PCI_VENDOR_ID_E2C		0x1836
#define PCI_DEVICE_ID_PX200		0x001B /* PX200 */
#define PCI_DEVICE_ID_PX400		0x001D /* PX400 */


/* driver minor device number */
#define E2CPX_MINOR	103

/* driver version */
#define E2CPX_VERSION	"0.6"

/* private driver data - add new driver variables here */
struct e2c_px_priv {
	/* can be used to access BAR IO space on 32 bit boundary */
	void __iomem *pci_cfg;
	void __iomem *sram;
	void __iomem *uart;

	/* runtime data */
	struct pci_dev *pdev;
	struct device *dev;
	
	/* uart line data */
	int uart_line[5];
};
static struct e2c_px_priv *px;

#define PX_UART0_BASE	0x0000
#define PX_UART1_BASE	0x0020
#define PX_UART2_BASE	0x0040
#define PX_UART3_BASE	0x0060
#define PX_UART4_BASE	0x0080

static int e2c_init_serial_port(struct e2c_px_priv *e2c_px, int line,
	u32 base)
{
	struct uart_port serial_port;
	struct pci_dev *pdev = e2c_px->pdev;
	struct uart_8250_port port8250;

	/* TODO: these values may need fined tuned to Altera datasheet values */
	/* create UART config for Altera UARTs */
	memset(&port8250, 0, sizeof(port8250));
	memset(&serial_port, 0, sizeof(serial_port));
	serial_port.iotype = UPIO_MEM32; /* registers on 32 bit boundary ? */
	serial_port.type = PORT_16550A; /* TODO : tune FIFO size */
	serial_port.uartclk = 50000000;
	serial_port.fifosize = 16;
	serial_port.flags = UPF_SHARE_IRQ | UPF_BOOT_AUTOCONF | UPF_IOREMAP
		| UPF_FIXED_PORT | UPF_FIXED_TYPE;
	serial_port.regshift = 2;

	/* register UART */
	serial_port.membase = e2c_px->uart + base;
	serial_port.mapbase = pci_resource_start(pdev, 2) + base;
	serial_port.irq	 = pdev->irq;
	serial_port.line = line;
	port8250.port = serial_port;
	e2c_px->uart_line[line] = serial8250_register_8250_port(&port8250);
	if (e2c_px->uart_line[line] < 0)
		dev_err(&pdev->dev, "error: failed to register uart%d %d\n",
			line, e2c_px->uart_line[line]);

	return e2c_px->uart_line[line];
}

static int e2c_px_serial_init(struct e2c_px_priv *e2c_px)
{	
	int ret;

	ret = e2c_init_serial_port(e2c_px, 0, PX_UART0_BASE);
	if (ret < 0)
		goto err_uart0;

	ret = e2c_init_serial_port(e2c_px, 1, PX_UART1_BASE);
	if (ret < 0)
		goto err_uart1;

	ret = e2c_init_serial_port(e2c_px, 2, PX_UART2_BASE);
	if (ret < 0)
		goto err_uart2;

	ret = e2c_init_serial_port(e2c_px, 3, PX_UART3_BASE);
	if (ret < 0)
		goto err_uart3;

	ret = e2c_init_serial_port(e2c_px, 4, PX_UART4_BASE);
	if (ret < 0)
		goto err_uart4;

	return 0;

err_uart4:
	serial8250_unregister_port(e2c_px->uart_line[4]);
err_uart3:
	serial8250_unregister_port(e2c_px->uart_line[3]);
err_uart2:
	serial8250_unregister_port(e2c_px->uart_line[2]);
err_uart1:
	serial8250_unregister_port(e2c_px->uart_line[1]);
err_uart0:
	serial8250_unregister_port(e2c_px->uart_line[0]);
	return ret;
}

static int e2c_px_open(struct inode *inode, struct file *file)
{
	struct e2c_px_priv *e2c_px = px;
	struct device *dev;

	file->private_data = e2c_px;
	dev = e2c_px->dev;

	dev_dbg(dev, "file open\n");

	return 0;
}

static int e2c_px_release(struct inode *inode, struct file *file)
{
	struct e2c_px_priv *e2c_px = file->private_data;
	struct device *dev = e2c_px->dev;

	dev_dbg(dev, "file release\n");

	file->private_data = NULL;
	return 0;
}

/* ioctl handler for any userspace comand/control */
static long e2c_px_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	struct e2c_px_priv *e2c_px = file->private_data;
	struct device *dev = e2c_px->dev;
	int err = 0;

	dev_dbg(dev, "ioctl cmd %d\n", cmd);

	return err;
}

/* memory map the SRAM into userspace */
static int e2c_px_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct e2c_px_priv *e2c_px = file->private_data;
	struct pci_dev *pdev = e2c_px->pdev;
	struct device *dev = e2c_px->dev;
	unsigned long addr = pci_resource_start(pdev, 1);

	dev_dbg(dev, "mmap addr 0x%lx\n", addr);

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	return vm_iomap_memory(vma, addr, PAGE_SIZE);
}

static const struct file_operations e2c_px_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= e2c_px_ioctl,
	.open		= e2c_px_open,
	.release	= e2c_px_release,
	.mmap		= e2c_px_mmap,
};

static struct miscdevice e2c_px_misc = { E2CPX_MINOR, "nvram", &e2c_px_fops };


/* Intitialise the px */
static int e2c_px_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct e2c_px_priv *e2c_px;
	int ret;
	printk("Starting PX Probe\n");
	/* allocate memory for driver private data */
	e2c_px = devm_kzalloc(&pdev->dev, sizeof(*e2c_px), GFP_KERNEL);
	if (e2c_px == NULL)
		return -ENOMEM;
	pci_set_drvdata(pdev, e2c_px);
	e2c_px->pdev = pdev;
	e2c_px->dev = &pdev->dev;
	px = e2c_px;

	/* enable the px300/200 PCI device */
	printk("Enabling PCIe Device\n");
	ret = pci_enable_device(pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't enable device\n");
		return ret;
	}

	/* Map the PCI config space */
	printk("Map Config Space\n");
	e2c_px->pci_cfg = pci_ioremap_bar(pdev, 0);
	if (!e2c_px->pci_cfg) {
		ret = -ENODEV;
		goto pci_err;
	}

	/* Map SRAM config space */
	printk("Map SRAM\n");
	e2c_px->sram = pci_ioremap_bar(pdev, 1);
	if (!e2c_px->sram) {
		ret = -ENODEV;
		goto sram_err;
	}

	/* Map UART and I2C config space */
	printk("Map UART\n");
	e2c_px->uart = pci_ioremap_bar(pdev, 2);
	if (!e2c_px->uart) {
		ret = -ENODEV;
		goto uart_err;
	}

	/* register a misc device for userspace IO */
	ret = misc_register(&e2c_px_misc);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't register device %d\n", ret);
		goto region_err;
	}

	/* register UARTS */
	ret = e2c_px_serial_init(e2c_px);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't register UARTs %d\n", ret);
		goto uart_reg_err;
	}

	/* success */
	dev_info(&pdev->dev, "e2c PX driver version %s\n", E2CPX_VERSION);
	dev_info(&pdev->dev, "using uart lines ttyS%d,%d,%d,%d,%d\n",
		e2c_px->uart_line[0], e2c_px->uart_line[1],
		e2c_px->uart_line[2],e2c_px->uart_line[3],e2c_px->uart_line[4]);
	return 0;

	/* cleanup after any errors */
uart_reg_err:
	misc_deregister(&e2c_px_misc);
pci_err:
	pci_disable_device(pdev);
region_err:
	iounmap(e2c_px->uart);
uart_err:
	iounmap(e2c_px->sram);
sram_err:
	iounmap(e2c_px->pci_cfg);
	return ret;
}

/* Free all px driver resources */
static void e2c_px_remove(struct pci_dev *pdev)
{
	struct e2c_px_priv *e2c_px = pci_get_drvdata(pdev);
	
	misc_deregister(&e2c_px_misc);

	serial8250_unregister_port(e2c_px->uart_line[0]);
	serial8250_unregister_port(e2c_px->uart_line[1]);
	serial8250_unregister_port(e2c_px->uart_line[2]);
	serial8250_unregister_port(e2c_px->uart_line[3]);
	serial8250_unregister_port(e2c_px->uart_line[4]);

	iounmap(e2c_px->uart);
	iounmap(e2c_px->sram);
	iounmap(e2c_px->pci_cfg);

	pci_disable_device(pdev);
	px = NULL;
}

/* List of PCI IDs this driver supports */
static struct pci_device_id e2c_px_id_table[] = {
	{PCI_VENDOR_ID_E2C, PCI_DEVICE_ID_PX200, PCI_ANY_ID,PCI_ANY_ID, 0, 0},
	{PCI_VENDOR_ID_E2C, PCI_DEVICE_ID_PX400, PCI_ANY_ID,PCI_ANY_ID, 0, 0},
	{0}
};
MODULE_DEVICE_TABLE(pci, e2c_px_id_table);

static struct pci_driver e2c_px_driver = {
	.name		= "e2c_px",
	.id_table	= e2c_px_id_table,
	.probe		= e2c_px_probe,
	.remove		= e2c_px_remove,
};
module_pci_driver(e2c_px_driver);

MODULE_AUTHOR("Ian Craig <ian@etwoc.com>");
MODULE_DESCRIPTION("PX");
MODULE_LICENSE("GPL v2");


