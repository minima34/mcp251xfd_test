#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>

#include <asm/unaligned.h>
// by Dv: compatibility
#include <linux/version.h>
// by Dv: compatibility /

#include <linux/iopoll.h>

#include "mcp251xfd.h"
#define DEVICE_NAME "mcptest"

#define TID_CAN_CLK  20000000
#define TID_SPI_FREQ 4000000

#define tid_poll_timeout(cond, sleep_us, timeout_us, sleep_before_read, op, args...) \
({ \
	ktime_t timeout = ktime_add_us(ktime_get(), timeout_us); \
	might_sleep_if(sleep_us); \
	if(sleep_before_read) \
		usleep_range((sleep_us >> 2) + 1, sleep_us); \
	for (;;) { \
		op(args); \
		if (cond) \
			break; \
		if (timeout_us && ktime_compare(ktime_get(), timeout) > 0) { \
			op(args); \
			break; \
		} \
		if (sleep_us) \
			usleep_range((sleep_us >> 2) + 1, sleep_us); \
	} \
	(cond) ? 0 : -ETIMEDOUT; \
})

#define my_regmap_read_poll_timeout(cond, sleep_us, timeout_us, op, spi, addr, val) \
({ \
	int __ret, __tmp; \
	__tmp = tid_poll_timeout(val || (cond), sleep_us, timeout_us, false, op, spi, addr, &(val)); \
	__ret ?: __tmp; \
})

static const struct mcp251xfd_devtype_data mcp251xfd_devtype_data_mcp2517fd = {
	.quirks = MCP251XFD_QUIRK_MAB_NO_WARN | MCP251XFD_QUIRK_CRC_REG |
		MCP251XFD_QUIRK_CRC_RX | MCP251XFD_QUIRK_CRC_TX |
		MCP251XFD_QUIRK_ECC,
	.model = MCP251XFD_MODEL_MCP2517FD,
};

static const struct mcp251xfd_devtype_data mcp251xfd_devtype_data_mcp2518fd = {
	.quirks = MCP251XFD_QUIRK_CRC_REG | MCP251XFD_QUIRK_CRC_RX |
		MCP251XFD_QUIRK_CRC_TX | MCP251XFD_QUIRK_ECC,
	.model = MCP251XFD_MODEL_MCP2518FD,
};

/* Autodetect model, start with CRC enabled. */
static const struct mcp251xfd_devtype_data mcp251xfd_devtype_data_mcp251xfd = {
	.quirks = MCP251XFD_QUIRK_CRC_REG | MCP251XFD_QUIRK_CRC_RX |
		MCP251XFD_QUIRK_CRC_TX | MCP251XFD_QUIRK_ECC,
	.model = MCP251XFD_MODEL_MCP251XFD,
};

u16 rx_reg;
u32 rx_val;

static int my_regmap_write(struct spi_device *spi, u16 reg, u32 val)
{
	struct spi_transfer xfer[2];

	//uint8_t *reg_bytes[2] = (void *) &reg;
	//uint8_t *val_bytes[4] = (void *) &val;

	reg|=MCP251XFD_SPI_INSTRUCTION_WRITE;

	xfer[0].tx_buf = &reg;
	xfer[0].rx_buf = &rx_reg;
	xfer[0].len = 2;

	xfer[1].tx_buf = &val;
	xfer[1].rx_buf = &rx_val;
	xfer[1].len = 4;

	printk(KERN_NOTICE "%s(%d) writing 0x%04x => 0x%08xx\nrx_reg => 0x%04x", __FUNCTION__, __LINE__, reg, val, rx_reg);

	return spi_sync_transfer(spi, xfer, ARRAY_SIZE(xfer));
}

static int my_regmap_read(struct spi_device *spi, u16 reg, u32 *val)
{
	struct spi_transfer xfer[2];

	reg|=MCP251XFD_SPI_INSTRUCTION_READ;

	xfer[0].tx_buf = &reg;
	xfer[0].rx_buf = &rx_reg;
	xfer[0].len = 2;

	u32 klokvam_nuli = 0x00000000;

	xfer[1].tx_buf = &klokvam_nuli;
	xfer[1].rx_buf = val;
	xfer[1].len = 4;

	printk(KERN_NOTICE "%s(%d) reading 0x%04x => 0x%08xx\nrx_reg => 0x%04x", __FUNCTION__, __LINE__, reg, *val, rx_reg);

	return spi_sync_transfer(spi, xfer, ARRAY_SIZE(xfer));
}

static int vikam_te(struct spi_device *spi)
{
	u32 osc, osc_reference, osc_mask;

	/* Set Power On Defaults for "Clock Output Divisor" and remove
	 * "Oscillator Disable" bit.
	 */
	printk (KERN_NOTICE "mask=0x%08x; clkdiv10=0x%08x;\n",MCP251XFD_REG_OSC_CLKODIV_MASK,MCP251XFD_REG_OSC_CLKODIV_10);
	osc = FIELD_PREP(MCP251XFD_REG_OSC_CLKODIV_MASK, MCP251XFD_REG_OSC_CLKODIV_10);

	
	osc_reference = MCP251XFD_REG_OSC_OSCRDY;
	osc_mask = MCP251XFD_REG_OSC_OSCRDY;
	//osc_mask = MCP251XFD_REG_OSC_OSCRDY | MCP251XFD_REG_OSC_PLLRDY;

	/* Note:
	 *
	 * If the controller is in Sleep Mode the following write only
	 * removes the "Oscillator Disable" bit and powers it up. All
	 * other bits are unaffected.
	 */

	my_regmap_write(spi, MCP251XFD_REG_OSC, 0x00000060);

	printk(KERN_NOTICE, "%s(%d) w1 osc=0x%08x\n", __FUNCTION__, __LINE__, osc);
	pr_info("[WR] rx_reg = 0x%04X \nrx_val = 0x%08X \n", rx_reg, rx_val);


	//(cond, sleep_us, timeout_us, op, spi, addr, val)
	int err = my_regmap_read_poll_timeout((osc & osc_mask) == osc_reference, MCP251XFD_OSC_STAB_SLEEP_US, MCP251XFD_OSC_STAB_TIMEOUT_US, my_regmap_read, spi, MCP251XFD_REG_OSC, osc);
	
	printk(KERN_NOTICE, "%s(%d) r0 osc=0x%08x\n", __FUNCTION__, __LINE__, osc);
	pr_info("[RD] rx_reg = 0x%04X \nrx_val = 0x%08X \n", rx_reg, rx_val);


	return err;
}

static int mcptest_probe(struct spi_device *spi)
{
	int err;

	struct mcp251xfd_priv *priv;
	u32 freq;
	
	freq = TID_CAN_CLK;
	
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5,9,0))
	if(!spi->irq)
	{
		return dev_err_probe(&spi->dev, -ENXIO, "No IRQ specified (maybe node \"interrupts-extended\" in DT missing)!\n");
	}
#else
	if(!spi->irq)
	{
		dev_err(&spi->dev, "No IRQ specified (maybe node \"interrupts-extended\" in DT missing)!\n");
		return -ENXIO;
	}
#endif

	//struct device_node *np;
	//np = spi->dev.of_node;

	//spi_set_drvdata(spi, priv);
	//priv->spi = spi;
	//priv->rx_int = rx_int;
	//priv->clk = clk;
	//
	//
	// CHECK
	//const void *match = of_device_get_match_data(&spi->dev);
	//
	//if (match)
	//	priv->devtype_data = *(struct mcp251xfd_devtype_data *)match;
	//else
	//	priv->devtype_data = *(struct mcp251xfd_devtype_data *)spi_get_device_id(spi)->driver_data;
	//
	// or
	//priv->devtype_data = *(struct mcptest_devtype_data *)
	//		spi_get_device_id(spi)->driver_data;
	//
	//priv->spi_max_speed_hz_orig = spi->max_speed_hz;
	
#ifndef TID_SPI_FREQ
	spi->max_speed_hz = min(spi->max_speed_hz, freq / 2 / 1000 * 850);
#else
	spi->max_speed_hz = min(TID_SPI_FREQ, freq / 2 / 1000 * 850);
#endif
	
	spi->bits_per_word = 8;
	
	/////////////////////////////////////////zgrep FORCE_UNLOAD /proc/config.gz

printk(KERN_NOTICE "spi_setup\n");
//return 0;

dev_err( &spi->dev, "%s():%d spi-> max_speed_hz:%d \n", __FUNCTION__, __LINE__, spi->max_speed_hz);
	err = spi_setup(spi);

dev_err( &spi->dev, "%s():%d spi_setup:%d \n", __FUNCTION__, __LINE__, err);
	if (err)
		goto remove;

	//err = mcptest_regmap_init(priv);
//dev_err( &spi->dev, "%s():%d spi_setup:%d \n", __FUNCTION__, __LINE__, err);
	//if (err)
	//	goto remove;

	//err = mcptest_register(priv);
//dev_err( &spi->dev, "%s():%d spi_setup:%d \n", __FUNCTION__, __LINE__, err);
	//if (err)
	//	goto remove;
	
	

	err = vikam_te(spi);

	if(err)
		goto remove;

//
//int spi_sync_transfer(struct spi_device *spi, struct spi_transfer *xfers, unsigned int num_xfers)
//spi – a device with which data will be exchanged.
//xfers – An array of spi_transfers.
//num_xfers – Number of items in the xfer array.

//
//int spi_async(struct spi_device *spi, struct spi_message *message)
//spi – a device with which data will be exchanged.
//message – describes the data transfers, including completion callback.
//

//
//This API is used to write the data and followed by a read. This is synchronous.
//
//int spi_write_then_read(struct spi_device * spi, const void * txbuf, unsigned n_tx, void * rxbuf, unsigned n_rx)
//
//spi – a device with which data will be exchanged.
//txbuf – data to be written.
//n_tx – size of txbuf (in bytes).
//rxbuf – the buffer into which data will be read.
//n_rx – size of rxbuf (in bytes).
//

	return 0;

 remove:
	spi->max_speed_hz = priv->spi_max_speed_hz_orig;

	if(spi)
	{
		spi_unregister_device(spi);    // Unregister the SPI slave
		pr_info("spi device unregistered :) happy hacking :P\nIm waiting the next test :P\n");
	}

	return err;
}

/*
static void __exit etx_spi_exit(void)
{ 
}
module_init(etx_spi_init);
module_exit(etx_spi_exit); // https://embetronicx.com/tutorials/linux/device-drivers/linux-kernel-spi-device-driver-tutorial/
*/

static int mcptest_remove(struct spi_device *spi)
{
	struct mcp251xfd_priv *priv = spi_get_drvdata(spi);

	spi->max_speed_hz = priv->spi_max_speed_hz_orig;

	if(spi)
	{
		spi_unregister_device(spi);    // Unregister the SPI slave
		pr_info("spi device unregistered :) happy hacking :P\nIm waiting the next test :P\n");
	}

	return 0;
}

/*
static const struct dev_pm_ops mcptest_pm_ops = {
	SET_RUNTIME_PM_OPS(mcptest_runtime_suspend,
			   mcptest_runtime_resume, NULL)
};
*/

static const struct spi_device_id mcptest_id_table[] = {
	{
		.name = "mcp2517fd",
		.driver_data = (kernel_ulong_t)&mcp251xfd_devtype_data_mcp2517fd,
	}, {
		.name = "mcp2518fd",
		.driver_data = (kernel_ulong_t)&mcp251xfd_devtype_data_mcp2518fd,
	}, {
		.name = "mcp251xfd",
		.driver_data = (kernel_ulong_t)&mcp251xfd_devtype_data_mcp251xfd,
	}, {
		/* sentinel */
	},
};
MODULE_DEVICE_TABLE(spi, mcptest_id_table);

static const struct of_device_id mcptest_of_match[] = {
	{
		.compatible = "microchip,mcp2517fd",
		.data = &mcp251xfd_devtype_data_mcp2517fd,
	}, {
		.compatible = "microchip,mcp2518fd",
		.data = &mcp251xfd_devtype_data_mcp2518fd,
	}, {
		.compatible = "microchip,mcp251xfd",
		.data = &mcp251xfd_devtype_data_mcp251xfd,
	}, {
		/* sentinel */
	},
};
MODULE_DEVICE_TABLE(of, mcptest_of_match);

static struct spi_driver mcptest_driver = {
	.driver = {
		.name = DEVICE_NAME,
		//.pm = &mcptest_pm_ops,
		.of_match_table = mcptest_of_match,
	},
	.probe = mcptest_probe,
	.remove = mcptest_remove,
	.id_table = mcptest_id_table,
};

module_spi_driver(mcptest_driver);

/*
static void mcptest_exit(void)
{
   printk(KERN_ALERT "Goodbye, world 2\n");
}
module_exit(mcptest_exit);
*/

MODULE_AUTHOR("Acho @ T&D Engineering");
MODULE_DESCRIPTION("mcptest test");
MODULE_LICENSE("GPL v2");
