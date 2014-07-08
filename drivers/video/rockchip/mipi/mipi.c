#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <linux/spi/spi.h>

#define CMD_SZ  4
/* Flash Operating Commands */
#define CMD_READ_ID                 0x9f
#define CMD_WRITE_ENABLE    0x06   
#define CMD_BULK_ERASE        0xc7
#define CMD_READ_BYTES        0x03
#define CMD_PAGE_PROGRAM    0x02
#define CMD_RDSR            0x05 

struct ssd2828{
		struct spi_device   *spi;
		struct mutex        lock;
		char    erase_opcode;
		char    cmd[CMD_SZ];
      };
      
struct ssd2828 *flash = NULL;

static int read_sr(struct ssd2828 *flash)
{
	ssize_t retval;
	u8 code = CMD_RDSR;
	u8 val;

	retval = spi_write_then_read(flash->spi, &code, 1, &val, 1);

	if (retval < 0) {
		dev_err(&flash->spi->dev, "error %d reading SR\n", (int) retval);
		return retval;
	}

	return val;
}

static int check_id( struct ssd2828 *flash )
{
	char buf[10] = {0};
	flash->cmd[0] = CMD_READ_ID;
	spi_write_then_read( flash->spi, flash->cmd, 1, buf, 3 );
	printk( "Manufacture ID: 0x%x\n", buf[0] );
	printk( "Device ID: 0x%x\n", buf[1] | buf[2]  << 8 );
	return buf[2] << 16 | buf[1] << 8 | buf[0];
}

static int __devinit ssd2828_probe(struct spi_device *spi)
{
	printk(KERN_INFO "ssd2828 probe start");
	
	flash = kzalloc(sizeof(struct ssd2828), GFP_KERNEL);
	if(!flash)
	{
		dev_err(&flash->spi->dev, "no memory for state\n");

    	return -ENOMEM;
    }
	
	flash->spi = spi;
	mutex_init( &flash->lock );
	/* save flash as driver's private data */
    spi_set_drvdata( spi, flash );
    
    mutex_init( &flash->lock );
	/* save flash as driver's private data */
    spi_set_drvdata( spi, flash );
   
    check_id( flash );    //读取ID
    
	
    
    printk(KERN_INFO "ssd2828 prob end");

}

static struct spi_driver ssd2828_driver = {
	.driver = {
		.name = "ssd2828",
		.owner = THIS_MODULE,
	},
	.probe = ssd2828_probe,
	//.remove = __devexit_p(ad9852_remove),
};

static int __init ssd2828_mipi_init(void)
{
    return spi_register_driver(&ssd2828_driver);
    printk(KERN_INFO "ssd2828 init");
    
}

static void __exit ssd2828_mipi_exit(void)
{
    spi_unregister_driver(&ssd2828_driver);

}

module_init(ssd2828_mipi_init);
module_exit(ssd2828_mipi_exit);
