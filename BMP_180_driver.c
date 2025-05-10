#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#define DRIVER_NAME     "bmp180_driver"
#define CLASS_NAME      "bmp180"
#define DEVICE_NAME     "bmp180"

#define BMP180_ADDR             0x77
#define BMP180_REG_ID           0xD0
#define BMP180_REG_CONTROL      0xF4
#define BMP180_REG_TEMP_DATA    0xF6
#define BMP180_REG_PRESSURE_DATA 0xF6
#define BMP180_CMD_TEMP         0x2E
#define BMP180_CMD_PRESSURE     0x34

// IOCTL commands
#define BMP180_IOCTL_MAGIC      'b'
#define BMP180_IOCTL_READ_TEMP  _IOR(BMP180_IOCTL_MAGIC, 1, int)
#define BMP180_IOCTL_READ_PRESS _IOR(BMP180_IOCTL_MAGIC, 2, int)

static struct i2c_client *bmp180_client;
static struct class* bmp180_class = NULL;
static struct device* bmp180_device = NULL;
static int major_number;

static int bmp180_read_raw_temp(struct i2c_client *client)
{
    int ret;
    u8 buf[2];

    i2c_smbus_write_byte_data(client, BMP180_REG_CONTROL, BMP180_CMD_TEMP);
    msleep(5);

    ret = i2c_smbus_read_i2c_block_data(client, BMP180_REG_TEMP_DATA, 2, buf);
    if (ret < 0)
        return ret;

    return (buf[0] << 8) | buf[1];
}

static int bmp180_read_raw_pressure(struct i2c_client *client)
{
    int ret;
    u8 buf[3];

    i2c_smbus_write_byte_data(client, BMP180_REG_CONTROL, BMP180_CMD_PRESSURE);
    msleep(8);

    ret = i2c_smbus_read_i2c_block_data(client, BMP180_REG_PRESSURE_DATA, 3, buf);
    if (ret < 0)
        return ret;

    return ((buf[0] << 16) | (buf[1] << 8) | buf[2]) >> 8;
}

static long bmp180_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int data;

    switch (cmd) {
        case BMP180_IOCTL_READ_TEMP:
            data = bmp180_read_raw_temp(bmp180_client);
            break;
        case BMP180_IOCTL_READ_PRESS:
            data = bmp180_read_raw_pressure(bmp180_client);
            break;
        default:
            return -EINVAL;
    }

    if (data < 0)
        return data;

    if (copy_to_user((int __user *)arg, &data, sizeof(data)))
        return -EFAULT;

    return 0;
}

static int bmp180_open(struct inode *inodep, struct file *filep)
{
    printk(KERN_INFO "BMP180 device opened\n");
    return 0;
}

static int bmp180_release(struct inode *inodep, struct file *filep)
{
    printk(KERN_INFO "BMP180 device closed\n");
    return 0;
}

static struct file_operations fops = {
    .open = bmp180_open,
    .unlocked_ioctl = bmp180_ioctl,
    .release = bmp180_release,
};

static int bmp180_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int chip_id;

    bmp180_client = client;

    chip_id = i2c_smbus_read_byte_data(client, BMP180_REG_ID);
    if (chip_id != 0x55) {
        printk(KERN_ERR "BMP180 not detected! ID = 0x%x\n", chip_id);
        return -ENODEV;
    }

    major_number = register_chrdev(0, DEVICE_NAME, &fops);
    if (major_number < 0) {
        printk(KERN_ERR "Failed to register a major number\n");
        return major_number;
    }

    bmp180_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(bmp180_class)) {
        unregister_chrdev(major_number, DEVICE_NAME);
        return PTR_ERR(bmp180_class);
    }

    bmp180_device = device_create(bmp180_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
    if (IS_ERR(bmp180_device)) {
        class_destroy(bmp180_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        return PTR_ERR(bmp180_device);
    }

    printk(KERN_INFO "BMP180 driver installed successfully\n");
    return 0;
}

static void bmp180_remove(struct i2c_client *client)
{
    device_destroy(bmp180_class, MKDEV(major_number, 0));
    class_unregister(bmp180_class);
    class_destroy(bmp180_class);
    unregister_chrdev(major_number, DEVICE_NAME);

    printk(KERN_INFO "BMP180 driver removed\n");
}

static const struct i2c_device_id bmp180_id[] = {
    { "bmp180", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, bmp180_id);

static const struct of_device_id bmp180_of_match[] = {
    { .compatible = "bosch,bmp180", },
    { }
};
MODULE_DEVICE_TABLE(of, bmp180_of_match);

static struct i2c_driver bmp180_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(bmp180_of_match),
    },
    .probe = bmp180_probe,
    .remove = bmp180_remove,
    .id_table = bmp180_id,
};

static int __init bmp180_init(void)
{
    return i2c_add_driver(&bmp180_driver);
}

static void __exit bmp180_exit(void)
{
    i2c_del_driver(&bmp180_driver);
}

module_init(bmp180_init);
module_exit(bmp180_exit);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("BMP180 I2C Driver with IOCTL Interface");
MODULE_LICENSE("GPL");
