#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/cdev.h>

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

#define BMP180_CALIB_START      0xAA
#define BMP180_CALIB_LEN        22

#define BMP180_OSS              0

#define BMP180_IOCTL_MAGIC      'b'
#define BMP180_IOCTL_READ_TEMP_REAL  _IOR(BMP180_IOCTL_MAGIC, 1, int)
#define BMP180_IOCTL_READ_PRESS_REAL _IOR(BMP180_IOCTL_MAGIC, 2, int)

static struct i2c_client *bmp180_client = NULL;
static struct class *bmp180_class = NULL;
static struct device* bmp180_device = NULL;
static int major_number;

static s16 AC1, AC2, AC3, B1, B2, MB, MC, MD;
static u16 AC4, AC5, AC6;

static int bmp180_read_calibration_data(struct i2c_client *client)
{
    u8 calib_data[BMP180_CALIB_LEN];
    int ret = i2c_smbus_read_i2c_block_data(client, BMP180_CALIB_START, BMP180_CALIB_LEN, calib_data);
    if (ret < 0)
        return ret;

    AC1 = (calib_data[0] << 8) | calib_data[1];
    AC2 = (calib_data[2] << 8) | calib_data[3];
    AC3 = (calib_data[4] << 8) | calib_data[5];
    AC4 = (calib_data[6] << 8) | calib_data[7];
    AC5 = (calib_data[8] << 8) | calib_data[9];
    AC6 = (calib_data[10] << 8) | calib_data[11];
    B1  = (calib_data[12] << 8) | calib_data[13];
    B2  = (calib_data[14] << 8) | calib_data[15];
    MB  = (calib_data[16] << 8) | calib_data[17];
    MC  = (calib_data[18] << 8) | calib_data[19];
    MD  = (calib_data[20] << 8) | calib_data[21];

    return 0;
}

static int bmp180_read_raw_temp(struct i2c_client *client)
{
    u8 buf[2];
    if (!bmp180_client) return -ENODEV;

    if (i2c_smbus_write_byte_data(client, BMP180_REG_CONTROL, BMP180_CMD_TEMP) < 0)
        return -EIO;
    msleep(5);
    if (i2c_smbus_read_i2c_block_data(client, BMP180_REG_TEMP_DATA, 2, buf) < 0)
        return -EIO;
    return (buf[0] << 8) | buf[1];
}

static int bmp180_read_raw_pressure(struct i2c_client *client)
{
    u8 buf[3];
    if (i2c_smbus_write_byte_data(client, BMP180_REG_CONTROL, BMP180_CMD_PRESSURE + (BMP180_OSS << 6)) < 0)
        return -EIO;
    msleep(8);
    if (i2c_smbus_read_i2c_block_data(client, BMP180_REG_PRESSURE_DATA, 3, buf) < 0)
        return -EIO;
    return (((buf[0] << 16) | (buf[1] << 8) | buf[2]) >> (8 - BMP180_OSS));
}

static int bmp180_calculate_B5(int UT)
{
    int X1 = ((UT - (int)AC6) * (int)AC5) >> 15;
    int X2 = ((int)MC << 11) / (X1 + (int)MD);
    return X1 + X2;
}

static int bmp180_get_temperature(void)
{
    int UT, B5;
    UT = bmp180_read_raw_temp(bmp180_client);
    if (UT < 0) return UT;

    B5 = bmp180_calculate_B5(UT);
    return (B5 + 8) >> 4;
}

static int bmp180_get_pressure(void)
{
    int X1, X2, X3, B3, B5, B6;
    unsigned int B4, B7;
    int p, UT, UP;

    UT = bmp180_read_raw_temp(bmp180_client);
    if (UT < 0) return UT;
    UP = bmp180_read_raw_pressure(bmp180_client);
    if (UP < 0) return UP;

    B5 = bmp180_calculate_B5(UT);

    B6 = B5 - 4000;
    X1 = (B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = (AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = (((AC1 * 4 + X3) << BMP180_OSS) + 2) >> 2;

    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = (AC4 * (unsigned int)(X3 + 32768)) >> 15;
    B7 = ((unsigned int)UP - B3) * (50000 >> BMP180_OSS);

    if (B7 < 0x80000000)
        p = (B7 << 1) / B4;
    else
        p = (B7 / B4) << 1;

    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p = p + ((X1 + X2 + 3791) >> 4);
    return p; // in Pa
}

static long bmp180_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int data;

    switch (cmd) {
        case BMP180_IOCTL_READ_TEMP_REAL:
            data = bmp180_get_temperature();
            break;
        case BMP180_IOCTL_READ_PRESS_REAL:
            data = bmp180_get_pressure();
            break;
        default:
            return -EINVAL;
    }

    if (copy_to_user((int __user *)arg, &data, sizeof(data)))
        return -EFAULT;

    return 0;
}

static int bmp180_open(struct inode *inode, struct file *file) {
    printk(KERN_INFO "BMP180: Device opened\n");
    return 0;
}

static int bmp180_release(struct inode *inode, struct file *file) {
    printk(KERN_INFO "BMP180: Device closed\n");
    return 0;
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = bmp180_open,
    .unlocked_ioctl = bmp180_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = bmp180_ioctl,
#endif
    .release = bmp180_release,
};

static int bmp180_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int chip_id = i2c_smbus_read_byte_data(client, BMP180_REG_ID);
    if (chip_id < 0) {
        printk(KERN_ERR "Failed to read chip ID\n");
        return chip_id;
    }
    if (chip_id != 0x55) {
        printk(KERN_ERR "BMP180 not detected! ID = 0x%x\n", chip_id);
        return -ENODEV;
    }

    if (bmp180_read_calibration_data(client) < 0) {
        printk(KERN_ERR "Failed to read BMP180 calibration data\n");
        return -EIO;
    }

    bmp180_client = client;
    i2c_set_clientdata(client, NULL); 

    printk(KERN_INFO "BMP180: Probe successful\n");
    return 0;
}

static void bmp180_remove(struct i2c_client *client)
{
    printk(KERN_INFO "BMP180 driver removed\n");
    bmp180_client = NULL;
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
        .of_match_table = of_match_ptr(bmp180_of_match),
        .owner = THIS_MODULE,
    },
    .probe = bmp180_probe,
    .remove = bmp180_remove,
    .id_table = bmp180_id,
};

static struct i2c_board_info bmp180_info = {
    I2C_BOARD_INFO("bmp180", BMP180_ADDR),
};
static struct i2c_client *bmp180_manual_client = NULL;

static int __init bmp180_init(void) {
    int ret;
    printk(KERN_INFO "BMP180: Initializing driver...\n");

    major_number = register_chrdev(0, DEVICE_NAME, &fops);
    if (major_number < 0) {
        printk(KERN_ALERT "BMP180: Failed to register a major number\n");
        return major_number;
    }
    printk(KERN_INFO "BMP180 driver installed successfully with major %d\n", major_number);

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

    ret = i2c_add_driver(&bmp180_driver);
    if (ret < 0) {
        device_destroy(bmp180_class, MKDEV(major_number, 0));
        class_destroy(bmp180_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        return ret;
    }

    struct i2c_adapter *adapter;

    adapter = i2c_get_adapter(1);
    if (!adapter) {
        printk(KERN_ERR "BMP180: Failed to get I2C adapter\n");
        i2c_del_driver(&bmp180_driver);
        device_destroy(bmp180_class, MKDEV(major_number, 0));
        class_destroy(bmp180_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        return -ENODEV;
    }

    bmp180_manual_client = i2c_new_client_device(adapter, &bmp180_info);
    i2c_put_adapter(adapter);
    if (IS_ERR(bmp180_manual_client)) {
        printk(KERN_ERR "BMP180: Failed to register manual I2C device\n");
        i2c_del_driver(&bmp180_driver);
        device_destroy(bmp180_class, MKDEV(major_number, 0));
        class_destroy(bmp180_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        return PTR_ERR(bmp180_manual_client);
    }
    return 0;
}

static void __exit bmp180_exit(void) {
    i2c_del_driver(&bmp180_driver);
    device_destroy(bmp180_class, MKDEV(major_number, 0));
    class_destroy(bmp180_class);
    unregister_chrdev(major_number, DEVICE_NAME);

    if (bmp180_manual_client) {
        i2c_unregister_device(bmp180_manual_client);
        bmp180_manual_client = NULL;
    };
    printk(KERN_INFO "BMP180: Goodbye from the driver!\n");
}

module_init(bmp180_init);
module_exit(bmp180_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dang Quang - Sy Phu - Khanh Hung - Thanh Phuoc");
MODULE_DESCRIPTION("A simple BMP180 char device driver");
make
