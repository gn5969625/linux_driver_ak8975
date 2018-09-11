#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <asm/mach/map.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include "ak8975.h"
#include <linux/sysfs.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/mutex.h>
//#include <linux/mod_devicetable.h>
#define DRV_VERSION "V1.0"

#define AKM_08975_MAX_RANGE 4095
/*
 * Adjusting the flux value with the sensitivity adjustment value should be
 * done via the following formula:
 *
 * Hadj = H * ( ( ( (ASA-128)*0.5 ) / 128 ) + 1 )
 * where H is the raw value, ASA is the sensitivity adjustment, and Hadj
 * is the retultant adjusted value.
 *
 * We reduce the formula to:
 *
 * Hadj = H * (ASA + 128) / 256
 *
 * H is in the range of -4096 to 4095.  The magnetometer has a range of
 * +-1229uT.  To go from the raw value to uT is:
 *
 * HuT = H * 1229/4096, or roughly, 3/10.
 *
 * Since 1uT = 0.01 gauss, our final scale factor becomes:
 *
 * Hadj = H * ((ASA + 128) / 256) * 3/10 * 1/100
 * Hadj = H * ((ASA + 128) * 0.003) / 256
 *
 * Since ASA doesn't change, we cache the retultant scale factor into the
 * device context in ak8975_setup().
 *
 * Given we use IIO_VAL_INT_PLUS_MICRO bit when displaying the scale, we
 * multiply the stored scale value by 1e6.
 */

// raw data convert to uT and multiple asa data to caculate the final data
static long ak8975_raw_to_gauss(u16 asa)
{
        return (((long)asa + 128) * 3000) / 256;
}

//static struct i2c_client *ak8975_client;
typedef struct akm8975_data {
       struct i2c_client *client;
       struct input_dev *input_dev;
       struct work_struct work;
       unsigned char asa[3]; //get Sensitivity Adjustment Values from fuse rom
       long raw_to_gauss[3]; //final scale to real data,ex: x_raw_data*raw_to_gauss[0]/10^6 (unit:gauss)
       struct mutex lock;
       unsigned char mode;
}akm8975_data;

static void set_akm8975_mode(akm8975_data *data,unsigned char mode) {
     int ret;
     data->mode = mode;
     ret = i2c_smbus_write_byte_data(data->client,AK8975_REG_CNTL2,data->mode);
}

static int i2c_ak8975_read_len(struct i2c_client *client,unsigned char reg_addr,unsigned char len,unsigned char *buf)
{
        int ret;
        unsigned char txbuf = reg_addr;
        struct i2c_msg msg[] = {
                {client->addr,0,1,&txbuf},
                {client->addr,1,len,buf}
        };
        ret = i2c_transfer(client->adapter,msg,2);
        if(ret < 0) {
                printk("i2c_transfer read len error\n");
                return -1;
        }
        return 0;
}

//static akm8975_data akm_data;
static void get_akm8975_raw_data(akm8975_data *data,unsigned char raw_data[]) {
     int ret;
     set_akm8975_mode(data,AK8975_MODE_SNG_MEASURE);
     do {
       ret = i2c_smbus_read_byte_data(data->client,AK8975_REG_ST1);
     }while(!(ret & AK8975_ST1_DRDY_MASK ));
     i2c_ak8975_read_len(data->client,AK8975_REG_HXL,6,raw_data);
}

static ssize_t ak8975_data_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    unsigned char raw_buf[6];
    akm8975_data *data = dev_get_drvdata(dev);
    get_akm8975_raw_data(data,raw_buf);
    input_report_abs(data->input_dev, ABS_X, (raw_buf[1]<<8) | raw_buf[0]);
    input_report_abs(data->input_dev, ABS_Y, (raw_buf[3]<<8) | raw_buf[2]);
    input_report_abs(data->input_dev, ABS_Z, (raw_buf[5]<<8) | raw_buf[4]);
    input_sync(data->input_dev);
    return sprintf(buf,"x=%x,y=%x,z=%x\n",(raw_buf[1]<<8) | raw_buf[0],(raw_buf[3]<<8) | raw_buf[2], (raw_buf[5]<<8) | raw_buf[4]);
}
static DEVICE_ATTR(ak8975_data, 0644 , ak8975_data_show, NULL);

static ssize_t ak8975_mode_show(struct device *dev,struct device_attribute *attr, char *buf)
{
        akm8975_data *data = dev_get_drvdata(dev);
	return sprintf(buf ,"mode = %x\n",data->mode);
}

static ssize_t ak8975_mode_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
        akm8975_data *data = dev_get_drvdata(dev);
        data->mode =  (unsigned char)simple_strtoul(buf,NULL, 10);
	set_akm8975_mode(data,data->mode);
        return count;
}

static DEVICE_ATTR(ak8975_mode, 0644 , ak8975_mode_show, ak8975_mode_store);
static struct attribute *ak8975_attrs[] = {
    &dev_attr_ak8975_mode.attr,
    &dev_attr_ak8975_data.attr,
    NULL
};
static struct attribute_group ak8975_attr_group = {
    .name = "ak8975",
    .attrs = ak8975_attrs,
};
static void set_akm8975_retet(akm8975_data *data) {
     int ret;
     ret = i2c_smbus_write_byte_data(data->client,AK8975_REG_CNTL3,AK8975_RESET_DATA_MASK);
}

static void get_akm8975_asa(akm8975_data *data) {
     int ret;
     //enter fuse rom mode
     ret = i2c_smbus_write_byte_data(data->client,AK8975_REG_CNTL2,AK8975_MODE_FUSE_ACCESS);
     mdelay(100);
     data->asa[0] = i2c_smbus_read_byte_data(data->client,AK8975_REG_ASAX);
     data->asa[1] = i2c_smbus_read_byte_data(data->client,AK8975_REG_ASAY);
     data->asa[2] = i2c_smbus_read_byte_data(data->client,AK8975_REG_ASAZ);
 
     data->raw_to_gauss[0] = ak8975_raw_to_gauss(data->asa[0]);
     data->raw_to_gauss[1] = ak8975_raw_to_gauss(data->asa[1]);
     data->raw_to_gauss[2] = ak8975_raw_to_gauss(data->asa[2]);
     //return power dowm mode
     ret = i2c_smbus_write_byte_data(data->client,AK8975_REG_CNTL2,AK8975_MODE_POWERDOWN);
}
static void set_ak8975_input_dev(akm8975_data *data) {
        int ret;
        data->input_dev = input_allocate_device();
        set_bit(EV_ABS, data->input_dev->evbit);
        /* x-axis magnetic */
        input_set_abs_params(data->input_dev, ABS_X, -AKM_08975_MAX_RANGE, AKM_08975_MAX_RANGE, 0, 0);
        /* y-axis magnetic */
        input_set_abs_params(data->input_dev, ABS_Y, -AKM_08975_MAX_RANGE, AKM_08975_MAX_RANGE, 0, 0);
        /* z-axis magnetic */
        input_set_abs_params(data->input_dev, ABS_Z, -AKM_08975_MAX_RANGE, AKM_08975_MAX_RANGE, 0, 0);
        data->input_dev->name = "akm8975";
        data->input_dev->dev.parent = &data->client->dev;
        ret = input_register_device(data->input_dev);
}
static int ak8975_dev_init(akm8975_data *data) {
	unsigned char ret;
	printk("%s called\n", __func__);
        i2c_smbus_write_byte_data(data->client,AK8975_REG_CNTL3,AK8975_RESET_DATA_MASK);
        mdelay(100);
        set_akm8975_mode(data,AK8975_MODE_POWERDOWN);
 
	ret = i2c_smbus_read_byte_data(data->client, AK8975_REG_WIA1);
        //if(ret < 0)
	//   return 0;
        //if(ret == AK8975_WIA1_VALUE)
        printk("%s,WIA ID:%x\n",__func__,ret);
        ret = i2c_smbus_read_byte_data(data->client, AK8975_REG_INFO);
	//if(ret < 0)
        //   return 0;
        //if(ret == AK8975_WIA2_VALUE)
        printk("%s,Device INFO:%x\n",__func__,ret);

        get_akm8975_asa(data);
        set_akm8975_mode(data,AK8975_MODE_POWERDOWN);
        set_ak8975_input_dev(data);

}

static int ak8975_probe(struct i2c_client *i2c, const struct i2c_device_id *id) {
	int ret;
        u8 wia_val[2];
        akm8975_data *data = NULL;
	dev_dbg(&i2c->dev, "%s\n", __func__);
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL))
                return -ENODEV;
	dev_info(&i2c->dev, "chip found, driver version " DRV_VERSION "\n");

        data = kzalloc(sizeof(akm8975_data),GFP_KERNEL);
	//ak8975_client = i2c;
        data->client = i2c;
        ret = i2c_smbus_read_i2c_block_data(i2c, AK8975_REG_WIA1,
					    2, wia_val);
	if (ret < 0) {
		dev_err(&i2c->dev, "Error reading WIA\n");
		return ret;
	}
        dev_dbg(&i2c->dev, "WIA %02x %02x\n", wia_val[0], wia_val[1]);

        mutex_init(&data->lock);
	ak8975_dev_init(data);
	printk("ak8975 device component found!~\n");
	ret = sysfs_create_group(&i2c->dev.kobj, &ak8975_attr_group);
        i2c_set_clientdata(i2c,data);
	return 0;
}

static int ak8975_remove(struct i2c_client *i2c) {
        akm8975_data *data = i2c_get_clientdata(i2c);
        input_unregister_device(data->input_dev);
        input_free_device(data->input_dev);
        mutex_destroy(&data->lock);
        kfree(data);
	sysfs_remove_group(&i2c->dev.kobj, &mydrv_attr_group);
	return 0;
}
/*
static int akm8975_open(struct inode *inode, struct file *file) {
        return 0;
}

static int akm8975_release(struct inode *inode, struct file *file) {
        return 0;
}

static long akm8975_ioctl( struct file *file, unsigned int cmd,unsigned long arg) {
        return 0;
}

static struct file_operations akm8975_fops = {
        .owner = THIS_MODULE,
        .open = akm8975_open,
        .release = akm8975_release,
        .ioctl = akm8975_ioctl,
};

static struct miscdevice akm8975_device = {
        .minor = MISC_DYNAMIC_MINOR,
        .name = "akm8975_device",
        .fops = &akm8975_fops,
};
*/
static const struct i2c_device_id ak8975_id[] = {  
    { "kevin,ak8975",0},
    {}
};
MODULE_DEVICE_TABLE(i2c, ak8975_id);

static struct of_device_id ak8975_of_match[] = {
        { .compatible = "kevin,ak8975" },
        { },
};
MODULE_DEVICE_TABLE(of, ak8975_of_match);

static struct i2c_driver ak8975_driver = {
    .driver = {
        .name           = "kevin,ak8975",
        .owner          = THIS_MODULE,
        .of_match_table = of_match_ptr(ak8975_of_match),
    },
    .probe      = ak8975_probe,
    .remove     = ak8975_remove,
    .id_table   = ak8975_id,
};
module_i2c_driver(ak8975_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin.Shen");
MODULE_DESCRIPTION("A i2c-ak8975 driver for testing module ");
MODULE_VERSION(DRV_VERSION);
