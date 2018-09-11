#ifndef __AK8975_H_
#define __AK8975_H_

//by CAD1 CAD0 pin setting
//I2C address
#define AK8975_I2C_CAD00_ADD            0x0C //default
#define AK8975_I2C_CAD01_ADD            0x0D
#define AK8975_I2C_CAD10_ADD            0x0E
#define AK8975_I2C_CAD11_ADD            0x0F

#define AK8975_REG_WIA                  0x00
#define AK8975_REG_INFO		        0x01
#define AK8975_REG_ST1                  0x02
#define AK8975_REG_HXL			0x03
#define AK8975_REG_HXH			0x04
#define AK8975_REG_HYL			0x05
#define AK8975_REG_HYH			0x06
#define AK8975_REG_HZL			0x07
#define AK8975_REG_HZH			0x08
#define AK8975_REG_ST2                  0x09
#define AK8975_REG_CNTL		        0x0A
#define AK8975_REG_ASAX		        0x10
#define AK8975_REG_ASAY		        0x11
#define AK8975_REG_ASAZ		        0x12

#define AK8975_WIA1_VALUE		0x48

#define AK8975_MODE_POWERDOWN           0x00
#define AK8975_MODE_SNG_MEASURE	        0x01
#define AK8975_MODE_SELF_TEST		0x08
#define AK8975_MODE_FUSE_ACCESS	        0x0F

#define AK8975_ST1_DRDY_MASK           0x01

#define AK8975_ST2_HOLF_MASK           0x08
#define AK8975_ST2_DERR_MASK           0x04

#define AK8975_RAW_TO_GAUSS(asa)	((((asa) + 128) * 3000) / 256)

#define AK8975_MAX_CONVERSION_TIMEOUT_MS	500
#define AK8975_CONVERSION_DONE_POLL_TIME_MS	10

#endif
