/* MS5611.h */
#ifndef MS5611_H
#define MS5611_H

#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO    22    /* GPIO SCL */
#define I2C_MASTER_SDA_IO    21    /* GPIO SDA */
#define I2C_MASTER_FREQ_HZ   100000
#define I2C_MASTER_NUM       I2C_NUM_0
#define MS5611_ADDR          0x77   /* MS5611 DEFAULT I2C ADDRESS */

void i2c_master_init();
void ms5611_reset();
void ms5611_read_prom();
void ms5611_read_data(float *temperature, float *pressure, float *altitude);

#endif /* MS5611_H */

