#ifndef __I2C_MODULE_H__
#define __I2C_MODULE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

/*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL

/*!< GPIO number used for I2C master data  */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA

/*!< I2C master i2c port number, the number of i2c peripheral interfaces
 * available will depend on the chip */
#define I2C_MASTER_NUM (0)

#define I2C_MASTER_FREQ_HZ (400000)   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE (0) /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE (0) /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS (1000)

esp_err_t i2c_master_init(void);

#ifdef __cplusplus
}
#endif

#endif
