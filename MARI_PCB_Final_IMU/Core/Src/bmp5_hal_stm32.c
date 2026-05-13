/* shim for the BMP581 */
#include "bmp5.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

#define BMP581_I2C_ADDR_7BIT  0x46u   // SDO tied low. Use 0x47 if SDO=high.
uint8_t bmp581_addr = BMP581_I2C_ADDR_7BIT;
struct bmp5_dev   bmp_dev;
struct bmp5_osr_odr_press_config bmp_osr_odr;

/* Bosch API hands us a 7-bit address via intf_ptr.
 * STM32 HAL wants the 8-bit form (address << 1). */
BMP5_INTF_RET_TYPE bmp5_i2c_read(uint8_t reg, uint8_t *buf,
                                        uint32_t len, void *intf_ptr)
{
    uint8_t addr7 = *(uint8_t *)intf_ptr;
    HAL_StatusTypeDef s = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr7 << 1),
                                           reg, I2C_MEMADD_SIZE_8BIT,
                                           buf, len, 100);
    return (s == HAL_OK) ? BMP5_INTF_RET_SUCCESS : -1;
}

BMP5_INTF_RET_TYPE bmp5_i2c_write(uint8_t reg, const uint8_t *buf,
                                         uint32_t len, void *intf_ptr)
{
    uint8_t addr7 = *(uint8_t *)intf_ptr;
    HAL_StatusTypeDef s = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(addr7 << 1),
                                            reg, I2C_MEMADD_SIZE_8BIT,
                                            (uint8_t *)buf, len, 100);
    return (s == HAL_OK) ? BMP5_INTF_RET_SUCCESS : -1;
}

void bmp5_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    static uint32_t cyc_per_us = 0U;
    if (cyc_per_us == 0U) {
        /* First call — turn DWT on and cache the cycle rate. */
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0U;
        DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
        cyc_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
    }
    uint32_t start  = DWT->CYCCNT;
    uint32_t target = period * cyc_per_us;
    while ((DWT->CYCCNT - start) < target) { __NOP(); }
}
