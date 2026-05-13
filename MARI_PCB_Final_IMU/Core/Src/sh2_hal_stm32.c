/* shim for the BNO085 */
#include "sh2_hal.h"
#include "sh2_err.h"
#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdbool.h>

extern osSemaphoreId_t imuSemHandle;
extern I2C_HandleTypeDef hi2c2;
#define BNO_I2C  (&hi2c2)

#define SH2_I2C_ADDR_7B  0x4A      /* SA0 = 0 (R4 to GND) */

static volatile bool     rxReady;
volatile uint32_t rxTimestampUs;
static volatile bool     inReset;

/* DWT cycle counter as us source */
static void usTimerInit(void) { /* no-op */ }

static uint32_t timeUs(void)
{
    return HAL_GetTick() * 1000U;   // ms → µs, monotonic, always works
}

/* Called from HAL_GPIO_EXTI_Callback when PD0 falls */
void BNO_INTN_OnFalling(void)
{
    rxTimestampUs = timeUs();
    inReset = false;
    rxReady = true;
    if (imuSemHandle != NULL) osSemaphoreRelease(imuSemHandle);

}

static int hal_open(sh2_Hal_t *self)
{
    usTimerInit();
    rxReady = false;

    /* BNO and STM32 share NRESET — by the time we get here the BNO has
       already booted and INTN may already be asserted (low).  Check both. */
    if (HAL_GPIO_ReadPin(BNO_INTN_GPIO_Port, BNO_INTN_Pin) == GPIO_PIN_RESET) {
        rxReady = true;
        inReset = false;
    } else {
        inReset = true;
        uint32_t start = HAL_GetTick();
        while (inReset && (HAL_GetTick() - start) < 2000) {
            HAL_Delay(1);
        }
    }
    return SH2_OK;
}

static void hal_close(sh2_Hal_t *self) { (void)self; }

static int hal_read(sh2_Hal_t *self, uint8_t *pBuf, unsigned len, uint32_t *t)
{
    (void)self;
    if (!rxReady) return 0;
    rxReady = false;          // consume the flag up front

    /* Read everything in one go. The BNO is happy to send up to 'len' bytes;
       SHTP's first 2 bytes tell us how much was actually meaningful. */
    if (HAL_I2C_Master_Receive(BNO_I2C, SH2_I2C_ADDR_7B << 1,
                               pBuf, len, 50) != HAL_OK) {
        return 0;
    }

    uint16_t total = ((pBuf[1] & 0x7F) << 8) | pBuf[0];
    if (total == 0 || total == 0xFFFF) return 0;
    if (total > len) total = len;

    *t = rxTimestampUs;
    return total;
}

static int hal_write(sh2_Hal_t *self, uint8_t *pBuf, unsigned len)
{
    (void)self;
    if (HAL_I2C_Master_Transmit(BNO_I2C, SH2_I2C_ADDR_7B << 1,
                                pBuf, len, 50) != HAL_OK)
        return SH2_ERR_IO;        // negative → SHTP txProcess will abort, not retry forever
    return len;
}

static uint32_t hal_getTimeUs(sh2_Hal_t *self) { (void)self; return timeUs(); }

static sh2_Hal_t sh2Hal;

sh2_Hal_t *sh2_hal_init(void)
{
    sh2Hal.open      = hal_open;
    sh2Hal.close     = hal_close;
    sh2Hal.read      = hal_read;
    sh2Hal.write     = hal_write;
    sh2Hal.getTimeUs = hal_getTimeUs;
    return &sh2Hal;
}
