/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "bmp5.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart8;
DMA_HandleTypeDef hdma_uart8_rx;
DMA_HandleTypeDef hdma_uart8_tx;

/* Definitions for TelemTxTask */
osThreadId_t TelemTxTaskHandle;
const osThreadAttr_t TelemTxTask_attributes = {
  .name = "TelemTxTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FlightTask */
osThreadId_t FlightTaskHandle;
const osThreadAttr_t FlightTask_attributes = {
  .name = "FlightTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for TelemRxTask */
osThreadId_t TelemRxTaskHandle;
const osThreadAttr_t TelemRxTask_attributes = {
  .name = "TelemRxTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for HousekeepingTas */
osThreadId_t HousekeepingTasHandle;
const osThreadAttr_t HousekeepingTas_attributes = {
  .name = "HousekeepingTas",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for i2cMutex */
osMutexId_t i2cMutexHandle;
const osMutexAttr_t i2cMutex_attributes = {
  .name = "i2cMutex"
};
/* Definitions for loraRxSem */
osSemaphoreId_t loraRxSemHandle;
const osSemaphoreAttr_t loraRxSem_attributes = {
  .name = "loraRxSem"
};
/* Definitions for imuSem */
osSemaphoreId_t imuSemHandle;
const osSemaphoreAttr_t imuSem_attributes = {
  .name = "imuSem"
};
/* USER CODE BEGIN PV */
#define LORA_TX_BUF_SIZE   256
#define LORA_RX_DMA_SIZE   256

__attribute__((aligned(32))) static uint8_t lora_tx_buf[LORA_TX_BUF_SIZE];
__attribute__((aligned(32))) static uint8_t lora_rx_dma_buf[LORA_RX_DMA_SIZE];

static volatile uint16_t lora_rx_old_pos = 0;
static char              lora_rx_line2[LORA_RX_DMA_SIZE];
static volatile uint16_t lora_rx_line_len = 0;
static volatile uint8_t  lora_tx_busy = 0;

#define APP_ADDR 1

/* Shared offsets (set once at startup) */
static volatile float pitchOffset_g, rollOffset_g, yawOffset_g;

/* Shared telemetry snapshot — written by FlightTask, read by TelemTxTask.
   No mutex: each field is naturally 32-bit aligned and atomic on Cortex-M7.
   Brief tearing across fields is acceptable for telemetry. */
typedef struct {
    /* IMU */
    float pitch, roll, yaw;
    float baro_temp, baro_press_pa;
    /* Motors (final mixed output) */
    float motor[4];
    /* PID terms */
    float pid[9];   // pitchP,I,D, rollP,I,D, yawP,I,D
    /* Mixer outputs (your "esc" slot) */
    float mixer[4]; // throttleOut, pitchOut, rollOut, yawOut
    /* RC duty cycles */
    int   duty_cycle[4];
    /* Battery */
    int   battery_state;
} telem_state_t;

static volatile telem_state_t g_telem;

// Task Counts
volatile uint32_t flight_count = 0;
volatile uint32_t telem_tx_count = 0;
volatile uint32_t telem_rx_count = 0;
volatile uint32_t housekeeping_count = 0;

/* Per-second rates (computed by housekeeping in Hz) */
volatile uint32_t flight_rate        = 0;
volatile uint32_t telem_tx_rate      = 0;
volatile uint32_t telem_rx_rate      = 0;
volatile uint32_t housekeeping_rate  = 0;

/* Task CPU usage from uxTaskGetSystemState */
typedef struct {
    char     name[16];
    uint32_t cpu_percent;
    uint32_t stack_remaining;
    uint8_t  state;
} task_stat_t;

#define MAX_TASKS 10 // 10 tasks max
volatile task_stat_t g_task_stats[MAX_TASKS];
volatile uint8_t     g_num_tasks = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM12_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART8_Init(void);
static void MX_I2C2_Init(void);
void StartTelemTxTask(void *argument);
void StartFlightTask(void *argument);
void StartTelemRxTask(void *argument);
void StartHousekeepingTask(void *argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void LoRa_OnIdleLine(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void RTOS_RunTimeStats_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t RTOS_RunTimeStats_GetCounter(void)
{
    /* HCLK / 16 — at 216 MHz that's a 13.5 MHz counter, wraps every ~5 min,
     * which is much longer than any reasonable stats-collection interval. */
    return DWT->CYCCNT >> 4; // Uses DWT for clock
}

static volatile float g_yaw, g_pitch, g_roll;

/*Every time the SH-2 library successfully decodes an incoming sensor report packet,
 *Calls sh2_decodeSensorEvent() to convert the raw event into a typed sh2_SensorValue_t union.
 *Filters for SH2_GAME_ROTATION_VECTOR
 *Pulls the four quaternion components out (i, j, k, real)
 *Converts the quaternion to Euler angles (yaw / pitch / roll)
 *Stores them in g_yaw, g_pitch, g_roll for FlightTask
 * */
static void BNO085_interpreter(void *cookie, sh2_SensorEvent_t *ev)
{
    sh2_SensorValue_t v;
    if (sh2_decodeSensorEvent(&v, ev) != SH2_OK) return;
    if (v.sensorId != SH2_GAME_ROTATION_VECTOR) return;

    float i = v.un.gameRotationVector.i;
    float j = v.un.gameRotationVector.j;
    float k = v.un.gameRotationVector.k;
    float r = v.un.gameRotationVector.real;

    float ysqr = j * j;
    float t2 = 2.0f * (r * j - k * i);
    if (t2 >  1.0f) t2 =  1.0f;
    if (t2 < -1.0f) t2 = -1.0f;

    const float RAD2DEG = 57.2957795f;
    g_roll  = atan2f(2.0f*(r*i + j*k), 1.0f - 2.0f*(i*i + ysqr)) * RAD2DEG;
    g_pitch = asinf(t2)                                          * RAD2DEG;
    g_yaw   = atan2f(2.0f*(r*k + i*j), 1.0f - 2.0f*(ysqr + k*k)) * RAD2DEG;
}

// For things that aren't sensor data (just leave empty)
static void async_eventHandler(void *cookie, sh2_AsyncEvent_t *ev) {
	(void)cookie;
	(void)ev;
}

extern sh2_Hal_t *sh2_hal_init(void);

// Starts BNO85 from the sh2 drivers lib
volatile uint8_t g_bno_fault = 0;

volatile uint32_t rcc =0;
void BNO085_Start(void)
{

    sh2_Hal_t *hal = sh2_hal_init();
    if (sh2_open(hal, async_eventHandler, NULL) != SH2_OK) {
        g_bno_fault = 1;
        return;
    }
    sh2_setSensorCallback(BNO085_interpreter, NULL);

    sh2_SensorConfig_t cfg = {0};
    cfg.reportInterval_us = 2500;   /* 400 Hz */
    if (sh2_setSensorConfig(SH2_GAME_ROTATION_VECTOR, &cfg) != SH2_OK) {
        g_bno_fault = 2;
    }
}

//Variables for Radio Receiver Data
int capture_value[4] = {0,0,0,0};
int duty_cycle[4] = {0,0,0,0};
float motorNW = 0; //NW motor output
float motorNE = 0; //NE motor output
float motorSE = 0; //SE motor output
float motorSW = 0; //SW motor output
float pitchP = 0;
float pitchI = 0;
float pitchD = 0;
float rollP = 0;
float rollI = 0;
float rollD = 0;
float yawP = 0;
float yawI = 0;
float yawD = 0;
float pitchError = 0;
float pitchError2 = 0;
float rollError = 0;
float rollError2 = 0;
float yawError = 0;
float yawError2 = 0;
float throttle_DC = 0;
float pitch_DC = 0;
float roll_DC = 0;
float yaw_DC = 0;
int counter = 0;
volatile uint8_t timer_flag = 0;

//Variables for LoRa
volatile uint8_t lora_tx_ready = 1;
volatile uint8_t lora_new_payload = 0;
volatile uint32_t lora_tx_last_send_ms = 0;

//Variables for Sunny Commands and Mari ping
volatile uint8_t mari_ping_pending = 0;
volatile uint8_t last_sunny_addr = 0;
volatile uint8_t last_sunny_n = 0;
char last_mari_sender[16] = {0};

//Sequence numbers for telemetry packets
uint32_t seq_user = 100;
uint32_t seq_imu = 200;
uint32_t seq_pid = 300;
uint32_t seq_motor = 400;
uint32_t seq_esc = 500;
uint32_t seq_batt = 600;
uint32_t seq_raw = 700;
uint32_t seq_rate = 0;
uint32_t seq_cpu = 0;

//LoRa Defines
#define LORA_RX_LINE_MAX 128
#define LORA_DATA_MAX      128
char lora_payload[LORA_DATA_MAX];
char lora_rx_line[LORA_RX_LINE_MAX];

//ADC References
#define ADC_MAX_COUNT          4095.0f
#define ADC_REF_VOLTAGE        3.3f

#define BAT_R1                 47500.0f
#define BAT_R2                 4700.0f

#define GREEN_ON   11.4f
#define GREEN_OFF  11.2f
#define YELLOW_ON  10.5f
#define YELLOW_OFF 10.7f

uint32_t adc_raw = 0;
float battery_voltage = 0.0f;
int battery_state = 2; // start as green (or 1 if you prefer)


//Read PWM signal depending on which timer is input and story in duty cycle array
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
	  if (htim->Instance == TIM2) {
		  //Read input capture value
		  capture_value[0] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		  if (capture_value[0] != 0) {
			  //calculate duty cycle
			  duty_cycle[0] = 1000 * HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) / capture_value[0];
		  }
	  }
	  if (htim->Instance == TIM3) {
		  //Read input capture value
		  capture_value[1] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		  if (capture_value[1] != 0) {
			  //calculate duty cycle
			  duty_cycle[1] = 1000 * HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) / capture_value[1];
		  }
	  }
	  if (htim->Instance == TIM4) {
		  //Read input capture value
		  capture_value[2] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		  if (capture_value[2] != 0) {
			  //calculate duty cycle
			  duty_cycle[2] = 1000 * HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) / capture_value[2];
		  }
		}
	  if (htim->Instance == TIM5) {
    		  //Read input capture value
    		  capture_value[3] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    		  if (capture_value[3] != 0) {
    			  //calculate duty cycle
    			  duty_cycle[3] = 1000 * HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) / capture_value[3];
    		  }
	  }
  }
}

//Write to Serial Wire Viewer
int _write(int file, char *ptr, int len)
{
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}


//Read Voltage from the ADC
uint32_t Read_ADC_Value(void)
{
    uint32_t sum = 0;

    for (int i = 0; i < 16; i++)
    {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }

    return (sum / 16);
}

//Read Battery Voltage
float Read_Battery_Voltage(void)
{
    float v_adc;

    adc_raw = Read_ADC_Value();
    v_adc = ((float)adc_raw / ADC_MAX_COUNT) * ADC_REF_VOLTAGE;
    battery_voltage = v_adc * ((BAT_R1 + BAT_R2) / BAT_R2);

    return battery_voltage;
}




//Function Prototypes
void LoRa_TX(uint8_t addr, const char *fmt, ...);
HAL_StatusTypeDef LoRa_RX(char *out, uint16_t maxLen, uint32_t timeout);
uint8_t LoRa_RX_Parse(const char *line, char *payload_out, uint16_t payload_max, uint8_t *addr_out);
void LoRa_ProcessLine(char *line);
void LoRa_TX_Scheduler(uint8_t addr, float imu[3], float bmp_data[2], float motor[4], float pid[9], float esc[4], int duty_cycle[4], int rates[4], int cpu_percent[5]);
HAL_StatusTypeDef LoRa_SendAT(const char *fmt, ...);
uint8_t LoRa_Parse_SUNNY_Ping(const char *payload, uint8_t *n_out);
void LoRa_RAW_Send(uint8_t addr, const char *raw_payload);
void LoRa_Init(void);

// BMP PFPs
BMP5_INTF_RET_TYPE bmp5_i2c_read (uint8_t, uint8_t *, uint32_t, void *);
BMP5_INTF_RET_TYPE bmp5_i2c_write(uint8_t, const uint8_t *, uint32_t, void *);
void bmp5_delay_us (uint32_t, void *);

typedef struct {
	float pressure;
	float temperature;
} BMP_581_AP;

// BMP581 externs for bmp shim
extern uint8_t bmp581_addr;
extern struct bmp5_dev bmp_dev;
extern struct bmp5_osr_odr_press_config bmp_osr_odr;

int8_t BMP_581_Init(void)
{
    bmp_dev.intf       = BMP5_I2C_INTF;
    bmp_dev.read       = bmp5_i2c_read;
    bmp_dev.write      = bmp5_i2c_write;
    bmp_dev.delay_us   = bmp5_delay_us;
    bmp_dev.intf_ptr   = &bmp581_addr;

    int8_t rslt = bmp5_init(&bmp_dev);   // soft-reset + chip-ID check
    if (rslt != BMP5_OK) {
        LoRa_RAW_Send(APP_ADDR, "BMP581 initialized FAIL.");
        HAL_Delay(100);
        return rslt;
    }

    /* Standby first so config writes are accepted */
    rslt = bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, &bmp_dev);

    /* Pick OSR/ODR. 50 Hz with 8x press / 1x temp is a sane default
     * for a flight controller altitude loop; tune to taste. */
    rslt |= bmp5_get_osr_odr_press_config(&bmp_osr_odr, &bmp_dev);
    bmp_osr_odr.osr_t       = BMP5_OVERSAMPLING_1X;
    bmp_osr_odr.osr_p       = BMP5_OVERSAMPLING_8X;
    bmp_osr_odr.odr         = BMP5_ODR_50_HZ;
    bmp_osr_odr.press_en    = BMP5_ENABLE;
    rslt |= bmp5_set_osr_odr_press_config(&bmp_osr_odr, &bmp_dev);

    // IIR for noise filtering — helps with prop wash / cabin pressure noise.
    struct bmp5_iir_config iir = {
        .set_iir_t   = BMP5_IIR_FILTER_COEFF_1,
        .set_iir_p   = BMP5_IIR_FILTER_COEFF_3,
        .shdw_set_iir_t = BMP5_ENABLE,
        .shdw_set_iir_p = BMP5_ENABLE,
        .iir_flush_forced_en = BMP5_DISABLE,
    };

    rslt |= bmp5_set_iir_config(&iir, &bmp_dev);

    rslt |= bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, &bmp_dev);

    if (rslt == BMP5_OK){
    	LoRa_RAW_Send(APP_ADDR,"BMP581 initialized SUCCESS.");
    }
    else {
    	LoRa_RAW_Send(APP_ADDR, "BMP581 initialized PARTIAL FAIL.");
    }
    HAL_Delay(200);

    return rslt;
}

BMP_581_AP BMP_581_Read_Data(void)
{
    BMP_581_AP out = {0};
    struct bmp5_sensor_data d;
    if (bmp5_get_sensor_data(&d, &bmp_osr_odr, &bmp_dev) == BMP5_OK) {
        out.pressure    = d.pressure;     // in Pascals
        out.temperature = d.temperature;  // in Celsius
    }
    return out;
}

//Get dt for PIDs
float get_dt(void)
{
	static uint32_t last = 0;
	    uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);

	    if (last == 0) {
	        last = now;
	        return 0.001f;
	    }

	    uint32_t diff = now - last;
	    last = now;

	    return (float)diff / 1000000.0f;
}

uint8_t LoRa_Parse_SUNNY_Ping(const char *payload, uint8_t *n_out)
{
    if (payload == NULL || n_out == NULL) {
        return 0;
    }

    if (strncmp(payload, "SUN:", 4) != 0) {
        return 0;
    }

    char *sep = strchr(payload, '|');
    if (sep == NULL) {
        return 0;
    }

    if (strcmp(sep + 1, "SUNNY") != 0) {
        return 0;
    }

    int n = atoi(payload + 4);
    if (n < 0) {
        return 0;
    }

    *n_out = (uint8_t)n;
    return 1;
}

void LoRa_TX(uint8_t addr, const char *fmt, ...)
{
    if (!lora_tx_ready || lora_tx_busy) {
        return;
    }

    char payload[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(payload, sizeof(payload), fmt, args);
    va_end(args);

    int len = strlen(payload);
    if (len > 240 || len <= 0) return;

    int cmd_len = snprintf((char*)lora_tx_buf, LORA_TX_BUF_SIZE,
                               "AT+SEND=%u,%d,%s\r\n", (unsigned)addr, len, payload);
    if (cmd_len <= 0 || cmd_len >= LORA_TX_BUF_SIZE) return;

    /* Cache maintenance for D-cache + DMA on STM32F7 */
    SCB_CleanDCache_by_Addr((uint32_t*)lora_tx_buf, ((cmd_len + 31) & ~31));

    lora_tx_ready = 0;
    lora_tx_busy = 1;
    lora_tx_last_send_ms = HAL_GetTick();

    HAL_UART_Transmit_DMA(&huart8, lora_tx_buf, cmd_len);
    /* TX completion sets lora_tx_busy = 0 in HAL_UART_TxCpltCallback */
    /* `+OK` from the LoRa modem then sets lora_tx_ready = 1 in LoRa_ProcessLine */
}

void LoRa_RAW_Send(uint8_t addr, const char *raw_payload)
{
    if (raw_payload == NULL) {
        return;
    }

    (unsigned long)seq_raw++;

    LoRa_TX(addr, "RW:%lu|%s", (unsigned long)seq_raw, raw_payload);
}

HAL_StatusTypeDef LoRa_SendAT(const char *fmt, ...)
{
    char cmd[128];
    va_list args;

    va_start(args, fmt);
    vsnprintf(cmd, sizeof(cmd), fmt, args);
    va_end(args);

    size_t len = strlen(cmd);
    if (len + 2 >= sizeof(cmd)) {
        return HAL_ERROR;
    }

    cmd[len++] = '\r';
    cmd[len++] = '\n';
    cmd[len] = '\0';

    lora_tx_ready = 0;
    lora_tx_last_send_ms = HAL_GetTick();

    return HAL_UART_Transmit(&huart8, (uint8_t *)cmd, len, HAL_MAX_DELAY);
}

void LoRa_Send_SUNNY_ACK(uint8_t n, uint8_t addr)
{
    LoRa_TX(addr, "MARI:%u|ACK|Even if we have no faces, we Shadows still have hearts that can be blackened.",
                 (unsigned int)n);
}

HAL_StatusTypeDef LoRa_RX(char *out, uint16_t maxLen, uint32_t timeout)
{
    uint8_t ch;
    uint16_t i = 0;

    if (maxLen < 2) {
        return HAL_ERROR;
    }

    while (1) {
        HAL_StatusTypeDef status = HAL_UART_Receive(&huart8, &ch, 1, timeout);
        if (status != HAL_OK) {
            return status;
        }

        if (ch == '\r') {
            continue;
        }

        if (ch == '\n') {
            out[i] = '\0';
            return HAL_OK;
        }

        if (i < maxLen - 1) {
            out[i++] = (char)ch;
        } else {
            out[i] = '\0';
            return HAL_ERROR;
        }
    }
}

uint8_t LoRa_RX_Parse(const char *line, char *payload_out, uint16_t payload_max,
                      uint8_t *addr_out)
{
    const char *p = line;
    char *endptr;
    long addr;
    long len;
    uint16_t copy_len;

    if (strncmp(p, "+RCV=", 5) != 0) return 0;
    p += 5;

    addr = strtol(p, &endptr, 10);
    if (endptr == p || *endptr != ',') return 0;
    p = endptr + 1;

    len = strtol(p, &endptr, 10);
    if (endptr == p || *endptr != ',' || len < 0) return 0;
    p = endptr + 1;

    copy_len = (uint16_t)len;
    if (copy_len >= payload_max) copy_len = payload_max - 1;

    memcpy(payload_out, p, copy_len);
    payload_out[copy_len] = '\0';

    if (addr_out) *addr_out = (uint8_t)addr;
    return 1;
}

void LoRa_ProcessLine(char *line)
{
    if (!lora_tx_ready && (HAL_GetTick() - lora_tx_last_send_ms >= 1000)) {
        lora_tx_ready = 1;
    }

    if (line == NULL || line[0] == '\0') {
        return;
    }

    if (strcmp(line, "+OK") == 0) {
        lora_tx_ready = 1;
        return;
    }

    if (strncmp(line, "+ERR=", 5) == 0) {
        lora_tx_ready = 1;
        return;
    }

    if (strncmp(line, "+RCV=", 5) == 0) {
        uint8_t sender_addr = 0;
        if (LoRa_RX_Parse(line, lora_payload, LORA_DATA_MAX, &sender_addr)) {
            last_sunny_addr = sender_addr;
            lora_new_payload = 1;
        }
        return;
    }
}

void LoRa_TX_Scheduler(uint8_t addr, float imu[3], float bmp_data[2], float motor[4], float pid[9], float esc[4], int duty_cycle[4], int rates[4], int cpu_percent[5])
{
    static uint32_t last_imu_tx    = 0;
    static uint32_t last_motor_tx  = 0;
    static uint32_t last_batt_tx   = 0;
    static uint32_t last_user_tx   = 0;
    static uint32_t last_pid_tx    = 0;
//    static uint32_t last_esc_tx    = 0;
    static uint32_t last_rate_tx = 0;
    static uint32_t last_cpu_tx = 0;
    static uint8_t slot = 0;   // 0=I, 1=M, 2=B, 3=U, 4=P, 5=E, 6=R, 7=C

    uint32_t now = HAL_GetTick();

    /* Recovery: if modem hasn't replied in 1 second, force ready */
    if (!lora_tx_ready && (now - lora_tx_last_send_ms >= 1000)) {
        lora_tx_ready = 1;
        lora_tx_busy = 0;   /* also clear DMA-busy in case TxCplt was missed */
    }


    if (!lora_tx_ready) {
        return;
    }

    /* ===== Priority slot: pending Sunny ACK ===== */
    if (mari_ping_pending) {
        LoRa_Send_SUNNY_ACK(last_sunny_n, last_sunny_addr);
        mari_ping_pending = 0;
        return;            /* burn this scheduler tick on the ACK; */
                           /* normal telemetry resumes next call    */
    }

    /* ===== Normal round-robin===== */

    for (int i = 0; i < 8; i++) {
        switch (slot) {
        case 0:
            if ((now - last_imu_tx) >= 800) { //800ms
                LoRa_TX(addr, "I:%lu|%.2f,%.2f,%.2f,%.2f,%.2f",
                		(unsigned long)seq_imu++,
                        imu[0], // pitch
                        imu[1], // roll
                        imu[2], // yaw
                        bmp_data[0], // Celsius
                        bmp_data[1] / 100.0f); //hPa
                last_imu_tx = now;
                slot = 1;
                return;
            }
            slot = 1;
            break;

        case 1:
            if ((now - last_motor_tx) >= 500) { //500ms
                LoRa_TX(addr, "M:%lu|%.0f,%.0f,%.0f,%.0f",
                		(unsigned long)seq_motor++,
                        motor[0], motor[1], motor[2], motor[3]);
                last_motor_tx = now;
                slot = 2;
                return;
            }
            slot = 2;
            break;

        case 2:
            if ((now - last_batt_tx) >= 5000) { //5000ms
                LoRa_TX(addr, "B:%lu|%d", (unsigned long)seq_batt++ , battery_state);
                last_batt_tx = now;
                slot = 3;
                return;
            }
            slot = 3;
            break;

        case 3:
            if ((now - last_user_tx) >= 500) { //500ms
                LoRa_TX(addr, "U:%lu|%d,%d,%d,%d",
                		(unsigned long)seq_user++,
                        duty_cycle[0],
                        duty_cycle[1],
                        duty_cycle[2],
                        duty_cycle[3]);
                last_user_tx = now;
                slot = 4;
                return;
            }
            slot = 4;
            break;
        case 4:
			if ((now - last_pid_tx) >= 500) { //500ms
				LoRa_TX(addr, "P:%lu|%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
						(unsigned long)seq_pid++,
						pid[0], pid[1], pid[2], // pitch PID
						pid[3], pid[4], pid[5], // roll PID
						pid[6], pid[7], pid[8]  // yaw PID
				);
				last_pid_tx = now;
				slot = 5;
				return;
			}
			slot = 5;
			break;

		case 5:
//			if ((now - last_esc_tx) >= 500) { //500ms
//				LoRa_TX(addr, "E:%lu|%.2f,%.2f,%.2f,%.2f",
//						(unsigned long)seq_esc++,
//						esc[0],
//						esc[1],
//						esc[2],
//						esc[3]);
//				last_esc_tx = now;
				slot = 6;
				return;
//			}
			slot = 6;
			break;

		case 6:
		    if ((now - last_rate_tx) >= 1000) { //1000ms
		        LoRa_TX(addr,"R:%lu|%d,%d,%d,%d",
		                (unsigned long)seq_rate++,
		                rates[0],
						rates[1],
						rates[2],
						rates[3]);
		        last_rate_tx = now;
		        slot = 7;
		        return;
		    }
		    slot = 7;
		    break;

		case 7:
		    if ((now - last_cpu_tx) >= 5000) { //5000ms
		        LoRa_TX(addr,"C:%lu|%d,%d,%d,%d,%d",
		                (unsigned long)seq_cpu++,
		                /* assuming task indices — verify with debugger first */
						cpu_percent[0],  /* FlightTask */
						cpu_percent[1],  /* TelemTxTask */
						cpu_percent[2],  /* TelemRxTask */
						cpu_percent[3],  /* Housekeeping */
						cpu_percent[4]);  /* IDLE */
		        last_cpu_tx = now;
		        slot = 0;
		        return;
		    }
		    slot = 0;
		    break;
        }
    }
}

void LoRa_Init(void){
	lora_tx_ready = 1;

	LoRa_SendAT("AT+ADDRESS=%d", 0); // Drone address

	HAL_Delay(100);
	LoRa_SendAT("AT+NETWORKID=18");
	HAL_Delay(100);
	LoRa_SendAT("AT+PARAMETER=7,7,1,8");
	HAL_Delay(100);
	LoRa_SendAT("AT+CRFOP=20");
	HAL_Delay(100);
	LoRa_RAW_Send(APP_ADDR,"Params: SF7, 125kHz, CR4/5, Prem 8");
	HAL_Delay(100);
	LoRa_RAW_Send(APP_ADDR,"RF Out Power is 20 dBm");
	HAL_Delay(100);
	LoRa_RAW_Send(APP_ADDR, "RYLR998 initialized SUCCESS.");
}

void UpdateTaskStats(void)
{
    static TaskStatus_t snapshot[MAX_TASKS];
    uint32_t total_runtime;

    UBaseType_t n = uxTaskGetSystemState(snapshot, MAX_TASKS, &total_runtime);
    if (total_runtime == 0) total_runtime = 1;   /* avoid div-by-zero */

    //Initialize task stats to 0
    for (int s = 0; s < MAX_TASKS; s++) {
            g_task_stats[s].name[0]         = 0;
            g_task_stats[s].cpu_percent     = 0;
            g_task_stats[s].stack_remaining = 0;
            g_task_stats[s].state           = 0;
        }

    static const struct {
        const char *name;
        uint8_t     slot;
    } slot_map[] = {
        { "FlightTask",       0 },
        { "TelemTxTask",      1 },
        { "TelemRxTask",      2 },
        { "HousekeepingTas",  3 },
        { "IDLE",             4 },
    };
    const int num_mapped = sizeof(slot_map) / sizeof(slot_map[0]);

    for (UBaseType_t i = 0; i < n; i++) {
        for (int m = 0; m < num_mapped; m++) {
            if (strcmp(snapshot[i].pcTaskName, slot_map[m].name) == 0) {
                uint8_t s = slot_map[m].slot;

                strncpy((char*)g_task_stats[s].name,
                        snapshot[i].pcTaskName, 15);
                g_task_stats[s].name[15] = 0;

                g_task_stats[s].cpu_percent =
                    (snapshot[i].ulRunTimeCounter * 100UL) / total_runtime;

                g_task_stats[s].stack_remaining =
                    snapshot[i].usStackHighWaterMark;

                g_task_stats[s].state =
                    (uint8_t)snapshot[i].eCurrentState;

                break;
            }
        }
    }

    g_num_tasks = n;
}

uint8_t NUMBER = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	// Sanity Check
	(unsigned long)NUMBER++;
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_ADC1_Init();
  MX_UART8_Init();
  MX_I2C2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */


  pitchOffset_g = 0; // store in globals
  rollOffset_g = 0;
  yawOffset_g = 0;



  // Initialize Input Channels
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_2);

  //Initialize Output Channels
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

  //Initialize PWM Output Signals to 0
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
  __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
  __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,0);
  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,0);

  HAL_Delay(200);
  BNO085_Start();
  HAL_Delay(500);
  BMP_581_Init();

  HAL_Delay(3000);
  (unsigned long)NUMBER++;


  // Initialize LoRa Module
  LoRa_Init();
  HAL_Delay(100);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of i2cMutex */
  i2cMutexHandle = osMutexNew(&i2cMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of loraRxSem */
  loraRxSemHandle = osSemaphoreNew(1, 0, &loraRxSem_attributes);

  /* creation of imuSem */
  imuSemHandle = osSemaphoreNew(1, 0, &imuSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TelemTxTask */
  TelemTxTaskHandle = osThreadNew(StartTelemTxTask, NULL, &TelemTxTask_attributes);

  /* creation of FlightTask */
  FlightTaskHandle = osThreadNew(StartFlightTask, NULL, &FlightTask_attributes);

  /* creation of TelemRxTask */
  TelemRxTaskHandle = osThreadNew(StartTelemRxTask, NULL, &TelemRxTask_attributes);

  /* creation of HousekeepingTas */
  HousekeepingTasHandle = osThreadNew(StartHousekeepingTask, NULL, &HousekeepingTas_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* UART8_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UART8_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(UART8_IRQn);
  /* I2C2_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C2_EV_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
  /* EXTI0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20404768;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x6000030D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 215;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 107;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 107;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 107;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 107;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 19999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 215;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 2499;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 215;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 2499;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 107;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 2499;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BNO_INTN_Pin */
  GPIO_InitStruct.Pin = BNO_INTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BNO_INTN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Count interrupts from BNO to determine speed
volatile uint32_t intn_count = 0;

extern void BNO_INTN_OnFalling(void);
   void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
   {
       if (GPIO_Pin == BNO_INTN_Pin) {
    	   intn_count++;
    	   BNO_INTN_OnFalling();
       }
   }

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART8) lora_tx_busy = 0;
}

void LoRa_OnIdleLine(void) {
    uint16_t dma_pos = LORA_RX_DMA_SIZE - __HAL_DMA_GET_COUNTER(huart8.hdmarx);
    if (dma_pos == lora_rx_old_pos) return;

    SCB_InvalidateDCache_by_Addr((uint32_t*)lora_rx_dma_buf, LORA_RX_DMA_SIZE);

    uint16_t len = 0;
    if (dma_pos > lora_rx_old_pos) {
        len = dma_pos - lora_rx_old_pos;
        memcpy(lora_rx_line2, &lora_rx_dma_buf[lora_rx_old_pos], len);
    } else {
        uint16_t tail = LORA_RX_DMA_SIZE - lora_rx_old_pos;
        memcpy(lora_rx_line2, &lora_rx_dma_buf[lora_rx_old_pos], tail);
        memcpy(lora_rx_line2 + tail, lora_rx_dma_buf, dma_pos);
        len = tail + dma_pos;
    }
    lora_rx_old_pos = dma_pos;
    lora_rx_line_len = len;

    osSemaphoreRelease(loraRxSemHandle);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTelemTxTask */
/**
  * @brief  Function implementing the TelemTxTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTelemTxTask */
void StartTelemTxTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  uint32_t tick = osKernelGetTickCount();
  for(;;)
  {
	telem_tx_count++;
    tick += 20;             // run scheduler every 50ms; per-message rate limits handle the rest
    osDelayUntil(tick);

    /* Take a local snapshot so values can't change mid-format */
    telem_state_t s;
    memcpy(&s, (const void*)&g_telem, sizeof(s));


    /* Build the arrays the scheduler expects */
    float    imu[3]    = { s.pitch, s.roll, s.yaw };
    float	 bmp[2]     = { s.baro_temp, s.baro_press_pa };
    float    motor[4]  = { s.motor[0], s.motor[1], s.motor[2], s.motor[3] };
    float    pid[9]    = { s.pid[0], s.pid[1], s.pid[2],
                           s.pid[3], s.pid[4], s.pid[5],
                           s.pid[6], s.pid[7], s.pid[8] };
    float    mixer[4]  = { s.mixer[0], s.mixer[1], s.mixer[2], s.mixer[3] };
    int      duty[4]   = { s.duty_cycle[0], s.duty_cycle[1],
                           s.duty_cycle[2], s.duty_cycle[3] };
    int		 rates[4]  = { flight_rate, telem_tx_rate, telem_rx_rate, housekeeping_rate};
    int		 cpu[5]	   = { 	g_task_stats[0].cpu_percent, g_task_stats[1].cpu_percent,
    						g_task_stats[2].cpu_percent, g_task_stats[3].cpu_percent,
							g_task_stats[4].cpu_percent };


    /* Note: battery_state is a global, scheduler reads it directly via the B: case */
    LoRa_TX_Scheduler(APP_ADDR, imu, bmp, motor, pid, mixer, duty, rates, cpu);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartFlightTask */
/**
* @brief Function implementing the FlightTask thread.
* @param argument: Not used
* @retval None
*/

static const float dutyMin[4] = {550, 590, 590, 590};
static const float dutyMax[4] = {950, 890, 890, 890};
static const float kpPitch = 2.0f, kiPitch = 0.0f, kdPitch = 0.0f;
static const float kpRoll  = 2.0f, kiRoll  = 0.0f, kdRoll  = 0.0f;
static const float kpYaw   = 0.0f, kiYaw   = 0.0f, kdYaw   = 0.0f;


/* USER CODE END Header_StartFlightTask */
void StartFlightTask(void *argument)
{
  /* USER CODE BEGIN StartFlightTask */
//  uint32_t tick = osKernelGetTickCount();
	osDelay(150);

  (unsigned long)NUMBER++;
  osDelay(100);

  (unsigned long)NUMBER++;
  osDelay(500);

  for(;;)
  {
	flight_count++;

	// Now a semaphore that is triggered with interrupts
	// For consistent loop rate
	osSemaphoreAcquire(imuSemHandle, 4);

//	tick += 2;                          // 2 ms = 500 Hz
//	osDelayUntil(tick);

    /* ----- RC duty cycle mapping ----- */
    for (int i = 0; i < 4; i++) {
    	if (duty_cycle[i] < dutyMin[i]) {
    		duty_cycle[i] = dutyMin[i];
    	}
    }
    float throttle_DC = (duty_cycle[0] - dutyMin[0]) / (dutyMax[0] - dutyMin[0]);
    float pitch_DC    = (duty_cycle[1] - dutyMin[1]) / (dutyMax[1] - dutyMin[1]);
    float roll_DC     = (duty_cycle[2] - dutyMin[2]) / (dutyMax[2] - dutyMin[2]);
    float yaw_DC      = (duty_cycle[3] - dutyMin[3]) / (dutyMax[3] - dutyMin[3]);

    float throttleUser = throttle_DC;
    float pitchUser = pitch_DC * 40.0f - 20.0f;
    float rollUser  = roll_DC  * 40.0f - 20.0f;
    float yawUser   = yaw_DC   * 40.0f - 20.0f;

    /* ----- IMU read (I2C1, share with anyone else who uses it) ----- */
    //osMutexAcquire(i2cMutexHandle, osWaitForever);
    sh2_service();
    /* drain pending packets from the BNO085
     * Since BNO is running at 400 Hz and FlightTask at 500 Hz, every 5th cycle gives
     * redundant IMU data
     * Consider making IMU interrupt-driven
     * */

    struct { float pitch, roll, yaw; } angles = { g_pitch, g_roll, g_yaw };
    //osMutexRelease(i2cMutexHandle);


	//-----------------------------------------------PID Control--------------------------------------------------------------------------//
	//Read BNO055 IMU Sensor Data (returns roll,pitch, yaw structure in 0-360 degrees)
	//BNO055_YPR_t angles = BNO055_ReadYPR();

    float dt = 0.002f; //Time it takes for one while loop (s)

	//Note: kp, ki, kd values are determined through trial/error
	//Pitch
	pitchError = pitchUser - (angles.pitch - pitchOffset_g);
	pitchP = kpPitch * pitchError;
	pitchI += kiPitch * pitchError * dt;
	pitchD = kdPitch * (pitchError - pitchError2) / dt;

	//Put limits on the integral portion
	if (pitchI > 500.0) {
	  pitchI = 500.0;
	}
	if (pitchI < -500.0) {
	  pitchI = -500.0;
	}
	if (duty_cycle[0] < 490) {
	  pitchI = 0;
	}

	pitchError2 = pitchError; //Update Error
	float pitchPID = pitchP + pitchI + pitchD; //Add PID together

	//Roll
	rollError = rollUser - (angles.roll - rollOffset_g);
	rollP = kpRoll * rollError;
	rollI += kiRoll * rollError * dt;
	rollD = kdRoll * (rollError - rollError2) / dt;

	//Put limits on the integral portion
	if (rollI > 500.0) {
	  rollI = 0;
	}
	if (rollI < -500.0) {
	  rollI = 0;
	}
	if (duty_cycle[0] < 490) {
	  rollI = 0;
	}

	rollError2 = rollError; //Update Error
	float rollPID = rollP + rollI + rollD; //Add PID together

	//Yaw
	yawError = yawUser - (angles.yaw - yawOffset_g);
	yawP = kpYaw * yawError;
	yawI += kiYaw * yawError * dt;
	yawD = kdYaw * (yawError - yawError2) / dt;

	//Put limits on the integral portion
	if (yawI > 500.0) {
	  yawI = 0.0;
	}
	if (yawI < -500.0) {
	  yawI = 0.0;
	}
	if (duty_cycle[0] < 500) {
	  yawI = 0;
	}
	yawError2 = yawError;//Update Error
	float yawPID = yawP + yawI + yawD;//Add PID together

	//---------------------------------------Convert to Output----------------------------------------------//
	//Convert throttle/PID values for mixing equations
	float throttleOut = throttleUser * 700 + 1100.0;
	float pitchOut = pitchPID;
	float rollOut = rollPID;
	float yawOut = yawPID;

	//Convert Inputs to motor outputs
	motorNW = throttleOut + pitchOut + rollOut - yawOut;
	motorNE = throttleOut + pitchOut - rollOut + yawOut;
	motorSW = throttleOut - pitchOut + rollOut + yawOut;
	motorSE = throttleOut - pitchOut - rollOut - yawOut;

	float motor[4] = {motorNW,motorNE,motorSW,motorSE};
	//Adjust motor outputs by rpm
	/*
	motor[0] = (motor[0] - 1100.0) * 10167.0 / 10167.0 + 1100.0;
	motor[1] = (motor[1] - 1100.0) * 10167.0 / 10013.0 + 1100.0;
	motor[2] = (motor[2] - 1100.0) * 10167.0 / 11747.0 + 1100.0;
	motor[3] = (motor[3] - 1100.0) * 10167.0 / 11489.0 + 1100.0;
	*/

	//Safety : bound motor outputs from 1100 - 1800
	for (int i = 0; i < 4; i++) {
	  if (motor[i] > 1800.0) {
		  motor[i] = 1800.0;
	  }
	  if (motor[i] < 1100.0) {
		  motor[i] = 1100.0;
	  }
	  //Set values to minimum if switched to Disarm
	  if (duty_cycle[0] < 490) {
		  motor[i] = 1100.0;
	  }
	}



	  //Output to motorE
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,motor[0]);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,motor[1]);
	__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,motor[2]);
	__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,motor[3]);

	  //---------------------------- Read Data------------------------------------------//
	  //Read BMP390 Data
	//BMP_390_AP BMP_data = BMP_390_Read_Data();

	// Send to LoRa
	g_telem.pitch = angles.pitch; //- pitchOffset_g;
	g_telem.roll  = angles.roll;  //- rollOffset_g;
	g_telem.yaw   = angles.yaw;   //- yawOffset_g;
	g_telem.motor[0] = motor[0];
	g_telem.motor[1] = motor[1];
	g_telem.motor[2] = motor[2];
	g_telem.motor[3] = motor[3];
	g_telem.pid[0] = pitchP; g_telem.pid[1] = pitchI; g_telem.pid[2] = pitchD;
	g_telem.pid[3] = rollP;  g_telem.pid[4] = rollI;  g_telem.pid[5] = rollD;
	g_telem.pid[6] = yawP;   g_telem.pid[7] = yawI;   g_telem.pid[8] = yawD;
	g_telem.mixer[0] = throttleOut;
	g_telem.mixer[1] = pitchOut;
	g_telem.mixer[2] = rollOut;
	g_telem.mixer[3] = yawOut;
	g_telem.duty_cycle[0] = duty_cycle[0];
	g_telem.duty_cycle[1] = duty_cycle[1];
	g_telem.duty_cycle[2] = duty_cycle[2];
	g_telem.duty_cycle[3] = duty_cycle[3];



  }
  /* USER CODE END StartFlightTask */
}

/* USER CODE BEGIN Header_StartTelemRxTask */
/**
* @brief Function implementing the TelemRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTelemRxTask */
void StartTelemRxTask(void *argument)
{
  /* USER CODE BEGIN StartTelemRxTask */

	/* Start RX DMA from inside the task — after kernel is running */
	HAL_UART_Receive_DMA(&huart8, lora_rx_dma_buf, LORA_RX_DMA_SIZE);
	__HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);

  for(;;)
  {
	telem_rx_count++;
    osSemaphoreAcquire(loraRxSemHandle, osWaitForever);

    if (lora_rx_line_len == 0 || lora_rx_line_len >= LORA_RX_LINE_MAX) continue;

    memcpy(lora_rx_line, lora_rx_line2, lora_rx_line_len);
    lora_rx_line[lora_rx_line_len] = 0;
    LoRa_ProcessLine(lora_rx_line);

    /* If a +RCV line was just parsed, lora_new_payload is set */
    if (lora_new_payload) {
      lora_new_payload = 0;

      uint8_t n = 0;
      if (LoRa_Parse_SUNNY_Ping(lora_payload, &n)) {
        /* Defer the ACK — the TX scheduler will send it on the next pass */
        last_sunny_n    = n;
        last_sunny_addr = 1;     /* TODO: capture real sender from LoRa_RX_Parse */
        mari_ping_pending = 1;
      }
      /* Other commands could be parsed here in the future:
       * arming, PID gain updates, mission commands, etc.*/
    }
  }
  /* USER CODE END StartTelemRxTask */
}

/* USER CODE BEGIN Header_StartHousekeepingTask */
/**
* @brief Function implementing the HousekeepingTas thread.
* @param argument: Not used
* @retval None
*/
volatile uint16_t bno_rate = 0;
/* USER CODE END Header_StartHousekeepingTask */
void StartHousekeepingTask(void *argument)
{
  /* USER CODE BEGIN StartHousekeepingTask */
	uint32_t tick = osKernelGetTickCount();

	uint32_t last_rate_print = 0;
	uint32_t last_stats      = 0;

	uint32_t last_flight     = 0;
	uint32_t last_telem_tx   = 0;
	uint32_t last_telem_rx   = 0;
	uint32_t last_hk         = 0;

	uint32_t last_bno = 0;

	for(;;)
	{


	tick += 100;                          // 10 Hz
	osDelayUntil(tick);
	housekeeping_count++;
	uint32_t now = HAL_GetTick();



	// Time calculation
	/* Every 1 second: compute rates */
	if (now - last_rate_print >= 1000) {
	  flight_rate       = flight_count       - last_flight;
	  telem_tx_rate     = telem_tx_count     - last_telem_tx;
	  telem_rx_rate     = telem_rx_count     - last_telem_rx;
	  housekeeping_rate = housekeeping_count - last_hk;

	  bno_rate = intn_count - last_bno;     /* edges/sec = Hz */

	  last_flight     = flight_count;
	  last_telem_tx   = telem_tx_count;
	  last_telem_rx   = telem_rx_count;
	  last_hk         = housekeeping_count;
	  last_rate_print = now;

	  last_bno = intn_count;
	}

	/* Every 2 seconds: refresh task CPU stats */
	if (now - last_stats >= 2000) {
	  UpdateTaskStats();
	  last_stats = now;
	}


	//    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10); // Debug

	/* Battery */
	battery_voltage = Read_Battery_Voltage();

	if (battery_state == 2) {
		if (battery_voltage < GREEN_OFF) battery_state = 1;
	} else if (battery_state == 1) {
		if (battery_voltage >= GREEN_ON) battery_state = 2;
		else if (battery_voltage < YELLOW_ON) battery_state = 0;
	} else {
		if (battery_voltage >= YELLOW_OFF) battery_state = 1;
	}

	if (battery_state == 2) {
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	} else if (battery_state == 1) {
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	} else {
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	}

	g_telem.battery_state = battery_state;

	/* Barometer (I2C1) */
	BMP_581_AP bmp = BMP_581_Read_Data();
	g_telem.baro_temp     = bmp.temperature;
	g_telem.baro_press_pa = bmp.pressure;

	}
  /* USER CODE END StartHousekeepingTask */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
