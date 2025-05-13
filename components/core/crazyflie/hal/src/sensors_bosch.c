#define DEBUG_MODULE "IMU"

#include "sensors_bosch.h"

#include <math.h>
#include "imu.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "system.h"
#include "configblock.h"
#include "param.h"
#include "nvicconf.h"
#include "ledseq.h"
#include "sound.h"
#include "filter.h"
#include "debug_cf.h"

/* Bosch Sensortec Drivers */
#include "bmm150.h"
#include "bmp280.h"
#include "bstdr_comm_support.h"
#include "static_mem.h"
#include "estimator.h"
#include "bmi270.h"
#include "config.h"
#include "stm32_legacy.h"
#include "bim270_common.h"
#include "i2cdev.h"

#include "zranger2.h"
#include "flowdeck_v1v2.h"
#include "crtp_commander.h"

#define SENSORS_READ_RATE_HZ 1000
#define SENSORS_STARTUP_TIME_MS 1000
#define SENSORS_READ_BARO_HZ 50
#define SENSORS_READ_MAG_HZ 20
#define SENSORS_DELAY_BARO (SENSORS_READ_RATE_HZ / SENSORS_READ_BARO_HZ)
#define SENSORS_DELAY_MAG (SENSORS_READ_RATE_HZ / SENSORS_READ_MAG_HZ)

/* calculate constants */
/* BMI270 */
#define SENSORS_BMI270_GYRO_FS_CFG BMI2_GYR_RANGE_2000
#define SENSORS_BMI270_DEG_PER_LSB_CFG (2.0f * 2000.0f) / 65536.0f

#define SENSORS_BMI270_ACCEL_CFG 16
#define SENSORS_BMI270_ACCEL_FS_CFG BMI2_ACC_RANGE_16G
#define SENSORS_BMI270_G_PER_LSB_CFG (2.0f * (float)SENSORS_BMI270_ACCEL_CFG) / 65536.0f
#define SENSORS_BMI270_1G_IN_LSB 65536 / SENSORS_BMI270_ACCEL_CFG / 2
#define SENSORS_ACC_SCALE_SAMPLES  200

// #define SENSORS_ENABLE_RANGE_VL53LX
// #define SENSORS_ENABLE_FLOW_PMW3901
#define SENSORS_ENABLE_PRESSURE_BMP280
#define SENSORS_ENABLE_MAG_BMM150

#ifdef SENSORS_ENABLE_RANGE_VL53LX
static bool isVl53l1xPresent = false;
#endif

#ifdef SENSORS_ENABLE_FLOW_PMW3901
static bool isPmw3901Present = false;
#endif

/* BMI055 */
// #define SENSORS_BMI055_GYRO_FS_CFG BMI055_GYRO_RANGE_2000_DPS
// #define SENSORS_BMI055_DEG_PER_LSB_CFG (2.0f * 2000.0f) / 65536.0f

// #define SENSORS_BMI055_ACCEL_CFG 16
// #define SENSORS_BMI055_ACCEL_FS_CFG BMI055_ACCEL_RANGE_16G
// #define SENSORS_BMI055_G_PER_LSB_CFG (2.0f * (float)SENSORS_BMI055_ACCEL_CFG) / 65536.0f
// #define SENSORS_BMI055_1G_IN_LSB (65536 / SENSORS_BMI055_ACCEL_CFG / 2)

/* BMI270 */
// #define SENSORS_BMI270_GYRO_FS_CFG BMI270_GYRO_RANGE_2000_DPS
// #define SENSORS_BMI270_DEG_PER_LSB_CFG (2.0f * 2000.0f) / 65536.0f

// #define SENSORS_BMI270_ACCEL_CFG 24
// #define SENSORS_BMI270_ACCEL_FS_CFG BMI270_ACCEL_RANGE_24G
// #define SENSORS_BMI270_G_PER_LSB_CFG (2.0f * (float)SENSORS_BMI270_ACCEL_CFG) / 65536.0f
// #define SENSORS_BMI270_1G_IN_LSB (65536 / SENSORS_BMI270_ACCEL_CFG / 2)

#define SENSORS_VARIANCE_MAN_TEST_TIMEOUT M2T(1000) // Timeout in ms
#define SENSORS_MAN_TEST_LEVEL_MAX 5.0f             // Max degrees off

#define GYRO_NBR_OF_AXES 3
#define GYRO_MIN_BIAS_TIMEOUT_MS M2T(1 * 1000)

// Number of samples used in variance calculation. Changing this effects the threshold
#define SENSORS_NBR_OF_BIAS_SAMPLES 512

// Variance threshold to take zero bias for gyro
#define GYRO_VARIANCE_BASE 2000
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)

#define SENSORS_TAKE_ACCEL_BIAS

/* available sensors */
#define SENSORS_BMI055 0x01
#define SENSORS_BMI270 0x02
#define SENSORS_BMM150 0x08
#define SENSORS_BMP280 0x10

#define ACCEL UINT8_C(0x00)
#define GYRO UINT8_C(0x01)

/* configure sensor's use
 * PRIMARIES are the sensors which are used for stabilization
 * SECONDARIES are only added to the log if compilations is
 * done with CFLAGS += -DLOG_SEC_IMU */
static uint8_t gyroPrimInUse = SENSORS_BMI270;
static uint8_t accelPrimInUse = SENSORS_BMI270;
// static uint8_t baroPrimInUse =          SENSORS_BMP280;
#ifdef LOG_SEC_IMU
static uint8_t gyroSecInUse = SENSORS_BMI270;
static uint8_t accelSecInUse = SENSORS_BMI270;
#endif

typedef struct
{
  Axis3f     bias;
  Axis3f     variance;
  Axis3f     mean;
  bool       isBiasValueFound;
  bool       isBufferFilled;
  Axis3i16*  bufHead;
  Axis3i16   buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;

/* initialize necessary variables */
static struct bmi2_dev bmi270Dev;
static struct bmp280_t bmp280Dev;
static struct bmm150_dev bmm150Dev;

static xQueueHandle accelPrimDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelPrimDataQueue, 1, sizeof(Axis3f));
static xQueueHandle gyroPrimDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroPrimDataQueue, 1, sizeof(Axis3f));
static xQueueHandle baroPrimDataQueue;
STATIC_MEM_QUEUE_ALLOC(baroPrimDataQueue, 1, sizeof(Axis3f));
static xQueueHandle magPrimDataQueue;
STATIC_MEM_QUEUE_ALLOC(magPrimDataQueue, 1, sizeof(baro_t));

static xSemaphoreHandle dataReady;
static StaticSemaphore_t dataReadyBuffer;

static bool isInit = false;
static bool allSensorsAreCalibrated = false;
static sensorData_t sensorData;

static bool isBarometerPresent = false;
static bool isMagnetometerPresent = false;
static uint8_t baroMeasDelayMin = SENSORS_DELAY_BARO;


static Axis3i16 gyroRaw;
static Axis3i16 accelRaw;
NO_DMA_CCM_SAFE_ZERO_INIT static BiasObj gyroBiasRunning;
static Axis3f gyroBias;
#if defined(SENSORS_GYRO_BIAS_CALCULATE_STDDEV) && defined (GYRO_BIAS_LIGHT_WEIGHT)
static Axis3f gyroBiasStdDev;
#endif
static bool gyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1;
static bool accScaleFound = false;
static uint32_t accScaleSumCount = 0;


#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];
static void applyAxis3fLpf(lpf2pData *data, Axis3f* in);

// Pre-calculated values for accelerometer alignment
static float cosPitch;
static float sinPitch;
static float cosRoll;
static float sinRoll;

static void sensorsDeviceInit(void);
static void sensorsTaskInit(void);
static void sensorsTask(void *param);
static void sensorsScaleBaro(baro_t *baroScaled, float pressure, float temperature);
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);

static void sensorsAccIIRLPFilter(Axis3i16 *in, Axis3i16 *out, Axis3i32 *storedValues, int32_t attenuation);
static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out);


#ifdef GYRO_GYRO_BIAS_LIGHT_WEIGHT
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
#else
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz,  Axis3f *gyroBiasOut);
#endif
static bool processAccScale(int16_t ax, int16_t ay, int16_t az);
static void sensorsBiasObjInit(BiasObj* bias);
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut);
static void sensorsCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut);
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z);
static bool sensorsFindBiasValue(BiasObj* bias);
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out);

STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE);

static void sensorsBiasObjInit(BiasObj* bias)
{
  bias->isBufferFilled = false;
  bias->bufHead = bias->buffer;
}

void sensorsBoschInit(void)
{
  if (isInit)
  {
    return;
  }

  dataReady = xSemaphoreCreateBinaryStatic(&dataReadyBuffer);

  sensorsBiasObjInit(&gyroBiasRunning);
  sensorsDeviceInit();
  sensorsTaskInit();

  isInit = true;
}

static void sensorsDeviceInit(void)
{
  if (isInit)
    return;

  bstdr_ret_t rslt;
  isBarometerPresent = false;

  // Wait for sensors to startup
  vTaskDelay(M2T(SENSORS_STARTUP_TIME_MS));

  i2cdevInit(I2C0_DEV);
  bmi2_spi_init();
  bmi270Dev.intf = BMI2_SPI_INTF;
  bmi270Dev.read = bmi2_spi_read;
  bmi270Dev.write = bmi2_spi_write;
  bmi270Dev.delay_us = bstdr_us_delay;
  bmi270Dev.dummy_byte = 1;
  bmi270Dev.gyro_en = 1;
  rslt = bmi270_init(&bmi270Dev); // initialize the device
  if (rslt == BSTDR_OK)
  {
    DEBUG_PRINTI("BMI270 SPI connection [OK].\n");
    struct bmi2_sens_config config[2];

    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;
    rslt |= bmi2_get_sensor_config(config, 2, &bmi270Dev);
    bmi2_error_codes_print_result(rslt);
    config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_800HZ;
    config[GYRO].cfg.gyr.range = SENSORS_BMI270_GYRO_FS_CFG;
    config[GYRO].cfg.gyr.bwp = BMI2_GYR_OSR4_MODE;
    config[GYRO].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

    config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_1600HZ;
    config[ACCEL].cfg.acc.range = SENSORS_BMI270_ACCEL_FS_CFG;
    config[ACCEL].cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;
    config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    rslt |= bmi2_set_sensor_config(config, 2, &bmi270Dev);

    uint8_t sensor_list[2] = {BMI2_ACCEL, BMI2_GYRO};
    rslt |= bmi2_sensor_enable(sensor_list, 2, &bmi270Dev);

    if (rslt == BSTDR_OK)
    {
      DEBUG_PRINTI("BMI270 SPI sensor enable [OK].\n");
    }
    else
    {
      DEBUG_PRINTW("BMI270 SPI sensor enable [FAIL].\n");
    }
    struct bmi2_sens_data imu_data;
    rslt |= bmi2_get_sensor_data(&imu_data, &bmi270Dev);
  }
  else
  {
    DEBUG_PRINTW("BMI270 SPI connection [FAIL].\n");
  }

#ifdef SENSORS_ENABLE_MAG_BMM150
  /* BMM150 */
  rslt = BSTDR_E_GEN_ERROR;

  /* Sensor interface over I2C */
  bmm150Dev.id = BMM150_DEFAULT_I2C_ADDRESS;
  bmm150Dev.interface = BMM150_I2C_INTF;
  bmm150Dev.read = (bmm150_com_fptr_t)bstdr_burst_read;
  bmm150Dev.write = (bmm150_com_fptr_t)bstdr_burst_write;
  bmm150Dev.delay_ms = bstdr_ms_delay;

  rslt = bmm150_init(&bmm150Dev);

  if (rslt == BMM150_OK)
  {
    bmm150Dev.settings.pwr_mode = BMM150_NORMAL_MODE;
    rslt |= bmm150_set_op_mode(&bmm150Dev);
    bmm150Dev.settings.preset_mode = BMM150_PRESETMODE_HIGHACCURACY;
    rslt |= bmm150_set_presetmode(&bmm150Dev);

    DEBUG_PRINTI("BMM150 I2C connection [OK].\n");
    isMagnetometerPresent = true;
  }
#endif

#ifdef SENSORS_ENABLE_PRESSURE_BMP280
  /* BMP280 */
  rslt = BSTDR_E_GEN_ERROR;

  bmp280Dev.bus_read = bstdr_burst_read;
  bmp280Dev.bus_write = bstdr_burst_write;
  bmp280Dev.delay_ms = bstdr_ms_delay;
  bmp280Dev.dev_addr = BMP280_I2C_ADDRESS1;
  rslt = bmp280_init(&bmp280Dev);
  if (rslt == BSTDR_OK)
  {
    isBarometerPresent = true;
    DEBUG_PRINTI("BMP280 I2C connection [OK].\n");
    bmp280_set_filter(BMP280_FILTER_COEFF_OFF);
    bmp280_set_oversamp_temperature(BMP280_OVERSAMP_2X);
    bmp280_set_oversamp_pressure(BMP280_OVERSAMP_8X);
    bmp280_set_power_mode(BMP280_NORMAL_MODE);
    bmp280Dev.delay_ms(20); // wait before first read out
    // read out data
    int32_t v_temp_s32;
    uint32_t v_pres_u32;
    bmp280_read_pressure_temperature(&v_pres_u32, &v_temp_s32);
    baroMeasDelayMin = SENSORS_DELAY_BARO;
  }
#endif
#ifdef SENSORS_ENABLE_RANGE_VL53LX
  zRanger2Init();

  if (zRanger2Test() == true)
  {
    isVl53l1xPresent = true;
    DEBUG_PRINTI("VL53L1X I2C connection [OK].\n");
  }
  else
  {
    // TODO: Should sensor test fail hard if no connection
    DEBUG_PRINTW("VL53L1X I2C connection [FAIL].\n");
  }

#endif

#ifdef SENSORS_ENABLE_FLOW_PMW3901
  flowdeck2Init();

  if (flowdeck2Test() == true)
  {
    isPmw3901Present = true;
    setCommandermode(POSHOLD_MODE);
    DEBUG_PRINTI("PMW3901 SPI connection [OK].\n");
  }
  else
  {
    // TODO: Should sensor test fail hard if no connection
    DEBUG_PRINTW("PMW3901 SPI connection [FAIL].\n");
  }
#endif
  for (uint8_t i = 0; i < 3; i++)
  {
    lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
    lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
  }

  cosPitch = cosf(configblockGetCalibPitch() * (float)M_PI / 180);
  sinPitch = sinf(configblockGetCalibPitch() * (float)M_PI / 180);
  cosRoll = cosf(configblockGetCalibRoll() * (float)M_PI / 180);
  sinRoll = sinf(configblockGetCalibRoll() * (float)M_PI / 180);

  isInit = true;
}

static void sensorsTaskInit(void)
{
  accelPrimDataQueue = STATIC_MEM_QUEUE_CREATE(accelPrimDataQueue);
  gyroPrimDataQueue = STATIC_MEM_QUEUE_CREATE(gyroPrimDataQueue);
#ifdef LOG_SEC_IMU
  accelSecDataQueue = STATIC_MEM_QUEUE_CREATE(accelSecDataQueue);
  gyroSecDataQueue = STATIC_MEM_QUEUE_CREATE(gyroSecDataQueue);
#endif
  magPrimDataQueue = STATIC_MEM_QUEUE_CREATE(magPrimDataQueue);
  baroPrimDataQueue = STATIC_MEM_QUEUE_CREATE(baroPrimDataQueue);

  STATIC_MEM_TASK_CREATE(sensorsTask, sensorsTask, SENSORS_TASK_NAME, NULL, SENSORS_TASK_PRI);
}

static void sensorsGyroGet(Axis3i16 *dataOut)
{
  struct bmi2_sens_data imu_data;
  bmi2_get_sensor_data(&imu_data, pBmi270);
  dataOut->x = imu_data.gyr.y;
  dataOut->y = -imu_data.gyr.x;
  dataOut->z = imu_data.gyr.z;
}

static void sensorsAccelGet(Axis3i16 *dataOut)
{
  struct bmi2_sens_data imu_data;
  bmi2_get_sensor_data(&imu_data, pBmi270);
  dataOut->x = imu_data.acc.y;
  dataOut->y = -imu_data.acc.x;
  dataOut->z = imu_data.acc.z;
}

static void sensorsTask(void *param)
{
  systemWaitStart();
  uint32_t lastWakeTime = xTaskGetTickCount();

  Axis3f accScaled;
  while (1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(SENSORS_READ_RATE_HZ));
    sensorData.interruptTimestamp = (uint64_t)esp_timer_get_time();
    // sensorsGyroGet(&gyroRaw);
    // sensorsAccelGet(&accelRaw);
    struct bmi2_sens_data imu_data;
    bmi2_get_sensor_data(&imu_data, pBmi270);
    gyroRaw.x = imu_data.gyr.y;
    gyroRaw.y = -imu_data.gyr.x;
    gyroRaw.z = imu_data.gyr.z;
    accelRaw.x = imu_data.acc.y;
    accelRaw.y = -imu_data.acc.x;
    accelRaw.z = imu_data.acc.z;
#ifdef GYRO_BIAS_LIGHT_WEIGHT
    gyroBiasFound = processGyroBiasNoBuffer(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#else
    gyroBiasFound = processGyroBias(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#endif
    if (gyroBiasFound)
    {
      processAccScale(accelRaw.x, accelRaw.y, accelRaw.z);
    }
    /* Gyro */
    sensorData.gyro.x = (gyroRaw.x - gyroBias.x) * SENSORS_BMI270_DEG_PER_LSB_CFG;
    sensorData.gyro.y = (gyroRaw.y - gyroBias.y) * SENSORS_BMI270_DEG_PER_LSB_CFG;
    sensorData.gyro.z = (gyroRaw.z - gyroBias.z) * SENSORS_BMI270_DEG_PER_LSB_CFG;
    applyAxis3fLpf((lpf2pData *)(&gyroLpf), &sensorData.gyro);

    /* Acelerometer */
    accScaled.x = accelRaw.x * SENSORS_BMI270_G_PER_LSB_CFG / accScale;
    accScaled.y = accelRaw.y * SENSORS_BMI270_G_PER_LSB_CFG / accScale;
    accScaled.z = accelRaw.z * SENSORS_BMI270_G_PER_LSB_CFG / accScale;
    sensorsAccAlignToGravity(&accScaled, &sensorData.acc);
    applyAxis3fLpf((lpf2pData *)(&accLpf), &sensorData.acc);
    if (isMagnetometerPresent)
    {
      static uint8_t magMeasDelay = SENSORS_DELAY_MAG;

      if (--magMeasDelay == 0)
      {
        bmm150_read_mag_data(&bmm150Dev);
        sensorData.mag.x = bmm150Dev.data.x;
        sensorData.mag.y = bmm150Dev.data.y;
        sensorData.mag.z = bmm150Dev.data.z;
        magMeasDelay = SENSORS_DELAY_MAG;
      }
    }

    if (isBarometerPresent)
    {
      static uint8_t baroMeasDelay = SENSORS_DELAY_BARO;
      static int32_t v_temp_s32;
      static uint32_t v_pres_u32;
      static baro_t *baro280 = &sensorData.baro;

      if (--baroMeasDelay == 0)
      {
        bmp280_read_pressure_temperature(&v_pres_u32, &v_temp_s32);
        sensorsScaleBaro(baro280, (float)v_pres_u32, (float)v_temp_s32 / 100.0f);
        baroMeasDelay = baroMeasDelayMin;
      }
    }
    xQueueOverwrite(accelPrimDataQueue, &sensorData.acc);
    xQueueOverwrite(gyroPrimDataQueue, &sensorData.gyro);

    if (isBarometerPresent)
    {
      xQueueOverwrite(baroPrimDataQueue, &sensorData.baro);
    }

    if (isMagnetometerPresent)
    {
      xQueueOverwrite(magPrimDataQueue, &sensorData.mag);
    }

    xSemaphoreGive(dataReady);
  }
}

void sensorsBoschWaitDataReady(void)
{
  xSemaphoreTake(dataReady, portMAX_DELAY);
}

static bool gyroSelftest()
{
  bool testStatus = true;

  int i = 3;
  do
  {
    sensorsGyroGet(&gyroRaw);
  } while (i-- > 0);

  if ((gyroRaw.x == 0 && gyroRaw.y == 0 && gyroRaw.z == 0))
  {
    DEBUG_PRINTW("BMI270 gyro returning x=0 y=0 z=0 [FAILED]\n");
    testStatus = false;
  }
  return testStatus;
}

/**
 * Calculates accelerometer scale out of SENSORS_ACC_SCALE_SAMPLES samples. Should be called when
 * platform is stable.
 */
static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
  if (!accScaleFound)
  {
    accScaleSum += sqrtf(powf(ax * SENSORS_BMI270_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_BMI270_G_PER_LSB_CFG, 2) + powf(az * SENSORS_BMI270_G_PER_LSB_CFG, 2));
    accScaleSumCount++;

    if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
    {
      accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
      accScaleFound = true;
    }
  }

  return accScaleFound;
}

#ifdef GYRO_BIAS_LIGHT_WEIGHT

#define SENSORS_BIAS_SAMPLES 1000
/**
 * Calculates the bias out of the first SENSORS_BIAS_SAMPLES gathered. Requires no buffer
 * but needs platform to be stable during startup.
 */
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
  static uint32_t gyroBiasSampleCount = 0;
  static bool gyroBiasNoBuffFound = false;
  static Axis3i64 gyroBiasSampleSum;
  static Axis3i64 gyroBiasSampleSumSquares;

  if (!gyroBiasNoBuffFound)
  {
    // If the gyro has not yet been calibrated:
    // Add the current sample to the running mean and variance
    gyroBiasSampleSum.x += gx;
    gyroBiasSampleSum.y += gy;
    gyroBiasSampleSum.z += gz;
#ifdef SENSORS_GYRO_BIAS_CALCULATE_STDDEV
    gyroBiasSampleSumSquares.x += gx * gx;
    gyroBiasSampleSumSquares.y += gy * gy;
    gyroBiasSampleSumSquares.z += gz * gz;
#endif
    gyroBiasSampleCount += 1;

    // If we then have enough samples, calculate the mean and standard deviation
    if (gyroBiasSampleCount == SENSORS_BIAS_SAMPLES)
    {
      gyroBiasOut->x = (float)(gyroBiasSampleSum.x) / SENSORS_BIAS_SAMPLES;
      gyroBiasOut->y = (float)(gyroBiasSampleSum.y) / SENSORS_BIAS_SAMPLES;
      gyroBiasOut->z = (float)(gyroBiasSampleSum.z) / SENSORS_BIAS_SAMPLES;

#ifdef SENSORS_GYRO_BIAS_CALCULATE_STDDEV
      gyroBiasStdDev.x = sqrtf((float)(gyroBiasSampleSumSquares.x) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->x * gyroBiasOut->x));
      gyroBiasStdDev.y = sqrtf((float)(gyroBiasSampleSumSquares.y) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->y * gyroBiasOut->y));
      gyroBiasStdDev.z = sqrtf((float)(gyroBiasSampleSumSquares.z) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->z * gyroBiasOut->z));
#endif
      gyroBiasNoBuffFound = true;
    }
  }

  return gyroBiasNoBuffFound;
}
#else
/**
 * Calculates the bias first when the gyro variance is below threshold. Requires a buffer
 * but calibrates platform first when it is stable.
 */
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
  sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

  if (!gyroBiasRunning.isBiasValueFound)
  {
    sensorsFindBiasValue(&gyroBiasRunning);
    if (gyroBiasRunning.isBiasValueFound)
    {
      soundSetEffect(SND_CALIB);
      ledseqRun(&seq_calibrated);
    }
  }

  gyroBiasOut->x = gyroBiasRunning.bias.x;
  gyroBiasOut->y = gyroBiasRunning.bias.y;
  gyroBiasOut->z = gyroBiasRunning.bias.z;

  return gyroBiasRunning.isBiasValueFound;
}
#endif


static void sensorsScaleBaro(baro_t *baroScaled, float pressure,
                             float temperature)
{
  baroScaled->pressure = pressure * 0.01f;
  baroScaled->temperature = temperature;
  baroScaled->asl = ((powf((1015.7f / baroScaled->pressure), 0.1902630958f) - 1.0f) * (25.0f + 273.15f)) / 0.0065f;
}

bool sensorsBoschReadGyro(Axis3f *gyro)
{
  return (pdTRUE == xQueueReceive(gyroPrimDataQueue, gyro, 0));
}

#ifdef LOG_SEC_IMU
bool sensorsReadGyroSec(Axis3f *gyro)
{
  return (pdTRUE == xQueueReceive(gyroSecDataQueue, gyro, 0));
}

bool sensorsReadAccSec(Axis3f *acc)
{
  return (pdTRUE == xQueueReceive(accelSecDataQueue, acc, 0));
}
#endif

bool sensorsBoschReadAcc(Axis3f *acc)
{
  return (pdTRUE == xQueueReceive(accelPrimDataQueue, acc, 0));
}

bool sensorsBoschReadMag(Axis3f *mag)
{
  return (pdTRUE == xQueueReceive(magPrimDataQueue, mag, 0));
}

bool sensorsBoschReadBaro(baro_t *baro)
{
  return (pdTRUE == xQueueReceive(baroPrimDataQueue, baro, 0));
}

void sensorsBoschAcquire(sensorData_t *sensors, const uint32_t tick)
{
  sensorsReadGyro(&sensors->gyro);
  sensorsReadAcc(&sensors->acc);
  sensorsReadMag(&sensors->mag);
  sensorsReadBaro(&sensors->baro);
#ifdef LOG_SEC_IMU
  sensorsReadGyroSec(&sensors->gyroSec);
  sensorsReadAccSec(&sensors->accSec);
#endif
}

bool sensorsBoschAreCalibrated()
{
  return gyroBiasFound;
}

static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut)
{
  uint32_t i;
  int64_t sum[GYRO_NBR_OF_AXES] = {0};
  int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
    sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
    sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
    sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
  }

  varOut->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
  varOut->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
  varOut->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

  meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

static void __attribute__((used)) sensorsCalculateBiasMean(BiasObj *bias, Axis3i32 *meanOut)
{
  uint32_t i;
  int32_t sum[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
  }

  meanOut->x = sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->y = sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->z = sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z)
{
  bias->bufHead->x = x;
  bias->bufHead->y = y;
  bias->bufHead->z = z;
  bias->bufHead++;

  if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES])
  {
    bias->bufHead = bias->buffer;
    bias->isBufferFilled = true;
  }
}

static bool sensorsFindBiasValue(BiasObj *bias)
{
  static int32_t varianceSampleTime;
  bool foundBias = false;

  if (bias->isBufferFilled)
  {
    sensorsCalculateVarianceAndMean(bias, &bias->variance, &bias->mean);

    if (bias->variance.x < GYRO_VARIANCE_THRESHOLD_X &&
        bias->variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
        bias->variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
        (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount()))
    {
      varianceSampleTime = xTaskGetTickCount();
      bias->bias.x = bias->mean.x;
      bias->bias.y = bias->mean.y;
      bias->bias.z = bias->mean.z;
      foundBias = true;
      bias->isBiasValueFound = true;
    }
  }

  return foundBias;
}

bool sensorsBoschTest(void)
{
  bool testStatus = true;

  if (!isInit)
  {
    DEBUG_PRINTW("Uninitialized\n");
    testStatus = false;
  }

  if (! gyroSelftest())
  {
    testStatus = false;
  }

  return testStatus;
}

bool sensorsBoschManufacturingTest(void)
{
  return true;
}

bool sensorsHasBarometer(void)
{
  return isBarometerPresent;
}

bool sensorsHasMangnetometer(void)
{
  return isMagnetometerPresent;
}


/**
 * Compensate for a miss-aligned accelerometer. It uses the trim
 * data gathered from the UI and written in the config-block to
 * rotate the accelerometer to be aligned with gravity.
 */
static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out)
{
  Axis3f rx;
  Axis3f ry;

  // Rotate around x-axis
  rx.x = in->x;
  rx.y = in->y * cosRoll - in->z * sinRoll;
  rx.z = in->y * sinRoll + in->z * cosRoll;

  // Rotate around y-axis
  ry.x = rx.x * cosPitch - rx.z * sinPitch;
  ry.y = rx.y;
  ry.z = -rx.x * sinPitch + rx.z * cosPitch;

  out->x = ry.x;
  out->y = ry.y;
  out->z = ry.z;
}

void sensorsBoschSetAccMode(accModes accMode)
{
  struct bmi2_sens_config config[2];
  config[ACCEL].type = BMI2_ACCEL;
  config[GYRO].type = BMI2_GYRO;
  bstdr_ret_t rslt;
  rslt = bmi2_get_sensor_config(config, 2, &bmi270Dev);
  if (rslt != BSTDR_OK)
  {
    DEBUG_PRINTW("BMI270 get sensor config [FAIL].\n");
  }
  for (uint8_t i = 0; i < 3; i++)
  {
    lpf2pInit(&accLpf[i], 1000, 500);
  }
  switch (accMode)
  {
  case ACC_MODE_PROPTEST:
    config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_1600HZ;
    config[ACCEL].cfg.acc.range = SENSORS_BMI270_ACCEL_FS_CFG;
    config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
    break;
  case ACC_MODE_FLIGHT:
  default:
    config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_1600HZ;
    config[ACCEL].cfg.acc.range = SENSORS_BMI270_ACCEL_FS_CFG;
    config[ACCEL].cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;
    break;
  }
  rslt = bmi2_set_sensor_config(config, 2, &bmi270Dev);
  if (rslt != BSTDR_OK)
  {
    DEBUG_PRINTW("BMI270 set sensor config [FAIL].\n");
  }
  for (uint8_t i = 0; i < 3; i++)
  {
    lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
  }
}

static void applyAxis3fLpf(lpf2pData *data, Axis3f *in)
{
  for (uint8_t i = 0; i < 3; i++)
  {
    in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
  }
}

PARAM_GROUP_START(imu_sensors)
PARAM_ADD(PARAM_UINT8, BoschGyrSel, &gyroPrimInUse)
PARAM_ADD(PARAM_UINT8, BoschAccSel, &accelPrimInUse)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, BMM150, &isMagnetometerPresent)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, BMP285, &isBarometerPresent)
PARAM_GROUP_STOP(imu_sensors)
