
/*
 * This file is part of VL53LX Platform
 *
 * Copyright (c) 2016, STMicroelectronics - All Rights Reserved
 *
 * License terms: BSD 3-clause "New" or "Revised" License.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <vl53lx_platform.h>
#ifndef SMALL_FOOTPRINT
#include <vl53lx_platform_ipp.h>
#endif
#include <vl53lx_platform_log.h>
#include <vl53lx_api.h>

// #include "stm32xxx_hal.h"
#include <time.h>
#include <math.h>

// #include "esp_log.h"
#include "driver/i2c.h"

#define DEBUG_MODULE "VLX3"
#include "debug_cf.h"

#ifdef VL53LX_LOG_ENABLE
#define trace_print(level, ...) VL53LX_trace_print_module_function(VL53LX_TRACE_MODULE_PLATFORM, level, VL53LX_TRACE_FUNCTION_NONE, ##__VA_ARGS__)
#define trace_i2c(...) VL53LX_trace_print_module_function(VL53LX_TRACE_MODULE_NONE, VL53LX_TRACE_LEVEL_NONE, VL53LX_TRACE_FUNCTION_I2C, ##__VA_ARGS__)
#endif



bool vl53l3xInit(VL53LX_Dev_t *pdev, I2C_Dev *I2cHandle)
{
  VL53LX_Error status = VL53LX_ERROR_NONE;

  pdev->I2cDevAddr = 0x29;
  pdev->i2c_slave_address = 0x29;
  pdev->I2Cx = I2cHandle;
  i2cdevInit(pdev->I2Cx);

  uint8_t byteData;
  uint16_t wordData;
  VL53LX_RdByte(pdev, 0x010F, &byteData);
  DEBUG_PRINT( "VL53L1X Model_ID: %02X\n\r", byteData);
  VL53LX_RdByte(pdev, 0x0110, &byteData);
  DEBUG_PRINT( "VL53L1X Module_Type: %02X\n\r", byteData);
  VL53LX_RdWord(pdev, 0x010F, &wordData);
  DEBUG_PRINT( "VL53L1X: %02X\n\r", wordData);

  status = VL53LX_WaitDeviceBooted(pdev);
  if (status == VL53LX_ERROR_NONE)
  {
	status = VL53LX_DataInit(pdev);

	if (status == VL53LX_ERROR_NONE)
	{
		status = VL53LX_StaticInit(pdev);
	}
  }

  return status == VL53LX_ERROR_NONE;
}

bool vl53lxTestConnection(VL53LX_Dev_t* pdev)
{
  VL53LX_DeviceInfo_t info;
  VL53LX_Error status = VL53LX_ERROR_NONE;

  status = VL53LX_GetDeviceInfo(pdev, &info);

  return status == VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WriteMulti(VL53LX_Dev_t *pdev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    int status_int;
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    if (!i2cdevWrite16(pdev->I2Cx, pdev->I2cDevAddr, index, count, pdata))
    {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53LX_Error VL53LX_ReadMulti(VL53LX_Dev_t *pdev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    if (!i2cdevRead16(pdev->I2Cx, pdev->I2cDevAddr, index, count, pdata))
    {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

VL53LX_Error VL53LX_WrByte(VL53LX_Dev_t *pdev, uint16_t index, uint8_t data)
{
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    int32_t status_int;

    if (!i2cdevWrite16(pdev->I2Cx, pdev->I2cDevAddr, index, 1, &data))
    {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

VL53LX_Error VL53LX_WrWord(VL53LX_Dev_t *pdev, uint16_t index, uint16_t data)
{
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    int32_t status_int;

    uint8_t _I2CBuffer[2];
    _I2CBuffer[0] = data >> 8;
    _I2CBuffer[1] = data & 0x00FF;
    if (!i2cdevWrite16(pdev->I2Cx, pdev->I2cDevAddr, index, 2, (uint8_t *)_I2CBuffer))
    {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

VL53LX_Error VL53LX_WrDWord(VL53LX_Dev_t *pdev, uint16_t index, uint32_t data)
{
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    uint8_t _I2CBuffer[4];
    _I2CBuffer[0] = (data >> 24) & 0xFF;
    _I2CBuffer[1] = (data >> 16) & 0xFF;
    _I2CBuffer[2] = (data >> 8) & 0xFF;
    _I2CBuffer[3] = (data >> 0) & 0xFF;

    if (!i2cdevWrite16(pdev->I2Cx, pdev->I2cDevAddr, index, 4, (uint8_t *)_I2CBuffer))
    {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

VL53LX_Error VL53LX_UpdateByte(VL53LX_Dev_t *pdev, uint16_t index, uint8_t AndData, uint8_t OrData)
{
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    uint8_t data;

    Status = VL53LX_RdByte(pdev, index, &data);
    if (Status)
    {
        goto done;
    }
    data = (data & AndData) | OrData;
    Status = VL53LX_WrByte(pdev, index, data);
done:
    return Status;
}

VL53LX_Error VL53LX_RdByte(VL53LX_Dev_t *pdev, uint16_t index, uint8_t *data)
{
    VL53LX_Error Status = VL53LX_ERROR_NONE;
	if (!i2cdevRead16(pdev->I2Cx, pdev->I2cDevAddr, index, 1, data))
    {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

VL53LX_Error VL53LX_RdWord(VL53LX_Dev_t *pdev, uint16_t index, uint16_t *data)
{
    VL53LX_Error Status = VL53LX_ERROR_NONE;
	uint8_t _I2CBuffer[2];

	if (!i2cdevRead16(pdev->I2Cx, pdev->I2cDevAddr, index, 2, (uint8_t *)_I2CBuffer))
	{
		Status = VL53LX_ERROR_CONTROL_INTERFACE;
	}
	*data = ((uint16_t)_I2CBuffer[0] << 8) + (uint16_t)_I2CBuffer[1];
    return Status;
}

VL53LX_Error VL53LX_RdDWord(VL53LX_Dev_t *pdev, uint16_t index, uint32_t *data)
{
    VL53LX_Error Status = VL53LX_ERROR_NONE;
	uint8_t _I2CBuffer[4];
	if (!i2cdevRead16(pdev->I2Cx, pdev->I2cDevAddr, index, 4, (uint8_t *)_I2CBuffer))
	{
		Status = VL53LX_ERROR_CONTROL_INTERFACE;
	}
	*data = ((uint32_t)_I2CBuffer[0] << 24) + ((uint32_t)_I2CBuffer[1] << 16) + ((uint32_t)_I2CBuffer[2] << 8) + (uint32_t)_I2CBuffer[3];
    return Status;
}

VL53LX_Error VL53LX_GetTickCount(
    VL53LX_Dev_t *pdev,
    uint32_t *ptick_count_ms)
{

    /* Returns current tick count in [ms] */

    VL53LX_Error status = VL53LX_ERROR_NONE;

    //*ptick_count_ms = timeGetTime();
    *ptick_count_ms = 0;

#ifdef VL53LX_LOG_ENABLE
    trace_print(
        VL53LX_TRACE_LEVEL_DEBUG,
        "VL53LX_GetTickCount() = %5u ms;\n",
        *ptick_count_ms);
#endif

    return status;
}

#define trace_print(level, ...)                    \
    _LOG_TRACE_PRINT(VL53LX_TRACE_MODULE_PLATFORM, \
                     level, VL53LX_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

#define trace_i2c(...)                         \
    _LOG_TRACE_PRINT(VL53LX_TRACE_MODULE_NONE, \
                     VL53LX_TRACE_LEVEL_NONE, VL53LX_TRACE_FUNCTION_I2C, ##__VA_ARGS__)

VL53LX_Error VL53LX_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
    *ptimer_freq_hz = 0;

    trace_print(VL53LX_TRACE_LEVEL_INFO, "VL53LX_GetTimerFrequency: Freq : %dHz\n", *ptimer_freq_hz);
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitMs(VL53LX_Dev_t *pdev, int32_t wait_ms)
{
    (void)pdev;
    vTaskDelay(M2T(wait_ms));
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitUs(VL53LX_Dev_t *pdev, int32_t wait_us)
{
	uint32_t delay_ms = (wait_us + 900) / 1000;

	if(delay_ms == 0)
	{
	  delay_ms = 1;
	}

	vTaskDelay(M2T(delay_ms));
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitValueMaskEx(
    VL53LX_Dev_t *pdev,
    uint32_t timeout_ms,
    uint16_t index,
    uint8_t value,
    uint8_t mask,
    uint32_t poll_delay_ms)
{

    /*
     * Platform implementation of WaitValueMaskEx V2WReg script command
     *
     * WaitValueMaskEx(
     *          duration_ms,
     *          index,
     *          value,
     *          mask,
     *          poll_delay_ms);
     */

    VL53LX_Error status = VL53LX_ERROR_NONE;
    uint32_t start_time_ms = 0;
    uint32_t current_time_ms = 0;
    uint32_t polling_time_ms = 0;
    uint8_t byte_value = 0;
    uint8_t found = 0;
#ifdef VL53LX_LOG_ENABLE
    uint8_t trace_functions = VL53LX_TRACE_FUNCTION_NONE;
#endif

    char register_name[VL53LX_MAX_STRING_LENGTH];

    /* look up register name */
#ifdef PAL_EXTENDED
    VL53LX_get_register_name(
        index,
        register_name);
#else
    VL53LX_COPYSTRING(register_name, "");
#endif

    /* Output to I2C logger for FMT/DFT  */

    /*trace_i2c("WaitValueMaskEx(%5d, 0x%04X, 0x%02X, 0x%02X, %5d);\n",
                 timeout_ms, index, value, mask, poll_delay_ms); */
    trace_i2c("WaitValueMaskEx(%5d, %s, 0x%02X, 0x%02X, %5d);\n",
              timeout_ms, register_name, value, mask, poll_delay_ms);

    /* calculate time limit in absolute time */

    VL53LX_GetTickCount(pdev, &start_time_ms);

    /* remember current trace functions and temporarily disable
     * function logging
     */

#ifdef VL53LX_LOG_ENABLE
    trace_functions = VL53LX_get_trace_functions();
    VL53LX_set_trace_functions(VL53LX_TRACE_FUNCTION_NONE);
#endif

    /* wait until value is found, timeout reached on error occurred */

    while ((status == VL53LX_ERROR_NONE) &&
           (polling_time_ms < timeout_ms) &&
           (found == 0))
    {

        if (status == VL53LX_ERROR_NONE)
            status = VL53LX_RdByte(
                pdev,
                index,
                &byte_value);

        if ((byte_value & mask) == value)
            found = 1;

        if (status == VL53LX_ERROR_NONE &&
            found == 0 &&
            poll_delay_ms > 0)
            status = VL53LX_WaitMs(
                pdev,
                poll_delay_ms);

        /* Update polling time (Compare difference rather than absolute to
        negate 32bit wrap around issue) */
        VL53LX_GetTickCount(pdev, &current_time_ms);
        polling_time_ms = current_time_ms - start_time_ms;
    }

#ifdef VL53LX_LOG_ENABLE
    /* Restore function logging */
    VL53LX_set_trace_functions(trace_functions);
#endif

    if (found == 0 && status == VL53LX_ERROR_NONE)
        status = VL53LX_ERROR_TIME_OUT;

    return status;
}
