/**
 * Copyright (C) 2021 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

#include "bme68x.h"
#include "common.h"
#include "stm32wlxx_hal.h"
#include "bsec_iot.h"



/******************************************************************************/
/*!                Static variable definition                                 */
static uint8_t dev_addr;
uint8_t GTXBuffer[512];
extern I2C_HandleTypeDef hi2c3;
extern UART_HandleTypeDef huart1;


/******************************************************************************/
/*!                User interface functions                                   */

void sleep_n(uint32_t t_ms, void *intf_ptr);


/*!
 * I2C read function
 */
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	uint8_t slave_addr = (uintptr_t)intf_ptr;
	uint8_t dev_addr = slave_addr <<1;
    
    /*
    HAL ERROR
    */

    // send register address
    HAL_I2C_Master_Transmit(&I2C_HANDLE, dev_addr, &reg_addr, 1, BUS_TIMEOUT);
    HAL_I2C_Master_Receive(&I2C_HANDLE, dev_addr, reg_data, len, BUS_TIMEOUT);
    return 0;

}

/*!
 * I2C write function
 */
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t slave_addr = (uintptr_t)intf_ptr;
    uint8_t dev_addr = slave_addr <<1;
    /*
    HAL ERROR
    */
    
    GTXBuffer[0] = reg_addr;
    memcpy(&GTXBuffer[1], reg_data, len);

    // send register address
    HAL_I2C_Master_Transmit(&I2C_HANDLE, dev_addr, GTXBuffer, len+1, BUS_TIMEOUT);

    return 0;
}


/*!
 * Delay function 
 */
void bme68x_delay_us(uint32_t period, void *intf_ptr)
{
	sleep_n(period, intf_ptr);
}

void bme68x_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BME68X_OK:

            /* Do nothing */
            break;
        case BME68X_E_NULL_PTR:
            printf("API name [%s]  Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BME68X_E_COM_FAIL:
            printf("API name [%s]  Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BME68X_E_INVALID_LENGTH:
            printf("API name [%s]  Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BME68X_E_DEV_NOT_FOUND:
            printf("API name [%s]  Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BME68X_E_SELF_TEST:
            printf("API name [%s]  Error [%d] : Self test error\r\n", api_name, rslt);
            break;
        case BME68X_W_NO_NEW_DATA:
            printf("API name [%s]  Warning [%d] : No new data found\r\n", api_name, rslt);
            break;
        default:
            printf("API name [%s]  Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

int8_t bme68x_interface_init(struct bme68x_dev *bme, uint8_t intf)
{
    int8_t rslt = BME68X_OK;
    

        
    /* Bus configuration : I2C */
    if (intf == BME68X_I2C_INTF)
    {
    	PDEBUG("I2C Interface\n");
        dev_addr = BME68X_I2C_ADDR_LOW;
        bme->read = bme68x_i2c_read;
        bme->write = bme68x_i2c_write;
        bme->intf = BME68X_I2C_INTF;
    }
        
    bme->delay_ms = bme68x_delay_us;
    bme->intf_ptr = &dev_addr;
    bme->amb_temp = 25; /* The ambient temperature in deg C is used for defining the heater temperature */
    

    return rslt;
}

void UART_Printf(uint8_t* buff, uint16_t size)
{
    //HAL_UART_Transmit_DMA(&huart2, buff, size);
    HAL_UART_Transmit(&UART_HANDLE, buff, size, BUS_TIMEOUT);
}

char chBuffer[256];
void PDEBUG(char *format, ...)
{
#if defined(DEBUG_EN)
    va_list ap;
#ifdef ADD_TIMESTAMP
    char timestamp[16];
#endif
    va_start(ap, format);
    vsnprintf(chBuffer, sizeof(chBuffer), format, ap);
#ifdef ADD_TIMESTAMP
    sprintf(timestamp, "[%d]", xTaskGetTickCount()); //xTaskGetTickCountFromISR()
    UART_Printf((uint8_t *)timestamp, strlen(timestamp));
#endif
    UART_Printf((uint8_t *)chBuffer,strlen(chBuffer));
    va_end(ap);
#endif
}
