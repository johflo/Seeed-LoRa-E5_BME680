/**
 * Copyright (C) Bosch Sensortec GmbH. All Rights Reserved. Confidential.
 *
 * Disclaimer
 *
 * Common:
 * Bosch Sensortec products are developed for the consumer goods industry. They may only be used
 * within the parameters of the respective valid product data sheet. Bosch Sensortec products are
 * provided with the express understanding that there is no warranty of fitness for a particular purpose.
 * They are not fit for use in life-sustaining, safety or security sensitive systems or any system or device
 * that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
 * Bosch Sensortec products are not fit for use in products which interact with motor vehicle systems.
 * The resale and/or use of products are at the purchaser's own risk and his own responsibility. The
 * examination of fitness for the intended use is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
 * incidental, or consequential damages, arising from any product use not covered by the parameters of
 * the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products, particularly with regard to
 * product safety and inform Bosch Sensortec without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
 * technical specifications of the product series. They are therefore not intended or fit for resale to third
 * parties or for use in end products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series. Bosch Sensortec
 * assumes no liability for the use of engineering samples. By accepting the engineering samples, the
 * Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
 * samples.
 *
 * Special:
 * This software module (hereinafter called "Software") and any information on application-sheets
 * (hereinafter called "Information") is provided free of charge for the sole purpose to support your
 * application work. The Software and Information is subject to the following terms and conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch Sensortec products by
 * personnel who have special experience and training. Do not use this Software if you do not have the
 * proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed or implied warranties,
 * including without limitation, the implied warranties of merchantability and fitness for a particular
 * purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for the functional impairment
 * of this Software in terms of fitness, performance and safety. Bosch Sensortec and their
 * representatives and agents shall not be liable for any direct or indirect damages or injury, except as
 * otherwise stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
 * responsibility for the consequences of use of such Information nor for any infringement of patents or
 * other rights of third parties which may result from its use. No license is granted by implication or
 * otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
 * subject to change without notice.
 *
 * It is not allowed to deliver the source code of the Software to any third party without permission of
 * Bosch Sensortec.
 *
 */

/*!
 * @file bsec_iot_example.c
 *
 * @brief
 * Example for using of BSEC library in a fixed configuration with the BME68x sensor.
 * This works by running an endless loop in the bsec_iot_loop() function.
 */

/*!
 * @addtogroup bsec_examples BSEC Examples
 * @brief BSEC usage examples
 * @{*/

/**********************************************************************************************************************/
/* header files */
/**********************************************************************************************************************/

#include "bsec_integration.h"
#include "common.h"
#include "bsec_serialized_configurations_iaq.h"
#include <string.h>
#include <inttypes.h>
#include "tim.h"
/**********************************************************************************************************************/
/* functions */
/**********************************************************************************************************************/

/*!
 * @brief           Write operation in either I2C or SPI
 *
 * param[in]        dev_addr        I2C or SPI device address
 * param[in]        reg_addr        register address
 * param[in]        reg_data_ptr    pointer to the data to be written
 * param[in]        data_len        number of bytes to be written
 *
 * @return          result of the bus communication function
 */
int8_t bus_write(uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len, void *intf_ptr)
{
    
    bme68x_i2c_write(reg_addr, reg_data_ptr, data_len, intf_ptr);

    return 0;
}

/*!
 * @brief           Read operation in either I2C or SPI
 *
 * param[in]        dev_addr        I2C or SPI device address
 * param[in]        reg_addr        register address
 * param[out]       reg_data_ptr    pointer to the memory to be used to store the read data
 * param[in]        data_len        number of bytes to be read
 *
 * @return          result of the bus communication function
 */
int8_t bus_read(uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len,void *intf_ptr)
{
    
    bme68x_i2c_read(reg_addr, reg_data_ptr, data_len, intf_ptr);

    return 0;
}

/*!
 * @brief           System specific implementation of sleep function
 *
 * @param[in]       t_us     Time in microseconds
 * @param[in]       intf_ptr Pointer to the interface descriptor
 * 
 * @return          none
 */
void sleep_n(uint32_t t_ms, void *intf_ptr)
{
	__HAL_TIM_SET_COUNTER(&htim2, 0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < t_ms);  // wait for the counter to reach the us input in the parameter
}

/*!
 * @brief           Capture the system time in microseconds
 *
 * @return          system_current_time    current system timestamp in microseconds
 */
int64_t get_timestamp_us()
{
    int64_t system_current_time = 0;
    uint32_t timer_tick = HAL_GetTick();
    //PDEBUG("HAL_GetTick: %d\n", timer_tick);
    
    system_current_time =((int64_t)timer_tick)*1000;
    //PDEBUG("System current time: %" PRIu64 "\n", system_current_time);
    
    return system_current_time;
}

/*!
 * @brief           Handling of the ready outputs
 *
 * @param[in]       timestamp               time in milliseconds
 * @param[in]       iaq                     IAQ
 * @param[in]       iaq_accuracy            IAQ accuracy
 * @param[in]       static_iaq              static IAQ
 * @param[in]       temperature             raw temperature
 * @param[in]       humidity                raw humidity
 * @param[in]       temperature             temperature
 * @param[in]       humidity                humidity
 * @param[in]       pressure                pressure
 * @param[in]       gas                     raw gas
 * @param[in]       gas_percentage          gas percentage
 * @param[in]       co2_equivalent          CO2 equivalent
 * @param[in]       breath_voc_equivalent   breath VOC equivalent
 * @param[in]       stabStatus              stabilization status
 * @param[in]       runInStatus             run in status
 * @param[in]       bsec_status             value returned by the bsec_do_steps() call
 *
 * @return          none
 */
uint32_t output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
     float pressure, float raw_temperature, float raw_humidity, float gas, float gas_percentage, bsec_library_return_t bsec_status,
     float static_iaq, float stabStatus, float runInStatus, float co2_equivalent, float breath_voc_equivalent)
{
	/*if (bsec_status == BSEC_OK)
    {
//      PDEBUG("Timestamp(ns): %d, iaq:%.3f, iaq_accuracy:%d, temp:%.3f, hum:%.3f, pres:%.3f, raw temp:%.3f, raw hum:%.3f, gas:%.3f\n",(int32_t)(timestamp/1000000),iaq,iaq_accuracy,temperature,humidity,pressure,raw_temperature,raw_humidity,gas);
    	PDEBUG("# timestamp = %d, iaq = %.3f, iaq_accuracy = %d, temperature=%.3f, humidity=%.3f, pressure=%.3f, bsec_status =%d\r\n",
    				(int32_t)(timestamp/1000000),iaq,iaq_accuracy,temperature,humidity, pressure,bsec_status);
  //  	PDEBUG("# raw_temperature = %.3f, raw_humidity=%.3f, gas=%.3f\r\n",raw_temperature,raw_humidity,gas);
    	PDEBUG("# co2_equivalent=%.3f, breath_voc_equivalent=%.3f, static_iaq=%.3f\r\n",co2_equivalent, breath_voc_equivalent,static_iaq);
  //  	PDEBUG("*********************************************************************************************************************************\r\n");
    }*/
    /* vsnprintf("timestamp=%d, iaq=%f, iaq_accuracy=%d, temperature=%f, humity=%f, pressure=%f, \
		raw_temperature=%f, raw_humity=%f, gas=%f, bsec_status=%d, static_iaq=%f, co2_equivalent=%f, \
		breath_voc_equivalent=%f\r\n", timestamp, iaq, iaq_accuracy, temperature, humidity, pressure, \
		raw_temperature, raw_humidity, gas, bsec_status, static_iaq, co2_equivalent, breath_voc_equivalent);
    */

}

/*!
 * @brief           Load previous library state from non-volatile memory
 *
 * @param[in,out]   state_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to state_buffer
 */
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
    // ...
    // Load a previous library state from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no state was available, 
    // otherwise return length of loaded state string.
    // ...
    return 0;
}

/*!
 * @brief           Save library state to non-volatile memory
 *
 * @param[in]       state_buffer    buffer holding the state to be stored
 * @param[in]       length          length of the state string to be stored
 *
 * @return          none
 */
void state_save(const uint8_t *state_buffer, uint32_t length)
{
    // ...
    // Save the string some form of non-volatile memory, if possible.
    // ...
}
 
/*!
 * @brief           Load library config from non-volatile memory
 *
 * @param[in,out]   config_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to config_buffer
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    // ...
    // Load a library config from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no config was available, 
    // otherwise return length of loaded config string.
    // ...
    memcpy(config_buffer, bsec_config_iaq, sizeof(bsec_config_iaq));
    return sizeof(bsec_config_iaq);
}

int BME680_BSEC_Init(void){
	return_values_init ret;

	    /* Call to the function which initializes the BSEC library
	     * Switch on low-power mode and provide no temperature offset */
	    ret = bsec_iot_init(BSEC_SAMPLE_RATE_ULP, 0.0f, bus_write, bus_read, sleep_n, state_load, config_load);
	    if (ret.bme68x_status)
	    {
	        /* Could not initialize BME68x */
	        return (int)ret.bme68x_status;
	    }
	    else if (ret.bsec_status)
	    {
	        /* Could not intialize BSEC library */
	        return (int)ret.bsec_status;
	    }

	    return 0;
}

int BME680_BSEC_MODULE_RUN(void)
{

    /* Call to endless loop function which reads and processes data based on sensor settings */
    /* State is saved every 10.000 samples, which means every 10.000 * 3 secs = 500 minutes  */
    bsec_iot_loop(sleep_n, get_timestamp_us, output_ready, state_save, 10000);
    
    return 0;
}

/*! @}*/

