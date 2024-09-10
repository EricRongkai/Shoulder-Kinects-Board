/**
 * Created by cottley on 4/13/2018.
 */

#ifndef ADS_HAL_
#define ADS_HAL_

#include <stdint.h>
#include "ads_err.h"
#include "stm32f10x.h"

#define ADS_TRANSFER_SIZE		(3)

#define BOOL uint8_t
#define TRUE 1
#define FALSE 0

//static inline void ads_hal_gpio_pin_write(GPIO_TypeDef* gpio, uint8_t pin, uint8_t val);

/**
 * @brief Interrupt handler function.	
 *
 */
void ads_hal_interrupt(u8 device);
	
/**
 * @brief Millisecond delay routine.
 */
void ads_hal_delay(uint16_t delay_ms);

/**
 * @brief Initializes the pin ADS_INTERRUPT_PIN as a falling edge pin change interrupt.
 *			Assign the interrupt service routine as ads_hal_interrupt. Enable pullup
 *			Enable interrupt	
 */
void ads_hal_pin_int_enable(BOOL enable);

/**
 * @brief Write buffer of data to the Angular Displacement Sensor
 *
 * @param buffer[in]	Write buffer
 * @param len			Length of buffer.
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_hal_write_buffer(uint8_t * buffer, uint8_t len);

/**
 * @brief Read buffer of data from the Angular Displacement Sensor
 *
 * @param buffer[out]	Read buffer
 * @param len			Length of buffer.
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_hal_read_buffer(uint8_t * buffer, uint8_t len);

/**
 * @brief Reset the Angular Displacement Sensor
 */
void ads_hal_reset(void);

/**
 * @brief Initializes the hardware abstraction layer 
 *
 * @param callback to ads.cpp, 
 * @param reset_pin pin number for reset line of ads
 * @param datardy_pin pin number for data ready interrupt 
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_hal_init(void (*callback)(uint8_t*,uint8_t), uint32_t reset_pin, uint32_t datardy_pin);

/**
 * @brief Gets the current i2c address that the hal layer is addressing. 	
 *				Used by device firmware update (dfu)
 * @return	uint8_t _address
 */
uint8_t ads_hal_get_address(void);

/**
 * @brief Sets the current i2c address that the hal layer is addressing. 	
 *				Used by device firmware update (dfu) 
 */
void ads_hal_set_address(uint8_t address);


int ads_hal_select_device(uint8_t device);

BOOL ADS_change_adress(u8 old_address,u8 new_address);

void I2C2_CLEAR_BUSY(void);


#endif /* ADS_HAL_ */
