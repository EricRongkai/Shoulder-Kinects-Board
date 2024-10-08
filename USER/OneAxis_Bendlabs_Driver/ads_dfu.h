/**
 * Created by cottley on 4/17/2018.
 */

#ifndef ADS_DFU_
#define ADS_DFU_

#include <stdint.h>
#include <stdbool.h>
#include "ads_err.h"
#include "ads_hal.h"
#include "ads_util.h"

/**
 * @brief Checks if the firmware image in the driver is newer than 
 *			the firmware on the device.
 *
 * @param ads_get_fw_ver	Get fw version command
 * @return	TRUE if update needed. FALSE if no updated needed
 */
BOOL ads_dfu_check(uint8_t ads_get_fw_ver);


/**
 * @brief Resets the ADS into bootloader mode
 *
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
 int ads_dfu_reset(void);


/**
 * @brief Resets the ADS into bootloader
 *
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_dfu_reset(void);

/**
 * @brief Writes firmware image, contained in ads_fw.h, to the ADS bootloader 
 *			  The ADS needs to be reset into bootloader mode prior to calling
 *				this function
 *
 * @return	ADS_OK if successful ADS_ERR_TIMEOUT if failed
 */
int ads_dfu_update(void);

#endif /* ADS_DFU_ */
