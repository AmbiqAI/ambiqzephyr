/*
 * Copyright 2023, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_rm69330

#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/policy.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/pinctrl.h>

#include <am_mcu_apollo.h>

LOG_MODULE_REGISTER(rm69330, CONFIG_DISPLAY_LOG_LEVEL);

//todo: this need kconfig
#define TCM_SIZE (1024)
#define CMD_SEND_TIMEOUT (100000)
#define FB_SEND_TIMEOUT_MS  (500)

#define RM69330_MSPI_CMD_WRITE                   0x02
#define RM69330_MSPI_PIXEL_WRITE_ADDR1           0x32    // address on single line
#define RM69330_MSPI_PIXEL_WRITE_ADDR4           0x12    // address on quad interface
#define RM69330_MSPI_CMD_READ                    0x03
#define RM69330_MSPI_READ_ID                     0x04

#define RM69330_MSPI_SOFTWARE_RESET              0x01
#define RM69330_MSPI_PAGE_PROGRAM                0x02
#define RM69330_MSPI_READ                        0x03
#define RM69330_MSPI_WRITE_DISABLE               0x04
#define RM69330_MSPI_READ_STATUS                 0x05
#define RM69330_MSPI_WRITE_ENABLE                0x06
#define RM69330_MSPI_FAST_READ                   0x0B
#define RM69330_MSPI_READ_PIXEL_FORMAT           0x0C
#define RM69330_MSPI_SLEEP_IN                    0x10
#define RM69330_MSPI_SLEEP_OUT                   0x11
#define RM69330_MSPI_NORMAL_MODE_ON              0x13
#define RM69330_MSPI_INVERSION_OFF               0x20
#define RM69330_MSPI_DISPLAY_OFF                 0x28
#define RM69330_MSPI_DISPLAY_ON                  0x29
#define RM69330_MSPI_SET_COLUMN                  0x2A
#define RM69330_MSPI_SET_ROW                     0x2B
#define RM69330_MSPI_MEM_WRITE                   0x2C
#define RM69330_MSPI_TE_LINE_OFF                 0x34
#define RM69330_MSPI_TE_LINE_ON                  0x35
#define RM69330_MSPI_SCAN_DIRECTION              0x36
#define RM69330_MSPI_IDLE_MODE_OFF               0x38
#define RM69330_MSPI_PIXEL_FORMAT                0x3A
#define RM69330_MSPI_DUAL_READ                   0x3B
#define RM69330_MSPI_MEM_WRITE_CONTINUE          0x3C
#define RM69330_MSPI_SET_TEAR_SCANLINE           0x44
#define RM69330_MSPI_WRITE_DISPLAY_BRIGHTNESS    0x51
#define RM69330_MSPI_WRITE_ENHVOL_CFG            0x61
#define RM69330_MSPI_RESET_ENABLE                0x66
#define RM69330_MSPI_QUAD_READ                   0x6B
#define RM69330_MSPI_WRITE_VOL_CFG               0x81
#define RM69330_MSPI_RESET_MEMORY                0x99
#define RM69330_MSPI_ENTER_4B                    0xB7
#define RM69330_MSPI_SET_DSPI_MODE               0xC4
#define RM69330_MSPI_BULK_ERASE                  0xC7
#define RM69330_MSPI_SECTOR_ERASE                0xD8
#define RM69330_MSPI_EXIT_4B                     0xE9
#define RM69330_MSPI_QUAD_IO_READ                0xEB
#define RM69330_MSPI_READ_QUAD_4B                0xEC
#define RM69330_MSPI_CMD_MODE                    0xFE

#define RM69330_MSPI_SPI_WRAM                    0x80
#define RM69330_MSPI_DSPI_WRAM                   0x81

#define RM69330_MSPI_COLOR_MODE_8BIT             0x72
#define RM69330_MSPI_COLOR_MODE_3BIT             0x73
#define RM69330_MSPI_COLOR_MODE_16BIT            0x75
#define RM69330_MSPI_COLOR_MODE_18BIT            0x76
#define RM69330_MSPI_COLOR_MODE_24BIT            0x77

#define RM69330_MSPI_SCAN_MODE_0                 0x40
#define RM69330_MSPI_SCAN_MODE_90                0x70
#define RM69330_MSPI_SCAN_MODE_180               0x10
#define RM69330_MSPI_SCAN_MODE_270               0x00

/* These configuration should all come from DT, not from this const definition.
*/
const static am_hal_mspi_dev_config_t  QuadCE0DisplayMSPICfg =
{
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
    .ui8TurnAround        = 0,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4,
    .bSendInstr           = true,
    .bSendAddr            = true,
    .bTurnaround          = false,
    .ui32TCBSize          = 0,
    .pTCB                 = NULL,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
    .ui8ReadInstr         = RM69330_MSPI_FAST_READ,
    .ui8WriteInstr        = RM69330_MSPI_PIXEL_WRITE_ADDR4,
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE,
};

struct rm69330_config {
    const struct pinctrl_dev_config *pcfg;
	const struct gpio_dt_spec reset_gpio;
	const struct gpio_dt_spec bl_gpio;
	const struct gpio_dt_spec te_gpio;
	uint16_t panel_width;
	uint16_t panel_height;
};


struct rm69330_data {
    am_hal_mspi_dev_config_t sMspiConfig;
    void *mspiHandle;
	uint8_t pixel_format;
	uint8_t bytes_per_pixel;
	struct gpio_callback te_gpio_cb;
	struct k_sem te_sem;
    struct k_sem fb_sem;
};

static void rm69330_te_isr_handler(const struct device *gpio_dev,
				   struct gpio_callback *cb, uint32_t pins)
{
	struct rm69330_data *data = CONTAINER_OF(cb, struct rm69330_data, te_gpio_cb);

	k_sem_give(&data->te_sem);
}

static int
rm69330_command_read(const struct device *dev,
                      uint32_t ui32Instr,
                      uint8_t *pData,
                      uint32_t ui32NumBytes)
{
	struct rm69330_data *data = dev->data;
    am_hal_mspi_dev_config_t* pMspiConfig = &data->sMspiConfig;
    void* pMspiHandle = data->mspiHandle;
	int ret = 0;

    am_hal_mspi_pio_transfer_t Transaction;

    __ASSERT_NO_MSG(ui32NumBytes <= 4);

    uint32_t ui32CurrentMspiDeviceConfig = pMspiConfig->eDeviceConfig;
    bool bNeedSwitch = (ui32CurrentMspiDeviceConfig > AM_HAL_MSPI_FLASH_SERIAL_CE1) ? true : false;

    //
    // Switch to Cmd configuration.
    //
    if ( bNeedSwitch )
    {
        uint32_t ui32CommandConfig = (ui32CurrentMspiDeviceConfig % 2) ? AM_HAL_MSPI_FLASH_SERIAL_CE1 : AM_HAL_MSPI_FLASH_SERIAL_CE0;
        am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_DEVICE_CONFIG, &ui32CommandConfig);
    }

    uint32_t readFreq = AM_HAL_MSPI_CLK_8MHZ;
    am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_CLOCK_CONFIG, &readFreq);

    //
    // Create the individual write transaction.
    //
    Transaction.ui32NumBytes       = ui32NumBytes;
    Transaction.bScrambling        = false;
    Transaction.eDirection         = AM_HAL_MSPI_RX;
    Transaction.bSendAddr          = true;
    Transaction.ui32DeviceAddr     = ui32Instr << 8;
    Transaction.bSendInstr         = true;
    Transaction.ui16DeviceInstr    = RM69330_MSPI_CMD_READ;
    Transaction.bTurnaround        = false;
    Transaction.bQuadCmd           = false;
    Transaction.bDCX               = false;
    Transaction.bEnWRLatency       = false;
    Transaction.bContinue          = false;    // MSPI CONT is deprecated for Apollo3
    Transaction.pui32Buffer        = (uint32_t*)pData;

    //
    // Execute the transaction over MSPI.
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_blocking_transfer(pMspiHandle, &Transaction,
                                         CMD_SEND_TIMEOUT))
    {
        LOG_ERR("Failed to send command.\n");
        ret = -1;
    }

    am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_CLOCK_CONFIG, &pMspiConfig->eClockFreq);

    //
    // Switch to Device configuration.
    //
    if ( bNeedSwitch )
    {
        //
        // Re-Configure the MSPI for the requested operation mode.
        //
        am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_DEVICE_CONFIG, &ui32CurrentMspiDeviceConfig);
    }

    return ret;
}

static int
rm69330_command_write(const struct device *dev,
                      uint32_t ui32Instr,
                      uint8_t *pData,
                      uint32_t ui32NumBytes)
{
	struct rm69330_data *data = dev->data;
    am_hal_mspi_dev_config_t* pMspiConfig = &data->sMspiConfig;
    void* pMspiHandle = data->mspiHandle;
	int ret = 0;

    am_hal_mspi_pio_transfer_t Transaction;

    __ASSERT_NO_MSG(ui32NumBytes <= 4);

    uint32_t ui32CurrentMspiDeviceConfig = pMspiConfig->eDeviceConfig;
    bool bNeedSwitch = (ui32CurrentMspiDeviceConfig > AM_HAL_MSPI_FLASH_SERIAL_CE1) ? true : false;

    //
    // Switch to Cmd configuration.
    //
    if ( bNeedSwitch )
    {
        uint32_t ui32CommandConfig = (ui32CurrentMspiDeviceConfig % 2) ? AM_HAL_MSPI_FLASH_SERIAL_CE1 : AM_HAL_MSPI_FLASH_SERIAL_CE0;
        am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_DEVICE_CONFIG, &ui32CommandConfig);
    }

    //
    // Create the individual write transaction.
    //
    Transaction.ui32NumBytes       = ui32NumBytes;
    Transaction.bScrambling        = false;
    Transaction.eDirection         = AM_HAL_MSPI_TX;
    Transaction.bSendAddr          = true;
    Transaction.ui32DeviceAddr     = ui32Instr << 8;
    Transaction.bSendInstr         = true;
    Transaction.ui16DeviceInstr    = RM69330_MSPI_CMD_WRITE;
    Transaction.bTurnaround        = false;
    Transaction.bQuadCmd           = false;
    Transaction.bDCX               = false;
    Transaction.bEnWRLatency       = false;
    Transaction.bContinue          = false;    // MSPI CONT is deprecated for Apollo3
    Transaction.pui32Buffer        = (uint32_t*)pData;

    //
    // Execute the transaction over MSPI.
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_blocking_transfer(pMspiHandle, &Transaction,
                                         CMD_SEND_TIMEOUT))
    {
        LOG_ERR("Failed to send command.\n");
        ret = -1;
    }

    //
    // Switch to Device configuration.
    //
    if ( bNeedSwitch )
    {
        //
        // Re-Configure the MSPI for the requested operation mode.
        //
        am_hal_mspi_control(pMspiHandle, AM_HAL_MSPI_REQ_DEVICE_CONFIG, &ui32CurrentMspiDeviceConfig);
    }

    return ret;
}


#define AM_BSP_GPIO_DSPL_RESET 11
#define AM_BSP_GPIO_DSPL0_OLED_EN 49
#define  AM_BSP_GPIO_DSPL0_OLED_PWER_EN 39
#define AM_BSP_GPIO_DSPL0_VIO_EN 40
#define AM_BSP_GPIO_DSPL0_DSPL_3V3_EN 31


static int rm69330_init(const struct device *dev)
{
	const struct rm69330_config *config = dev->config;
	struct rm69330_data *data = dev->data;
	int ret;

    /* Init MSPI interface, the mspi instence id and other properties should be got from DT*/

    /* Get mspi interface configration from device tree */
    data->sMspiConfig = QuadCE0DisplayMSPICfg; //todo: got this from DT, NOT const variable
    data->sMspiConfig.ui32TCBSize = 1024; //TODO: This should ge got from kconfig
    data->sMspiConfig.pTCB = k_malloc(data->sMspiConfig.ui32TCBSize*4); //TODO: This memory block may need to be allocated from SSRAM, how to do this?
    uint32_t ui32Module = 0; //todo: get this from DT
    uint8_t ui8CMDBuf[4];

    /* Init MSPI instance */
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_initialize(ui32Module, &data->mspiHandle))
    {
        LOG_ERR("Failed to initialize MSPI.\n");
        return -1;//TODO: SET A LINUX ERROR NUMBER
    }


    //TODO: USE ZEPHYR POWER management 
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(data->mspiHandle, AM_HAL_SYSCTRL_WAKE, false))
    {
        LOG_ERR("Failed to power on MSPI.\n");
        return -1;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(data->mspiHandle, &data->sMspiConfig))
    {
        LOG_ERR("Error - Failed to configure MSPI.\n");
        return -1;
    }
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(data->mspiHandle))
    {
        LOG_ERR("Error - Failed to enable MSPI.\n");
        return -1;
    }

    ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		return ret;
	}

    am_hal_gpio_pinconfig(AM_BSP_GPIO_DSPL_RESET, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_output_set(AM_BSP_GPIO_DSPL_RESET);

    // am_hal_gpio_pinconfig(AM_BSP_GPIO_DSPL_TE, g_AM_BSP_GPIO_DSPL_TE);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_DSPL0_OLED_EN, g_AM_HAL_GPIO_INPUT);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_DSPL0_OLED_PWER_EN, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_output_clear(AM_BSP_GPIO_DSPL0_OLED_PWER_EN);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_DSPL0_VIO_EN, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_output_set(AM_BSP_GPIO_DSPL0_VIO_EN);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_DSPL0_DSPL_3V3_EN, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_output_set(AM_BSP_GPIO_DSPL0_DSPL_3V3_EN);

    am_hal_gpio_output_clear(AM_BSP_GPIO_DSPL_RESET);
    k_sleep(K_MSEC(20));
    am_hal_gpio_output_set(AM_BSP_GPIO_DSPL_RESET);
    k_sleep(K_MSEC(150));  //Delay 150ms

    // am_hal_gpio_pinconfig(40, g_AM_HAL_GPIO_OUTPUT);
    // am_hal_gpio_output_set(40);

    // am_hal_gpio_pinconfig(31, g_AM_HAL_GPIO_OUTPUT);
    // am_hal_gpio_output_set(31);

    // am_hal_gpio_pinconfig(39, g_AM_HAL_GPIO_OUTPUT);
    // am_hal_gpio_output_clear(39);

    // // gpio_pin_configure_dt(&config->bl_gpio, GPIO_OUTPUT_INACTIVE);
    // // gpio_pin_set_dt(&config->bl_gpio, 0);

    // /* Display reset*/
	// if (config->reset_gpio.port != NULL) {
	// 	ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE);
	// 	if (ret < 0) {
	// 		LOG_ERR("Could not configure reset GPIO (%d)", ret);
	// 		return ret;
	// 	}

	// 	/*
	// 	 * Power to the display has been enabled via the regulator fixed api during
	// 	 * regulator init. Per datasheet, we must wait at least 10ms before
	// 	 * starting reset sequence after power on.
	// 	 */
	// 	k_sleep(K_MSEC(10));
	// 	/* Start reset sequence */
	// 	ret = gpio_pin_set_dt(&config->reset_gpio, 0);
	// 	if (ret < 0) {
	// 		LOG_ERR("Could not pull reset low (%d)", ret);
	// 		return ret;
	// 	}
	// 	/* Per datasheet, reset low pulse width should be at least 10usec */
	// 	k_sleep(K_USEC(30));
	// 	gpio_pin_set_dt(&config->reset_gpio, 1);
	// 	if (ret < 0) {
	// 		LOG_ERR("Could not pull reset high (%d)", ret);
	// 		return ret;
	// 	}
	// 	/*
	// 	 * It is necessary to wait at least 120msec after releasing reset,
	// 	 * before sending additional commands. This delay can be 5msec
	// 	 * if we are certain the display module is in SLEEP IN state,
	// 	 * but this is not guaranteed (for example, with a warm reset)
	// 	 */
	// 	k_sleep(K_MSEC(150));
	// }

	/* Now, write initialization settings for display, running at 400x392 */
    ui8CMDBuf[0] = 0x00;      //switch to User Commands Sets(UCS = CMD1)
    if ( rm69330_command_write(dev, RM69330_MSPI_CMD_MODE, ui8CMDBuf, 1) != 0)  // set page CMD1
    {
        return -1;
    }

    ui8CMDBuf[0] = RM69330_MSPI_SPI_WRAM;
    if ( rm69330_command_write(dev, RM69330_MSPI_SET_DSPI_MODE, ui8CMDBuf, 1)  != 0)  // spi ram enable default is mipi,Aaron modified
    {
        return -1;
    }

    if ( rm69330_command_write(dev, RM69330_MSPI_IDLE_MODE_OFF, NULL, 0)  != 0)  // idle mode off
    {
        return -1;
    }

    k_sleep(K_MSEC(120));

    if ( rm69330_command_write(dev, RM69330_MSPI_DISPLAY_OFF, NULL, 0)  != 0)  // display off
    {
        return -1;
    }

    ui8CMDBuf[0] = RM69330_MSPI_SCAN_MODE_270;
    if ( rm69330_command_write(dev, RM69330_MSPI_SCAN_DIRECTION, ui8CMDBuf, 1)  != 0)  // scan direction to 0
    {
        return -1;
    }

	if (data->pixel_format == PIXEL_FORMAT_RGB_888) {
		ui8CMDBuf[0] = RM69330_MSPI_COLOR_MODE_24BIT;
		data->bytes_per_pixel = 3;
	} else if (data->pixel_format == PIXEL_FORMAT_RGB_565) {
		ui8CMDBuf[0] = RM69330_MSPI_COLOR_MODE_16BIT;
		data->bytes_per_pixel = 2;
	}

    if ( rm69330_command_write(dev, RM69330_MSPI_PIXEL_FORMAT, ui8CMDBuf, 1)  != 0)
    {
        return -1;
    }

    ui8CMDBuf[0] = 0x00;      // TE on , only V-blanking
    if ( rm69330_command_write(dev, RM69330_MSPI_TE_LINE_ON, ui8CMDBuf, 1)  != 0)
    {
        return -1;
    }

    k_sleep(K_MSEC(10));

    ui8CMDBuf[0] = 0xff;      // write display brightness
    if ( rm69330_command_write(dev, RM69330_MSPI_WRITE_DISPLAY_BRIGHTNESS, ui8CMDBuf, 1)  != 0)
    {
        return -1;
    }

    k_sleep(K_MSEC(10));

    if ( rm69330_command_write(dev, RM69330_MSPI_SLEEP_OUT, NULL, 0)  != 0)  // sleep out
    {
        return -1;
    }

    k_sleep(K_MSEC(10));

    uint16_t ui16ColumnStart = 0;
    uint16_t ui16ColumnSize = config->panel_width;
    uint16_t ui16RowStart = 0;
    uint16_t ui16RowSize = config->panel_height;

    ui8CMDBuf[0] = (ui16ColumnStart / 256);
    ui8CMDBuf[1] = (ui16ColumnStart % 256);
    ui8CMDBuf[2] = (ui16ColumnStart + ui16ColumnSize - 1) / 256;
    ui8CMDBuf[3] = (ui16ColumnStart + ui16ColumnSize - 1) % 256;
    if ( rm69330_command_write(dev, RM69330_MSPI_SET_COLUMN, ui8CMDBuf, 4) != 0)
    {
        return -1;
    }

    k_sleep(K_MSEC(10));

    ui8CMDBuf[0] = (ui16RowStart / 256);
    ui8CMDBuf[1] = (ui16RowStart % 256);
    ui8CMDBuf[2] = (ui16RowStart + ui16RowSize -1) / 256;
    ui8CMDBuf[3] = (ui16RowStart + ui16RowSize -1) % 256;
    if ( rm69330_command_write(dev, RM69330_MSPI_SET_ROW, ui8CMDBuf, 4) != 0)
    {
        return -1;
    }

    k_sleep(K_MSEC(200));

    if ( rm69330_command_write(dev, RM69330_MSPI_NORMAL_MODE_ON, NULL, 0)  != 0)  // normal display on
    {
        return -1;
    }

    k_sleep(K_MSEC(10));

    /*Check display panel ID to confirm the initialization result */
    ui8CMDBuf[0] = 0;
    ui8CMDBuf[1] = 0;
    ui8CMDBuf[2] = 0;
    if (rm69330_command_read(dev, RM69330_MSPI_READ_ID, ui8CMDBuf, 3) != 0)
    {
        return -1;
    }

    if( (ui8CMDBuf[0]!= 0x00) || (ui8CMDBuf[1]!= 0x80) || (ui8CMDBuf[2]!= 0x00) )
    {
        LOG_ERR("RM69330 init failed!");
    }

    ret = rm69330_command_write(dev, RM69330_MSPI_DISPLAY_OFF, NULL, 0);
    if (ret < 0)
    {
        return ret;
    }   

    ret = rm69330_command_write(dev, RM69330_MSPI_DISPLAY_ON, NULL, 0);
    if (ret < 0)
    {
        return ret;
    }

    k_sleep(K_MSEC(10));



    //
    // Enable MSPI interrupts.
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_interrupt_clear(data->mspiHandle, AM_HAL_MSPI_INT_CQUPD | AM_HAL_MSPI_INT_ERR ))
    {
        return -1;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_interrupt_enable(data->mspiHandle, AM_HAL_MSPI_INT_CQUPD | AM_HAL_MSPI_INT_ERR ))
    {
        return -1;
    }

	if (config->te_gpio.port != NULL) {
		/* Setup TE pin */
		ret = gpio_pin_configure_dt(&config->te_gpio, GPIO_INPUT);
		if (ret < 0) {
			LOG_ERR("Could not configure TE GPIO (%d)", ret);
			return ret;
		}

		ret = gpio_pin_interrupt_configure_dt(&config->te_gpio,
						      GPIO_INT_EDGE_TO_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Could not configure TE interrupt (%d)", ret);
			return ret;
		}

		/* Init and install GPIO callback */
		gpio_init_callback(&data->te_gpio_cb, rm69330_te_isr_handler,
				BIT(config->te_gpio.pin));
		gpio_add_callback(config->te_gpio.port, &data->te_gpio_cb);

		/* Setup te pin semaphore */
		k_sem_init(&data->te_sem, 0, 1);
	}

	/* Setup te pin semaphore */
	k_sem_init(&data->fb_sem, 0, 1);

    return 0;

}

static void
rm69330_transfer_complete(void *pCallbackCtxt, uint32_t transactionStatus)
{
    struct k_sem* pCompleteSem = (struct k_sem*)pCallbackCtxt;
    k_sem_give(pCompleteSem);
}

//*****************************************************************************
//
// Programs the given range of display addresses.
//
//*****************************************************************************
uint32_t
rm69330_write_fb(const struct device *dev,
                 bool first_write,
                 const uint8_t *src,
                 uint32_t len,
                 am_hal_mspi_callback_t pfnCallback,
                 void *pCallbackCtxt)
{
	const struct rm69330_config *config = dev->config;
	struct rm69330_data *data = dev->data;
    am_hal_mspi_dma_transfer_t Transaction;
    uint32_t      ui32BytesLeft = len;

    //
    // Create the transaction.
    //
    Transaction.ui8Priority               = 1;
    Transaction.eDirection                = AM_HAL_MSPI_TX;
    Transaction.ui32TransferCount         = (ui32BytesLeft <= AM_HAL_MSPI_MAX_TRANS_SIZE) ? ui32BytesLeft : AM_HAL_MSPI_MAX_TRANS_SIZE;
    Transaction.ui32DeviceAddress         = ((first_write == true) ? RM69330_MSPI_MEM_WRITE : RM69330_MSPI_MEM_WRITE_CONTINUE) << 8;
    Transaction.ui32SRAMAddress           = (uint32_t)src;
    Transaction.ui32PauseCondition        = 0;
    Transaction.ui32StatusSetClr          = 0;

    //
    // Execute the transction over MSPI.
    //
    if (am_hal_mspi_nonblocking_transfer(data->mspiHandle,
                                       &Transaction,
                                       AM_HAL_MSPI_TRANS_DMA,
                                       (ui32BytesLeft <= AM_HAL_MSPI_MAX_TRANS_SIZE) ? pfnCallback : NULL,
                                       (ui32BytesLeft <= AM_HAL_MSPI_MAX_TRANS_SIZE) ? pCallbackCtxt : NULL))
    {
        return -1;
    }

    Transaction.ui32DeviceAddress         = RM69330_MSPI_MEM_WRITE_CONTINUE << 8;
    ui32BytesLeft -= Transaction.ui32TransferCount;
    while( ui32BytesLeft )
    {
        Transaction.ui32SRAMAddress      += Transaction.ui32TransferCount;
        Transaction.ui32TransferCount     = (ui32BytesLeft <= AM_HAL_MSPI_MAX_TRANS_SIZE) ? ui32BytesLeft : AM_HAL_MSPI_MAX_TRANS_SIZE;
        if (am_hal_mspi_nonblocking_transfer(data->mspiHandle,
                                             &Transaction,
                                             AM_HAL_MSPI_TRANS_DMA,
                                             (ui32BytesLeft <= AM_HAL_MSPI_MAX_TRANS_SIZE) ? pfnCallback : NULL,
                                             (ui32BytesLeft <= AM_HAL_MSPI_MAX_TRANS_SIZE) ? pCallbackCtxt : NULL))
        {
            return -1;
        }

        ui32BytesLeft -= Transaction.ui32TransferCount;
    }

    return 0;
}




static int rm69330_write(const struct device *dev, const uint16_t x,
			 const uint16_t y,
			 const struct display_buffer_descriptor *desc,
			 const void *buf)
{
	const struct rm69330_config *config = dev->config;
	struct rm69330_data *data = dev->data;
	int ret;
	uint16_t start, end, h_idx;
	const uint8_t *src;
	bool first_cmd;
	uint8_t param[4];

	LOG_DBG("W=%d, H=%d @%d,%d", desc->width, desc->height, x, y);

	/*
	 * RM69330 runs in MIPI DBI mode. This means we can use command mode
	 * to write to the video memory buffer on the RM69330 control IC,
	 * and the IC will update the display automatically.
	 */

	/* Set column address of target area */
	/* First two bytes are starting X coordinate */
	start = x;
	end = x + desc->width - 1;
	sys_put_be16(start, &param[0]);
	/* Second two bytes are ending X coordinate */
	sys_put_be16(end, &param[2]);
	ret = rm69330_command_write(dev, RM69330_MSPI_SET_COLUMN, param, 4);
	if (ret < 0) {
		return ret;
	}

	/* Set page address of target area */
	/* First two bytes are starting Y coordinate */
	start = y;
	end = y + desc->height - 1;
	sys_put_be16(start, &param[0]);
	/* Second two bytes are ending X coordinate */
	sys_put_be16(end, &param[2]);
	ret = rm69330_command_write(dev, RM69330_MSPI_SET_ROW, param, 4);
	if (ret < 0) {
		return ret;
	}

	/*
	 * Now, write the framebuffer. If the tearing effect GPIO is present,
	 * wait until the display controller issues an interrupt (which will
	 * give to the TE semaphore) before sending the frame
	 */
	// if (config->te_gpio.port != NULL) {
	// 	/* Block sleep state until next TE interrupt so we can send
	// 	 * frame during that interval
	// 	 */
	// 	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE,
	// 				 PM_ALL_SUBSTATES);
	// 	k_sem_take(&data->te_sem, K_FOREVER);
	// 	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE,
	// 				 PM_ALL_SUBSTATES);
	// }
	src = buf;
	first_cmd = true;

	if (desc->pitch == desc->width) {
		/* Buffer is contiguous, we can perform entire transfer */
		ret = rm69330_write_fb(dev, first_cmd, src,
			desc->height * desc->width * data->bytes_per_pixel, 
            rm69330_transfer_complete, 
            &data->fb_sem);
	} else {
		/* Buffer is not contiguous, we must write each line separately */
		for (h_idx = 0; h_idx < desc->height; h_idx++) {

			ret = rm69330_write_fb(dev, first_cmd, src,
				desc->width * data->bytes_per_pixel,
                ((h_idx == (desc->height - 1)) ?  rm69330_transfer_complete : NULL),
                ((h_idx == (desc->height - 1)) ?  &data->fb_sem : NULL));

            if(ret < 0)
            {
                break;
            }

			first_cmd = false;
			/* The pitch is not equal to width, account for it here */
			src += data->bytes_per_pixel * (desc->pitch - desc->width);
		}
	}

    if(ret == 0)
    {
        k_sem_take(&data->fb_sem, K_MSEC(FB_SEND_TIMEOUT_MS));
    }

	return ret;
}

static void rm69330_get_capabilities(const struct device *dev,
				     struct display_capabilities *capabilities)
{
	const struct rm69330_config *config = dev->config;
	const struct rm69330_data *data = dev->data;

	memset(capabilities, 0, sizeof(struct display_capabilities));
	capabilities->x_resolution = config->panel_width;
	capabilities->y_resolution = config->panel_height;
	capabilities->supported_pixel_formats = PIXEL_FORMAT_RGB_565 |
						PIXEL_FORMAT_RGB_888;
	capabilities->current_pixel_format = data->pixel_format;
	capabilities->current_orientation = DISPLAY_ORIENTATION_ROTATED_90;
}

static int rm69330_blanking_on(const struct device *dev)
{
	const struct rm69330_data *data = dev->data;

    if ( rm69330_command_write(dev, RM69330_MSPI_DISPLAY_OFF, NULL, 0) != 0)
    {
        return -1;
    }

    if ( rm69330_command_write(dev, RM69330_MSPI_SLEEP_IN, NULL, 0) !=0 )
    {
        return -1;
    }

    return 0;
}

static int rm69330_blanking_off(const struct device *dev)
{
	const struct rm69330_data *data = dev->data;

    if ( rm69330_command_write(dev, RM69330_MSPI_SLEEP_OUT, NULL, 0) != 0)
    {
        return -1;
    }

    if (rm69330_command_write(dev, RM69330_MSPI_DISPLAY_ON, NULL, 0) != 0)
    {
        return -1;
    }

    return 0;
}

static int rm69330_set_pixel_format(const struct device *dev,
				    const enum display_pixel_format pixel_format)
{
	const struct rm69330_config *config = dev->config;
	struct rm69330_data *data = dev->data;
	uint8_t param;

	switch (pixel_format) {
	case PIXEL_FORMAT_RGB_565:
		data->pixel_format = PIXEL_FORMAT_RGB_565;
		return 0;
	case PIXEL_FORMAT_RGB_888:
		data->pixel_format = PIXEL_FORMAT_RGB_888;
		return 0;
	default:
		/* Other display formats not implemented */
		return -ENOTSUP;
	}
	if (data->pixel_format == PIXEL_FORMAT_RGB_888) {
		param = RM69330_MSPI_COLOR_MODE_24BIT;
		data->bytes_per_pixel = 3;
	} else if (data->pixel_format == PIXEL_FORMAT_RGB_565) {
		param = RM69330_MSPI_COLOR_MODE_16BIT;
		data->bytes_per_pixel = 2;
	}

    if ( rm69330_command_write(dev, RM69330_MSPI_PIXEL_FORMAT, &param, 1)  != 0)
    {
        return -1;
    }

    return 0;
}

static int rm69330_set_orientation(const struct device *dev,
				   const enum display_orientation orientation)
{
	if (orientation == DISPLAY_ORIENTATION_NORMAL) {
		return 0;
	}
	LOG_ERR("Changing display orientation not implemented");
	return -ENOTSUP;
}

#ifdef CONFIG_PM_DEVICE

static int rm69330_pm_action(const struct device *dev,
			     enum pm_device_action action)
{
	// const struct rm69330_config *config = dev->config;
	// struct rm69330_data *data = dev->data;
	// struct mipi_dsi_device mdev = {0};

	// mdev.data_lanes = config->num_of_lanes;
	// mdev.pixfmt = data->pixel_format;

	// switch (action) {
	// case PM_DEVICE_ACTION_SUSPEND:
	// 	/* Detach from the MIPI DSI controller */
	// 	return mipi_dsi_detach(config->mipi_dsi, config->channel, &mdev);
	// case PM_DEVICE_ACTION_RESUME:
	// 	return mipi_dsi_attach(config->mipi_dsi, config->channel, &mdev);
	// default:
	// 	return -ENOTSUP;
	// }
    return 0;
}

#endif /* CONFIG_PM_DEVICE */

static const struct display_driver_api rm69330_api = {
	.blanking_on = rm69330_blanking_on,
	.blanking_off = rm69330_blanking_off,
	.get_capabilities = rm69330_get_capabilities,
	.write = rm69330_write,
	.set_pixel_format = rm69330_set_pixel_format,
	.set_orientation = rm69330_set_orientation,
};

#define RM69330_PANEL(id)							\
    PINCTRL_DT_INST_DEFINE(id); \
	static const struct rm69330_config rm69330_config_##id = {		\
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(id),                      \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(id, reset_gpios, {0}),	\
		.te_gpio = GPIO_DT_SPEC_INST_GET_OR(id, te_gpios, {0}),		\
        .bl_gpio = GPIO_DT_SPEC_INST_GET_OR(id, bl_gpios, {0}),		\
		.panel_width = DT_INST_PROP(id, width),				\
		.panel_height = DT_INST_PROP(id, height),			\
	};									\
	static struct rm69330_data rm69330_data_##id = {			\
		.pixel_format = DT_INST_PROP(id, pixel_format),			\
	};									\
	DEVICE_DT_INST_DEFINE(id,						\
			    &rm69330_init,					\
			    NULL,				\
			    &rm69330_data_##id,					\
			    &rm69330_config_##id,				\
			    POST_KERNEL,					\
			    CONFIG_APPLICATION_INIT_PRIORITY,			\
			    &rm69330_api);

DT_INST_FOREACH_STATUS_OKAY(RM69330_PANEL)
