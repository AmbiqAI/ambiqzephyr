/*
 * Copyright (c) 2017 Christer Weinigel.
 * Copyright (c) 2017, I-SENSE group of ICCS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB device controller shim driver for Apollo4p devices
 *
 * This driver uses the Apollo4 Plus low level drivers to talk to the USB
 * device controller on the Apollo4 family of devices using the
 * Apollo4 HAL layer.
 */

#include <soc.h>
#include "am_mcu_apollo.h"
#include <string.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/clock_control/clock_control_ambiq.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>

#define LOG_LEVEL CONFIG_USB_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(usb_dc_apollo4p);

/*
 * Vbus sensing is determined based on the presence of the hardware detection
 * pin(s) in the device tree. E.g: pinctrl-0 = <&usb_otg_fs_vbus_pa9 ...>;
 *
 * The detection pins are dependent on the enabled USB driver and the physical
 * interface(s) offered by the hardware. These are mapped to PA9 and/or PB13
 * (subject to MCU), being the former the most widespread option.
 */
#if DT_HAS_COMPAT_STATUS_OKAY(ambiq_apollo4p_usb)
#define DT_DRV_COMPAT    ambiq_apollo4p_usb
#define USB_IRQ_NAME     usb
#define USB_VBUS_SENSING false
#endif

#define USB_BASE_ADDRESS	DT_INST_REG_ADDR(0)
#define USB_IRQ			DT_INST_IRQ_BY_NAME(0, USB_IRQ_NAME, irq)
#define USB_IRQ_PRI		DT_INST_IRQ_BY_NAME(0, USB_IRQ_NAME, priority)
#define USB_NUM_BIDIR_ENDPOINTS	DT_INST_PROP(0, num_bidir_endpoints)
#define USB_RAM_SIZE		DT_INST_PROP(0, ram_size)

#if DT_INST_NODE_HAS_PROP(0, maximum_speed)
#define USB_MAXIMUM_SPEED	DT_INST_PROP(0, maximum_speed)
#endif

PINCTRL_DT_INST_DEFINE(0);
static const struct pinctrl_dev_config *usb_pcfg =
                        PINCTRL_DT_INST_DEV_CONFIG_GET(0);

#define EP0_MPS 64U
#define EP_MPS 64U

/*
 * USB BTABLE is stored in the PMA. The size of BTABLE is 4 bytes
 * per endpoint.
 *
 */
#define USB_BTABLE_SIZE  (8 * USB_NUM_BIDIR_ENDPOINTS)

/* Size of a USB SETUP packet */
#define SETUP_SIZE 8

/* Helper macros to make it easier to work with endpoint numbers */
#define EP0_IDX 0
#define EP0_IN (EP0_IDX | USB_EP_DIR_IN)
#define EP0_OUT (EP0_IDX | USB_EP_DIR_OUT)

/* Endpoint state */
struct usb_dc_apollo4p_ep_state {
    uint16_t ep_mps;		/** Endpoint max packet size */
    uint16_t ep_pma_buf_len;	/** Previously allocated buffer size */
    uint8_t ep_type;		/** Endpoint type */
    uint8_t ep_stalled;	/** Endpoint stall flag */
    usb_dc_ep_callback cb;	/** Endpoint callback function */
    uint32_t read_count;	/** Number of bytes in read buffer  */
    uint32_t read_offset;	/** Current offset in read buffer */
    struct k_sem write_sem;	/** Write boolean semaphore */
};

static void *pUSBHandle = NULL;

/* Driver state */
struct usb_dc_apollo4p_state {
    usb_dc_status_callback status_cb; /* Status callback */
    struct usb_dc_apollo4p_ep_state out_ep_state[USB_NUM_BIDIR_ENDPOINTS];
    struct usb_dc_apollo4p_ep_state in_ep_state[USB_NUM_BIDIR_ENDPOINTS];
    uint8_t ep_buf[USB_NUM_BIDIR_ENDPOINTS][EP_MPS];
    uint32_t pma_offset;
};

static struct usb_dc_apollo4p_state usb_dc_apollo4p_state;

am_hal_gpio_pincfg_t g_AM_BSP_GPIO_VDDUSB33_SWITCH =
{
    .GP.cfg_b.uFuncSel             = AM_HAL_PIN_13_GPIO,
    .GP.cfg_b.eGPInput             = AM_HAL_GPIO_PIN_INPUT_NONE,
    .GP.cfg_b.eGPRdZero            = AM_HAL_GPIO_PIN_RDZERO_READPIN,
    .GP.cfg_b.eIntDir              = AM_HAL_GPIO_PIN_INTDIR_NONE,
    .GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL,
    .GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P5X,
    .GP.cfg_b.uSlewRate            = 0,
    .GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_NONE,
    .GP.cfg_b.uNCE                 = 0,
    .GP.cfg_b.eCEpol               = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
    .GP.cfg_b.uRsvd_0              = 0,
    .GP.cfg_b.ePowerSw             = AM_HAL_GPIO_PIN_POWERSW_NONE,
    .GP.cfg_b.eForceInputEn        = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.eForceOutputEn       = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.uRsvd_1              = 0,
};

am_hal_gpio_pincfg_t g_AM_BSP_GPIO_VDDUSB0P9_SWITCH =
{
    .GP.cfg_b.uFuncSel             = AM_HAL_PIN_15_GPIO,
    .GP.cfg_b.eGPInput             = AM_HAL_GPIO_PIN_INPUT_NONE,
    .GP.cfg_b.eGPRdZero            = AM_HAL_GPIO_PIN_RDZERO_READPIN,
    .GP.cfg_b.eIntDir              = AM_HAL_GPIO_PIN_INTDIR_NONE,
    .GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL,
    .GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P5X,
    .GP.cfg_b.uSlewRate            = 0,
    .GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_NONE,
    .GP.cfg_b.uNCE                 = 0,
    .GP.cfg_b.eCEpol               = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
    .GP.cfg_b.uRsvd_0              = 0,
    .GP.cfg_b.ePowerSw             = AM_HAL_GPIO_PIN_POWERSW_NONE,
    .GP.cfg_b.eForceInputEn        = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.eForceOutputEn       = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.uRsvd_1              = 0,
};

/* Internal functions */
//*****************************************************************************
//! @brief  This is the event callback function
//! 
//! @param eDevState 
//*****************************************************************************
static void usb_dc_apollo4p_evt_callback(am_hal_usb_dev_event_e eDevState)
{
    switch (eDevState)
    {
        case AM_HAL_USB_DEV_EVT_BUS_RESET:
            //
            // enable usb bus interrupts
            //
            am_hal_usb_intr_usb_enable(pUSBHandle, USB_CFG2_SOFE_Msk | USB_CFG2_ResumeE_Msk | USB_CFG2_SuspendE_Msk |
                                                   USB_CFG2_ResetE_Msk);
            //
            // init the endpoint
            //
            am_hal_usb_ep_init(pUSBHandle, 0, 0, 64);
            am_hal_usb_set_dev_speed(pUSBHandle, AM_HAL_USB_SPEED_FULL);
            /* The DataInCallback will never be called at this point for any pending
             * transactions. Reset the IN semaphores to prevent perpetual locked state.
             * */
            for (int i = 0; i < USB_NUM_BIDIR_ENDPOINTS; i++) {
                k_sem_give(&usb_dc_apollo4p_state.in_ep_state[i].write_sem);
            }
            
            if (usb_dc_apollo4p_state.status_cb) {
                usb_dc_apollo4p_state.status_cb(USB_DC_RESET, NULL);
            }
            break;
        case AM_HAL_USB_DEV_EVT_RESUME:
            if (usb_dc_apollo4p_state.status_cb) {
                usb_dc_apollo4p_state.status_cb(USB_DC_RESUME, NULL);
            }
            // Do something for resuming
            // then set the device state to active
            am_hal_usb_set_dev_state(pUSBHandle, AM_HAL_USB_DEV_STATE_ACTIVE);
            break;
        case AM_HAL_USB_DEV_EVT_SOF:
            usb_dc_apollo4p_state.status_cb(USB_DC_SOF, NULL);
            break;
        case AM_HAL_USB_DEV_EVT_SUSPEND:
            if (usb_dc_apollo4p_state.status_cb) {
                usb_dc_apollo4p_state.status_cb(USB_DC_SUSPEND, NULL);
            }
            // Do something for suspending
            // then set the device state to suspended
            am_hal_usb_set_dev_state(pUSBHandle, AM_HAL_USB_DEV_STATE_SUSPENDED);
            break;
        default:
            // Not reachable case
            // add to suppress the compiling warning
            break;
    }
}

//*****************************************************************************
//! @brief  Setup request is received and pass it to upper layer TinyUSB
//!         stack to handle
//! 
//! @param setup 
//*****************************************************************************
static void usb_dc_apollo4p_ep0_setup_callback(uint8_t *usb_setup)
{
    struct usb_setup_packet *setup = (void *)usb_setup;
    struct usb_dc_apollo4p_ep_state *ep_state;

    LOG_DBG("");

    ep_state = usb_dc_apollo4p_get_ep_state(EP0_OUT); /* can't fail for ep0 */
    __ASSERT(ep_state, "No corresponding ep_state for EP0");

    ep_state->read_count = SETUP_SIZE;
    ep_state->read_offset = 0U;
    memcpy(&usb_dc_apollo4p_state.ep_buf[EP0_IDX],
           setup, ep_state->read_count);

    if (ep_state->cb) {
        ep_state->cb(EP0_OUT, USB_DC_EP_SETUP);

        if (!(setup->wLength == 0U) &&
        usb_reqtype_is_to_device(setup)) {
            usb_dc_ep_start_read(EP0_OUT,
            usb_dc_apollo4p_state.ep_buf[EP0_IDX],
            setup->wLength);
        }
    }
}

//*****************************************************************************
//! @brief 
//! 
//! @param ep_addr 
//! @param xfer_len 
//! @param code 
//! @param param 
//*****************************************************************************
static void usb_dc_apollo4p_ep_xfer_complete_callback(const uint8_t ep_addr,
                                  const uint16_t xfer_len,
                                  am_hal_usb_xfer_code_e code,
                                  void *param)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep_addr);

    if(USB_EP_DIR_IS_IN(ep_addr))
    {
        uint8_t ep = ep_idx | USB_EP_DIR_IN;
        struct usb_dc_apollo4p_ep_state *ep_state = usb_dc_apollo4p_get_ep_state(ep);
        LOG_DBG("ep_addr 0x%02x", ep_addr);
        __ASSERT(ep_state, "No corresponding ep_state for ep");
        k_sem_give(&ep_state->write_sem);
        if (ep_state->cb) {
            ep_state->cb(ep, USB_DC_EP_DATA_IN);
        }
    }
    else if(USB_EP_DIR_IS_OUT(ep_addr))
    {
        uint8_t ep = ep_idx | USB_EP_DIR_OUT;
        struct usb_dc_apollo4p_ep_state *ep_state = usb_dc_apollo4p_get_ep_state(ep);

        LOG_DBG("ep_addr 0x%02x, xfer_len %u", ep_addr, xfer_len);

        /* Transaction complete, data is now stored in the buffer and ready
        * for the upper stack (usb_dc_ep_read to retrieve).
        */
        ep_state->read_count = xfer_len;
        ep_state->read_offset = 0U;

        if (ep_state->cb) {
            ep_state->cb(ep, USB_DC_EP_DATA_OUT);
        }
    }
}

static struct usb_dc_apollo4p_ep_state *usb_dc_apollo4p_get_ep_state(uint8_t ep)
{
    struct usb_dc_apollo4p_ep_state *ep_state_base;

    if (USB_EP_GET_IDX(ep) >= USB_NUM_BIDIR_ENDPOINTS) {
        return NULL;
    }

    if (USB_EP_DIR_IS_OUT(ep)) {
        ep_state_base = usb_dc_apollo4p_state.out_ep_state;
    } else {
        ep_state_base = usb_dc_apollo4p_state.in_ep_state;
    }

    return ep_state_base + USB_EP_GET_IDX(ep);
}

//*****************************************************************************
//
//! @brief setup and enable HFRC2 when usb mode is highspeed
//! 
//! @param enable  true will enable HFCR2
//! @param enable  false will disable HFCR2 
//
//*****************************************************************************
static uint32_t usb_dc_apollo4p_setHFCR2(bool bEnableClock)
{

    am_hal_usb_hs_clock_type am_hal_hfrc2_clock_type = AM_HAL_USB_HS_CLK_DISABLE ;
    if ( bEnableClock )
    {
#if BOARD_DEVICE_RHPORT_SPEED == OPT_MODE_HIGH_SPEED
        am_hal_hfrc2_clock_type = AM_HAL_USB_HS_CLK_HFRC2_ADJ;
#endif
    }
    return am_hal_usb_control(AM_HAL_CLKGEN_CONTROL_SET_HFRC2_TYPE, &am_hal_hfrc2_clock_type ) ;

}

static void am_usb_isr(const void *arg)
{
    ARG_UNUSED(arg);
    uint32_t ui32IntStatus[3];
    am_hal_usb_intr_status_get(pUSBHandle,
                               &ui32IntStatus[0],
                               &ui32IntStatus[1],
                               &ui32IntStatus[2]);
    am_hal_usb_interrupt_service(pUSBHandle,
                                 ui32IntStatus[0],
                                 ui32IntStatus[1],
                                 ui32IntStatus[2]);
}

//*****************************************************************************
// @brief  Controller API
//
// @param rhport
//*****************************************************************************
static int usb_dc_apollo4p_init(void)
{
    unsigned int i;
    int ret;

    //
    // Powerup Sequence
    // see Apollo4 Errata ERR041: Induced D+ output pulse may cause
    // unintended disconnect.
    //

    //
    // this function call doesn't set any USB registers, it configures the driver
    //
    uint32_t initStat = am_hal_usb_initialize(0, (void *) &pUSBHandle);
    if (initStat != AM_HAL_STATUS_SUCCESS)
        return -EIO;

    //
    // Register the callback functions
    //
    am_hal_usb_register_dev_evt_callback(pUSBHandle, usb_dc_apollo4p_evt_callback);
    am_hal_usb_register_ep0_setup_received_callback(pUSBHandle, usb_dc_apollo4p_ep0_setup_callback);
    am_hal_usb_register_ep_xfer_complete_callback(pUSBHandle, usb_dc_apollo4p_ep_xfer_complete_callback);

    //
    // Rev B power up sequence. ERR041
    //

    //
    // enable internal power rail
    //
    am_hal_usb_power_control(pUSBHandle, AM_HAL_SYSCTRL_WAKE, false);

    //
    // clear USB PHY reset in MCU control registers
    //
    am_hal_usb_enable_phy_reset_override();

    //
    // Enable the USB power rails
    //
    am_hal_gpio_pinconfig(13, g_AM_BSP_GPIO_VDDUSB33_SWITCH);
    am_hal_gpio_state_write(13, AM_HAL_GPIO_OUTPUT_SET);
    am_hal_gpio_pinconfig(15, g_AM_BSP_GPIO_VDDUSB0P9_SWITCH);
    am_hal_gpio_state_write(15, AM_HAL_GPIO_OUTPUT_SET);
    am_util_delay_ms(50);

    LOG_DBG("Pinctrl signals configuration");
    ret = pinctrl_apply_state(usb_pcfg, PINCTRL_STATE_DEFAULT);
    if (pinctrl_apply_state(usb_pcfg, PINCTRL_STATE_DEFAULT) < 0)
    {
        LOG_ERR("USB pinctrl setup failed (%d)", ret);
        return ret;
    }

    //
    // disable BC detection voltage source
    //
    am_hal_usb_hardware_unreset();
    //
    // start HFCR2 if the USB will run at HIGH speed
    // set argument to true, as we want the clock enabled if 
	// high speed is enabled
    //
    uint32_t ui32hfcr2_status = usb_dc_apollo4p_setHFCR2(true);

    //
    // set USB PHY reset disable
    //
    am_hal_usb_disable_phy_reset_override();

    am_hal_usb_dev_speed_e eUsbSpeed = AM_HAL_USB_SPEED_HIGH;

    if (ui32hfcr2_status == AM_HAL_STATUS_SUCCESS)
    {
        am_hal_usb_set_dev_speed(pUSBHandle, eUsbSpeed);
    }

    //
    // enable usb module interrupts
    //
    am_hal_usb_intr_usb_enable(pUSBHandle, USB_INTRUSB_Reset_Msk);

    //
    // this implements the soft connect
    //
    am_hal_usb_attach(pUSBHandle);

	usb_dc_apollo4p_state.out_ep_state[EP0_IDX].ep_mps = EP0_MPS;
	usb_dc_apollo4p_state.out_ep_state[EP0_IDX].ep_type = 0;//EP_TYPE_CTRL;
	usb_dc_apollo4p_state.in_ep_state[EP0_IDX].ep_mps = EP0_MPS;
	usb_dc_apollo4p_state.in_ep_state[EP0_IDX].ep_type = 0;//EP_TYPE_CTRL;

	/* Start PMA configuration for the endpoints after the BTABLE. */
	usb_dc_apollo4p_state.pma_offset = USB_BTABLE_SIZE;

	for (i = 0U; i < USB_NUM_BIDIR_ENDPOINTS; i++) {
		k_sem_init(&usb_dc_apollo4p_state.in_ep_state[i].write_sem, 1, 1);
	}

	IRQ_CONNECT(USB_IRQ, USB_IRQ_PRI,
		    am_usb_isr, 0, 0);
	irq_enable(USB_IRQ);
	return 0;
}

/* Zephyr USB device controller API implementation */

int usb_dc_attach(void)
{
    int ret;

    LOG_DBG("");

    ret = usb_dc_apollo4p_init();
    if (ret) {
        return ret;
    }

    return 0;
}

int usb_dc_ep_set_callback(const uint8_t ep, const usb_dc_ep_callback cb)
{
    struct usb_dc_apollo4p_ep_state *ep_state = usb_dc_apollo4p_get_ep_state(ep);

    LOG_DBG("ep 0x%02x", ep);

    if (!ep_state) {
        return -EINVAL;
    }

    ep_state->cb = cb;

    return 0;
}

void usb_dc_set_status_callback(const usb_dc_status_callback cb)
{
    LOG_DBG("");

    usb_dc_apollo4p_state.status_cb = cb;
}

int usb_dc_set_address(const uint8_t addr)
{
    LOG_DBG("addr %u (0x%02x)", addr, addr);

    // Response with status first before changing device address
    am_hal_usb_ep_xfer(pUSBHandle, EP0_IN, NULL, 0);

    am_hal_usb_set_addr(pUSBHandle, addr);
    am_hal_usb_set_dev_state(pUSBHandle, AM_HAL_USB_DEV_STATE_ADDRESSED);

    return 0;
}

int usb_dc_ep_start_read(uint8_t ep, uint8_t *data, uint32_t max_data_len)
{
	uint32_t status;

	LOG_DBG("ep 0x%02x, len %u", ep, max_data_len);

	/* we flush EP0_IN by doing a 0 length receive on it */
	if (!USB_EP_DIR_IS_OUT(ep) && (ep != EP0_IN || max_data_len)) {
		LOG_ERR("invalid ep 0x%02x", ep);
		return -EINVAL;
	}

	if (max_data_len > EP_MPS) {
		max_data_len = EP_MPS;
	}

    status = am_hal_usb_ep_xfer(pUSBHandle, ep,
                        usb_dc_apollo4p_state.ep_buf[USB_EP_GET_IDX(ep)],
                        max_data_len);

	if (status != AM_HAL_STATUS_SUCCESS) {
		LOG_ERR("am_hal_usb_ep_xfer failed(0x%02x), %d", ep,
			(int)status);
		return -EIO;
	}

	return 0;
}

int usb_dc_ep_check_cap(const struct usb_dc_ep_cfg_data * const cfg)
{
    uint8_t ep_idx = USB_EP_GET_IDX(cfg->ep_addr);

    LOG_DBG("ep %x, mps %d, type %d", cfg->ep_addr, cfg->ep_mps,
    cfg->ep_type);

    if ((cfg->ep_type == USB_DC_EP_CONTROL) && ep_idx) {
        LOG_ERR("invalid endpoint configuration");
        return -1;
    }

    if (ep_idx > (USB_NUM_BIDIR_ENDPOINTS - 1)) {
        LOG_ERR("endpoint index/address out of range");
        return -1;
    }

    return 0;
}

int usb_dc_ep_configure(const struct usb_dc_ep_cfg_data * const ep_cfg)
{
	uint8_t ep = ep_cfg->ep_addr;
	struct usb_dc_apollo4p_ep_state *ep_state = usb_dc_apollo4p_get_ep_state(ep);

	if (!ep_state) {
		return -EINVAL;
	}

	LOG_DBG("ep 0x%02x, previous ep_mps %u, ep_mps %u, ep_type %u",
		ep_cfg->ep_addr, ep_state->ep_mps, ep_cfg->ep_mps,
		ep_cfg->ep_type);

	if (ep_cfg->ep_mps > ep_state->ep_pma_buf_len) {
		if (USB_RAM_SIZE <=
		    (usb_dc_apollo4p_state.pma_offset + ep_cfg->ep_mps)) {
			return -EINVAL;
		}
/*        am_hal_usb_ep_init((void *) pUSBHandle,
                                      (ep_cfg->ep_addr & 0x7F),
                                      ((ep_cfg->ep_addr & 0x80) >> 7),
                                      ep_cfg->ep_mps);*/
		ep_state->ep_pma_buf_len = ep_cfg->ep_mps;
		usb_dc_apollo4p_state.pma_offset += ep_cfg->ep_mps;
	}
	ep_state->ep_mps = ep_cfg->ep_mps;

	switch (ep_cfg->ep_type) {
	case USB_DC_EP_CONTROL:
		ep_state->ep_type = 0;//EP_TYPE_CTRL;
		break;
	case USB_DC_EP_ISOCHRONOUS:
		ep_state->ep_type = 1;//EP_TYPE_ISOC;
		break;
	case USB_DC_EP_BULK:
		ep_state->ep_type = 2;//EP_TYPE_BULK;
		break;
	case USB_DC_EP_INTERRUPT:
		ep_state->ep_type = 3;//EP_TYPE_INTR;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int usb_dc_ep_set_stall(const uint8_t ep)
{
	struct usb_dc_apollo4p_ep_state *ep_state = usb_dc_apollo4p_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

    am_hal_usb_ep_stall(pUSBHandle, ep);

	ep_state->ep_stalled = 1U;

	return 0;
}

int usb_dc_ep_clear_stall(const uint8_t ep)
{
	struct usb_dc_apollo4p_ep_state *ep_state = usb_dc_apollo4p_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

    am_hal_usb_ep_clear_stall(pUSBHandle, ep);

	ep_state->ep_stalled = 0U;
	ep_state->read_count = 0U;

	return 0;
}

int usb_dc_ep_is_stalled(const uint8_t ep, uint8_t *const stalled)
{
	struct usb_dc_apollo4p_ep_state *ep_state = usb_dc_apollo4p_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state || !stalled) {
		return -EINVAL;
	}

	*stalled = ep_state->ep_stalled;

	return 0;
}

int usb_dc_ep_enable(const uint8_t ep)
{
	struct usb_dc_apollo4p_ep_state *ep_state = usb_dc_apollo4p_get_ep_state(ep);
	uint32_t status;

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

	LOG_DBG("am_hal_usb_ep_init(0x%02x, %u, %u)", ep, ep_state->ep_mps,
		ep_state->ep_type);

	status = am_hal_usb_ep_init(pUSBHandle, ep,
				 ep_state->ep_mps, ep_state->ep_type);
	if (status != AM_HAL_STATUS_SUCCESS) {
		LOG_ERR("am_hal_usb_ep_init failed(0x%02x), %d", ep,
			(int)status);
		return -EIO;
	}

	return 0;
}

int usb_dc_ep_disable(const uint8_t ep)
{
	struct usb_dc_apollo4p_ep_state *ep_state = usb_dc_apollo4p_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}
	LOG_ERR("Not implemented");

	return 0;
}

int usb_dc_ep_write(const uint8_t ep, const uint8_t *const data,
		    const uint32_t data_len, uint32_t * const ret_bytes)
{
	struct usb_dc_apollo4p_ep_state *ep_state = usb_dc_apollo4p_get_ep_state(ep);
	uint32_t status;
	uint32_t len = data_len;
	int ret = 0;

	LOG_DBG("ep 0x%02x, len %u", ep, data_len);

	if (!ep_state || !USB_EP_DIR_IS_IN(ep)) {
		LOG_ERR("invalid ep 0x%02x", ep);
		return -EINVAL;
	}

	ret = k_sem_take(&ep_state->write_sem, K_NO_WAIT);
	if (ret) {
		LOG_ERR("Unable to get write lock (%d)", ret);
		return -EAGAIN;
	}

	if (!k_is_in_isr()) {
		irq_disable(USB_IRQ);
	}

	if (ep == EP0_IN && len > USB_MAX_CTRL_MPS) {
		len = USB_MAX_CTRL_MPS;
	}

    status = am_hal_usb_ep_xfer(pUSBHandle, ep, data, len);

	if (status != AM_HAL_STATUS_SUCCESS) {
		LOG_ERR("am_hal_usb_ep_xfer failed(0x%02x), %d", ep,
			(int)status);
		k_sem_give(&ep_state->write_sem);
		ret = -EIO;
	}

	if (!ret && ep == EP0_IN && len > 0) {
		/* Wait for an empty package as from the host.
		 * This also flushes the TX FIFO to the host.
		 */
		usb_dc_ep_start_read(ep, NULL, 0);
	}

	if (!k_is_in_isr()) {
		irq_enable(USB_IRQ);
	}

	if (!ret && ret_bytes) {
		*ret_bytes = len;
	}

	return ret;
}

int usb_dc_ep_read_wait(uint8_t ep, uint8_t *data, uint32_t max_data_len,
			uint32_t *read_bytes)
{
	struct usb_dc_apollo4p_ep_state *ep_state = usb_dc_apollo4p_get_ep_state(ep);
	uint32_t read_count;

	if (!ep_state) {
		LOG_ERR("Invalid Endpoint %x", ep);
		return -EINVAL;
	}

	read_count = ep_state->read_count;

	LOG_DBG("ep 0x%02x, %u bytes, %u+%u, %p", ep, max_data_len,
		ep_state->read_offset, read_count, data);

	if (!USB_EP_DIR_IS_OUT(ep)) { /* check if OUT ep */
		LOG_ERR("Wrong endpoint direction: 0x%02x", ep);
		return -EINVAL;
	}

	/* When both buffer and max data to read are zero, just ignore reading
	 * and return available data in buffer. Otherwise, return data
	 * previously stored in the buffer.
	 */
	if (data) {
		read_count = MIN(read_count, max_data_len);
		memcpy(data, usb_dc_apollo4p_state.ep_buf[USB_EP_GET_IDX(ep)] +
		       ep_state->read_offset, read_count);
		ep_state->read_count -= read_count;
		ep_state->read_offset += read_count;
	} else if (max_data_len) {
		LOG_ERR("Wrong arguments");
	}

	if (read_bytes) {
		*read_bytes = read_count;
	}

	return 0;
}

int usb_dc_ep_read_continue(uint8_t ep)
{
	struct usb_dc_apollo4p_ep_state *ep_state = usb_dc_apollo4p_get_ep_state(ep);

	if (!ep_state || !USB_EP_DIR_IS_OUT(ep)) { /* Check if OUT ep */
		LOG_ERR("Not valid endpoint: %02x", ep);
		return -EINVAL;
	}

	/* If no more data in the buffer, start a new read transaction.
	 * DataOutStageCallback will called on transaction complete.
	 */
	if (!ep_state->read_count) {
		usb_dc_ep_start_read(ep, usb_dc_apollo4p_state.ep_buf[USB_EP_GET_IDX(ep)],
				     EP_MPS);
	}

	return 0;
}

int usb_dc_ep_read(const uint8_t ep, uint8_t *const data, const uint32_t max_data_len,
       uint32_t * const read_bytes)
{
    if (usb_dc_ep_read_wait(ep, data, max_data_len, read_bytes) != 0) {
        return -EINVAL;
    }

    if (usb_dc_ep_read_continue(ep) != 0) {
        return -EINVAL;
    }

    return 0;
}

int usb_dc_ep_halt(const uint8_t ep)
{
    return usb_dc_ep_set_stall(ep);
}

int usb_dc_ep_flush(const uint8_t ep)
{
    struct usb_dc_apollo4p_ep_state *ep_state = usb_dc_apollo4p_get_ep_state(ep);

    if (!ep_state) {
        return -EINVAL;
    }

    LOG_ERR("Not implemented");

    return 0;
}

int usb_dc_ep_mps(const uint8_t ep)
{
    struct usb_dc_apollo4p_ep_state *ep_state = usb_dc_apollo4p_get_ep_state(ep);

    if (!ep_state) {
        return -EINVAL;
    }

    return ep_state->ep_mps;
}

int usb_dc_wakeup_request(void)
{
    am_hal_usb_start_remote_wakeup(pUSBHandle);

    return 0;
}

int usb_dc_detach(void)
{
    //
    // detach from the host
    //
    am_hal_usb_detach(pUSBHandle);

    //
    // wait for any interrupts to happen and be processed
    //
    am_util_delay_ms(50);

    //
    // disable USB interrupts
    //
    am_hal_usb_intr_usb_disable(pUSBHandle,
                                (USB_INTRUSB_SOF_Msk |
                                 USB_INTRUSB_Reset_Msk |
                                 USB_INTRUSB_Resume_Msk |
                                 USB_INTRUSB_Suspend_Msk));

    //
    // disable high speed clock
    //
    am_hal_usb_dev_speed_e eUsbSpeed = AM_HAL_USB_SPEED_FULL;
    am_hal_usb_set_dev_speed(pUSBHandle, eUsbSpeed);

    //
    // disable HFCR2
    //
    usb_dc_apollo4p_setHFCR2(false);

    // power things down
    am_hal_usb_enable_phy_reset_override();
    am_hal_gpio_state_write(13, AM_HAL_GPIO_OUTPUT_CLEAR);
    am_hal_gpio_state_write(15, AM_HAL_GPIO_OUTPUT_CLEAR);
    am_util_delay_ms(50);
    am_hal_usb_power_control(pUSBHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false);

    am_hal_usb_deinitialize(pUSBHandle);

    pUSBHandle = NULL;

    if (irq_is_enabled(USB_IRQ)) {
        irq_disable(USB_IRQ);
    }

    return 0;
}

int usb_dc_reset(void)
{
    LOG_ERR("Not implemented");

    return 0;
}

