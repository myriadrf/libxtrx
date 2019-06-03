#ifndef XTRX_OCTO_API_H_
#define XTRX_OCTO_API_H_

#include <xtrx_api.h>

/** @file xtrx_octo_api.h
 *
 * Public API & constant for OCTO boards
 */

enum xtrx_octo_fe_regs {
    XTRX_OCTO_OSC_DAC  = XTRX_FE_CUSTOM_0 + 0x100,
    XTRX_OCTO_CAL_PATH = XTRX_FE_CUSTOM_0 + 0x101,
};

/**
 * @brief xtrx_octo_set_cal_path
 * @param dev
 * @param cal
 * @return 0 on success, -errno  otherwise
 */
static inline int xtrx_octo_set_cal_path(struct xtrx_dev* dev,
                                         bool cal)
{
    return xtrx_val_set(dev, XTRX_RX, XTRX_CH_ALL,
                        (xtrx_val_t)XTRX_OCTO_CAL_PATH, (uint64_t)cal);
}

/**
 * @brief xtrx_octo_set_osc_dac
 * @param dev
 * @param value
 * @return
 */
static inline int xtrx_octo_set_osc_dac(struct xtrx_dev* dev,
                                        uint16_t value)
{
    return xtrx_val_set(dev, XTRX_RX, XTRX_CH_ALL,
                        (xtrx_val_t)XTRX_OCTO_CAL_PATH, (uint64_t)value);
}


#endif
