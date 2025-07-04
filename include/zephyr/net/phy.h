/**
 * @file
 *
 * @brief Public APIs for Ethernet PHY drivers.
 */

/*
 * Copyright (c) 2021 IP-Logix Inc.
 * Copyright 2022 NXP
 * Copyright (c) 2025 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_PHY_H_
#define ZEPHYR_INCLUDE_DRIVERS_PHY_H_

/**
 * @brief Ethernet PHY Interface
 * @defgroup ethernet_phy Ethernet PHY Interface
 * @since 2.7
 * @version 0.8.0
 * @ingroup networking
 * @{
 */
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/sys/util_macro.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Ethernet link speeds. */
enum phy_link_speed {
	/** 10Base Half-Duplex */
	LINK_HALF_10BASE = BIT(0),
	/** 10Base Full-Duplex */
	LINK_FULL_10BASE = BIT(1),
	/** 100Base Half-Duplex */
	LINK_HALF_100BASE = BIT(2),
	/** 100Base Full-Duplex */
	LINK_FULL_100BASE = BIT(3),
	/** 1000Base Half-Duplex */
	LINK_HALF_1000BASE = BIT(4),
	/** 1000Base Full-Duplex */
	LINK_FULL_1000BASE = BIT(5),
	/** 2.5GBase Full-Duplex */
	LINK_FULL_2500BASE = BIT(6),
	/** 5GBase Full-Duplex */
	LINK_FULL_5000BASE = BIT(7),
};

/**
 * @brief Check if phy link is full duplex.
 *
 * @param x Link capabilities
 *
 * @return True if link is full duplex, false if not.
 */
#define PHY_LINK_IS_FULL_DUPLEX(x)                                                                 \
	(x & (LINK_FULL_10BASE | LINK_FULL_100BASE | LINK_FULL_1000BASE | LINK_FULL_2500BASE |     \
	      LINK_FULL_5000BASE))

/**
 * @brief Check if phy link speed is 1 Gbit/sec.
 *
 * @param x Link capabilities
 *
 * @return True if link is 1 Gbit/sec, false if not.
 */
#define PHY_LINK_IS_SPEED_1000M(x) (x & (LINK_HALF_1000BASE | LINK_FULL_1000BASE))

/**
 * @brief Check if phy link speed is 100 Mbit/sec.
 *
 * @param x Link capabilities
 *
 * @return True if link is 100 Mbit/sec, false if not.
 */
#define PHY_LINK_IS_SPEED_100M(x) (x & (LINK_HALF_100BASE | LINK_FULL_100BASE))

/**
 * @brief Check if phy link speed is 10 Mbit/sec.
 *
 * @param x Link capabilities
 *
 * @return True if link is 10 Mbit/sec, false if not.
 */
#define PHY_LINK_IS_SPEED_10M(x) (x & (LINK_HALF_10BASE | LINK_FULL_10BASE))

/** @brief Link state */
struct phy_link_state {
	/** Link speed */
	enum phy_link_speed speed;
	/** When true the link is active and connected */
	bool is_up;
};

/** @brief Ethernet configure link flags. */
enum phy_cfg_link_flag {
	/** Auto-negotiation disable */
	PHY_FLAG_AUTO_NEGOTIATION_DISABLED = BIT(0),
};

/** @brief PLCA (Physical Layer Collision Avoidance) Reconciliation Sublayer configurations */
struct phy_plca_cfg {
	/** PLCA register map version */
	uint8_t version;
	/** PLCA configured mode (enable/disable) */
	bool enable;
	/** PLCA local node identifier */
	uint8_t node_id;
	/** PLCA node count */
	uint8_t node_count;
	/** Additional frames a node is allowed to send in single transmit opportunity (TO) */
	uint8_t burst_count;
	/** Wait time for the MAC to send a new frame before interrupting the burst */
	uint8_t burst_timer;
	/** PLCA to_timer in bit-times, which determines the PLCA transmit opportunity */
	uint8_t to_timer;
};

/**
 * @brief      Write PHY PLCA configuration
 *
 * This routine provides a generic interface to configure PHY PLCA settings.
 *
 * @param[in]  dev       PHY device structure
 * @param[in]  plca_cfg  Pointer to plca configuration structure
 *
 * @retval 0 If successful.
 * @retval -EIO If communication with PHY failed.
 */
int genphy_get_plca_cfg(const struct device *dev, struct phy_plca_cfg *plca_cfg);

/**
 * @brief      Read PHY PLCA configuration
 *
 * This routine provides a generic interface to get PHY PLCA settings.
 *
 * @param[in]  dev       PHY device structure
 * @param      plca_cfg  Pointer to plca configuration structure
 *
 * @retval 0 If successful.
 * @retval -EIO If communication with PHY failed.
 */
int genphy_set_plca_cfg(const struct device *dev, struct phy_plca_cfg *plca_cfg);

/**
 * @brief      Read PHY PLCA status
 *
 * This routine provides a generic interface to get PHY PLCA status.
 *
 * @param[in]  dev          PHY device structure
 * @param      plca_status  Pointer to plca status
 *
 * @retval 0 If successful.
 * @retval -EIO If communication with PHY failed.
 */
int genphy_get_plca_sts(const struct device *dev, bool *plca_status);

/**
 * @typedef phy_callback_t
 * @brief Define the callback function signature for
 * `phy_link_callback_set()` function.
 *
 * @param dev       PHY device structure
 * @param state     Pointer to link_state structure.
 * @param user_data Pointer to data specified by user
 */
typedef void (*phy_callback_t)(const struct device *dev, struct phy_link_state *state,
			       void *user_data);

/**
 * @cond INTERNAL_HIDDEN
 *
 * These are for internal use only, so skip these in
 * public documentation.
 */
__subsystem struct ethphy_driver_api {
	/** Get link state */
	int (*get_link)(const struct device *dev, struct phy_link_state *state);

	/** Configure link */
	int (*cfg_link)(const struct device *dev, enum phy_link_speed adv_speeds,
			enum phy_cfg_link_flag flags);

	/** Set callback to be invoked when link state changes. Driver has to invoke
	 * callback once after setting it, even if link state has not changed.
	 */
	int (*link_cb_set)(const struct device *dev, phy_callback_t cb, void *user_data);

	/** Read PHY register */
	int (*read)(const struct device *dev, uint16_t reg_addr, uint32_t *data);

	/** Write PHY register */
	int (*write)(const struct device *dev, uint16_t reg_addr, uint32_t data);

	/** Read PHY C45 register */
	int (*read_c45)(const struct device *dev, uint8_t devad, uint16_t regad, uint16_t *data);

	/** Write PHY C45 register */
	int (*write_c45)(const struct device *dev, uint8_t devad, uint16_t regad, uint16_t data);

	/* Set PLCA settings */
	int (*set_plca_cfg)(const struct device *dev, struct phy_plca_cfg *plca_cfg);

	/* Get PLCA settings */
	int (*get_plca_cfg)(const struct device *dev, struct phy_plca_cfg *plca_cfg);

	/* Get PLCA status */
	int (*get_plca_sts)(const struct device *dev, bool *plca_sts);
};
/**
 * @endcond
 */

/**
 * @brief      Configure PHY link
 *
 * This route configures the advertised link speeds.
 *
 * @param[in]  dev     PHY device structure
 * @param      speeds  OR'd link speeds to be advertised by the PHY
 * @param      flags   OR'd flags to control the link configuration or 0.
 *
 * @retval 0 If successful.
 * @retval -EIO If communication with PHY failed.
 * @retval -ENOTSUP If not supported.
 * @retval -EALREADY If already configured with the same speeds and flags.
 */
static inline int phy_configure_link(const struct device *dev, enum phy_link_speed speeds,
				     enum phy_cfg_link_flag flags)
{
	if (DEVICE_API_GET(ethphy, dev)->cfg_link == NULL) {
		return -ENOSYS;
	}

	/* Check if only one speed is set, when auto-negotiation is disabled */
	if ((flags & PHY_FLAG_AUTO_NEGOTIATION_DISABLED) && !IS_POWER_OF_TWO(speeds)) {
		return -EINVAL;
	}

	return DEVICE_API_GET(ethphy, dev)->cfg_link(dev, speeds, flags);
}

/**
 * @brief      Get PHY link state
 *
 * Returns the current state of the PHY link. This can be used by
 * to determine when a link is up and the negotiated link speed.
 *
 *
 * @param[in]  dev    PHY device structure
 * @param      state  Pointer to receive PHY state
 *
 * @retval 0 If successful.
 * @retval -EIO If communication with PHY failed.
 */
static inline int phy_get_link_state(const struct device *dev, struct phy_link_state *state)
{
	if (DEVICE_API_GET(ethphy, dev)->get_link == NULL) {
		return -ENOSYS;
	}

	return DEVICE_API_GET(ethphy, dev)->get_link(dev, state);
}

/**
 * @brief      Set link state change callback
 *
 * Sets a callback that is invoked when link state changes. This is the
 * preferred method for ethernet drivers to be notified of the PHY link
 * state change. The callback will be invoked once after setting it,
 * even if link state has not changed. There can only one callback
 * function set and active at a time. This function is mainly used
 * by ethernet drivers to register a callback to be notified of
 * link state changes and should therefore not be used by applications.
 *
 * @param[in]  dev        PHY device structure
 * @param      callback   Callback handler
 * @param      user_data  Pointer to data specified by user.
 *
 * @retval 0 If successful.
 * @retval -ENOTSUP If not supported.
 */
static inline int phy_link_callback_set(const struct device *dev, phy_callback_t callback,
					void *user_data)
{
	if (DEVICE_API_GET(ethphy, dev)->link_cb_set == NULL) {
		return -ENOSYS;
	}

	return DEVICE_API_GET(ethphy, dev)->link_cb_set(dev, callback, user_data);
}

/**
 * @brief      Read PHY registers
 *
 * This routine provides a generic interface to read from a PHY register.
 *
 * @param[in]  dev       PHY device structure
 * @param[in]  reg_addr  Register address
 * @param      value     Pointer to receive read value
 *
 * @retval 0 If successful.
 * @retval -EIO If communication with PHY failed.
 */
static inline int phy_read(const struct device *dev, uint16_t reg_addr, uint32_t *value)
{
	if (DEVICE_API_GET(ethphy, dev)->read == NULL) {
		return -ENOSYS;
	}

	return DEVICE_API_GET(ethphy, dev)->read(dev, reg_addr, value);
}

/**
 * @brief      Write PHY register
 *
 * This routine provides a generic interface to write to a PHY register.
 *
 * @param[in]  dev       PHY device structure
 * @param[in]  reg_addr  Register address
 * @param[in]  value     Value to write
 *
 * @retval 0 If successful.
 * @retval -EIO If communication with PHY failed.
 */
static inline int phy_write(const struct device *dev, uint16_t reg_addr, uint32_t value)
{
	if (DEVICE_API_GET(ethphy, dev)->write == NULL) {
		return -ENOSYS;
	}

	return DEVICE_API_GET(ethphy, dev)->write(dev, reg_addr, value);
}

/**
 * @brief      Read PHY C45 register
 *
 * This routine provides a generic interface to read to a PHY C45 register.
 *
 * @param[in]  dev       PHY device structure
 * @param[in]  devad       Device address
 * @param[in]  regad       Register address
 * @param      data        Pointer to receive read data
 *
 * @retval 0 If successful.
 * @retval -EIO If communication with PHY failed.
 */
static inline int phy_read_c45(const struct device *dev, uint8_t devad, uint16_t regad,
			       uint16_t *data)
{
	if (DEVICE_API_GET(ethphy, dev)->read_c45 == NULL) {
		return -ENOSYS;
	}

	return DEVICE_API_GET(ethphy, dev)->read_c45(dev, devad, regad, data);
}

/**
 * @brief      Write PHY C45 register
 *
 * This routine provides a generic interface to write to a PHY C45 register.
 *
 * @param[in]  dev       PHY device structure
 * @param[in]  devad       Device address
 * @param[in]  regad       Register address
 * @param[in]  data        Data to write
 *
 * @retval 0 If successful.
 * @retval -EIO If communication with PHY failed.
 */
static inline int phy_write_c45(const struct device *dev, uint8_t devad, uint16_t regad,
				uint16_t data)
{
	if (DEVICE_API_GET(ethphy, dev)->write_c45 == NULL) {
		return -ENOSYS;
	}

	return DEVICE_API_GET(ethphy, dev)->write_c45(dev, devad, regad, data);
}

/**
 * @brief      Write PHY PLCA configuration
 *
 * This routine provides a generic interface to configure PHY PLCA settings.
 *
 * @param[in]  dev       PHY device structure
 * @param[in]  plca_cfg  Pointer to plca configuration structure
 *
 * @retval 0 If successful.
 * @retval -EIO If communication with PHY failed.
 */
static inline int phy_set_plca_cfg(const struct device *dev, struct phy_plca_cfg *plca_cfg)
{
	if (DEVICE_API_GET(ethphy, dev)->set_plca_cfg == NULL) {
		return -ENOSYS;
	}

	return DEVICE_API_GET(ethphy, dev)->set_plca_cfg(dev, plca_cfg);
}

/**
 * @brief      Read PHY PLCA configuration
 *
 * This routine provides a generic interface to get PHY PLCA settings.
 *
 * @param[in]  dev       PHY device structure
 * @param      plca_cfg  Pointer to plca configuration structure
 *
 * @retval 0 If successful.
 * @retval -EIO If communication with PHY failed.
 */
static inline int phy_get_plca_cfg(const struct device *dev, struct phy_plca_cfg *plca_cfg)
{
	if (DEVICE_API_GET(ethphy, dev)->get_plca_cfg == NULL) {
		return -ENOSYS;
	}

	return DEVICE_API_GET(ethphy, dev)->get_plca_cfg(dev, plca_cfg);
}

/**
 * @brief      Read PHY PLCA status
 *
 * This routine provides a generic interface to get PHY PLCA status.
 *
 * @param[in]  dev          PHY device structure
 * @param      plca_status  Pointer to plca status
 *
 * @retval 0 If successful.
 * @retval -EIO If communication with PHY failed.
 */
static inline int phy_get_plca_sts(const struct device *dev, bool *plca_status)
{
	if (DEVICE_API_GET(ethphy, dev)->get_plca_sts == NULL) {
		return -ENOSYS;
	}

	return DEVICE_API_GET(ethphy, dev)->get_plca_sts(dev, plca_status);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_DRIVERS_PHY_H_ */
