/* shared_irq - Shared interrupt driver */

/*
 * Copyright (c) 2015 Intel corporation
 * Copyright (c) 2021 Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SHARED_IRQ_H_
#define ZEPHYR_INCLUDE_SHARED_IRQ_H_

#include <autoconf.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get a pointer within an shared-irqs specifier by name
 *
 * This can be used to get an individual shared-irq when a device generates more
 * than one, if the bindings give each shared-irq specifier a name.
 *
 * @param node_id Node identifier of a device which has a shared-irqs
 *                phandle-array property.
 * @param irq_name lowercase-and-underscores shared interrupt specifier name
 * @return the pointer to the shared-irq at the specifier given by the name
 */
#define SHARED_IRQ_BY_NAME(node_id, irq_name)                                  \
	DEVICE_DT_GET(DT_PHANDLE_BY_NAME(node_id, shared_irqs, irq_name))      \

/**
 * @brief Does the node have a shared-irq element with a matching name?
 *
 * @param node_id Node identifier of a device which may have a shared-irqs
 *                phandle-array element with the given name.
 * @param name lowercase-and-underscores name of a shared-irqs element
 *             as defined by the node's shared-irq-names property
 * @return 1 if the shared-irqs property has the named element, 0 otherwise
 */
#define SHARED_IRQ_HAS_NAME(node_id, name)                                     \
	IS_ENABLED(DT_CAT(node_id, _P_shared_irqs_NAME_##name##_EXISTS))

/**
 * @brief Connected to a devicetree nodes' irq by name.
 *
 * This routine connects to the irq with the given name
 *
 * The IRQ must be subsequently enabled before the interrupt handler
 * begins servicing interrupts.
 *
 * @warning
 * Although this routine is invoked at run-time, all of its arguments must be
 * computable by the compiler at build time.
 *
 * @param node_id node identifier of a device which has one or several
 *                named interrupts.
 * @param irq_name lowercase-and-underscores interrupt specifier name
 * @param isr_p Address of interrupt service routine.
 * @param isr_param_p Parameter passed to interrupt service routine.
 * @param flags_p Architecture-specific IRQ configuration flags
 */
#define IRQ_CONNECT_BY_NAME(node_id, irq_name, isr_p, isr_param_p, flags_p)    \
	IRQ_CONNECT(DT_IRQ_BY_NAME(node_id, irq_name, irq),                    \
		    DT_IRQ_BY_NAME(node_id, irq_name, priority),               \
		    isr_p, isr_param_p, flags_p)                               \

/**
 * @brief Connected to a devicetree nodes shared-irq by name.
 *
 * This routine connects to the shared-irqs element with the given name
 *
 * The IRQ must be subsequently enabled before the interrupt handler
 * begins servicing interrupts.
 *
 * @warning
 * Although this routine is invoked at run-time, all of its arguments must be
 * computable by the compiler at build time.
 *
 * @param node_id Node identifier of a device which has a shared-irqs
 *                phandle-array property.
 * @param irq_name lowercase-and-underscores shared-irq specifier name
 * @param isr_p Address of interrupt service routine.
 * @param isr_param_p Parameter passed to interrupt service routine.
 */
#define SHARED_IRQ_CONNECT_BY_NAME(node_id, irq_name, isr_p, isr_param_p)      \
	__ASSERT(device_is_ready(SHARED_IRQ_BY_NAME(node_id, irq_name)),       \
		 "shared-irq ##irq_idx## not ready");                          \
	shared_irq_isr_register(SHARED_IRQ_BY_NAME(node_id, irq_name),         \
				(isr_t)isr_p, isr_param_p)                     \

/**
 * @brief Connected to a devicetree nodes shared-irq by name if it exists,
 * else connect to a irq using the same given name.
 *
 * This routine connects to the shared-irqs element with the given name,
 * if such a shared-irq exists.
 * Otherwise it initializes an interrupt handler for an IRQ.
 * The IRQ must be subsequently enabled before the interrupt handler
 * begins servicing interrupts.
 *
 * @warning
 * Although this routine is invoked at run-time, all of its arguments must be
 * computable by the compiler at build time.
 *
 * @param node_id Node identifier of a device which has either a shared-irqs
 *                phandle-array or a interrupts property.
 * @param irq_name lowercase-and-underscores device shared-irq-name if exists,
 *                 interrupt-name otherwise.
 * @param isr_p Address of interrupt service routine.
 * @param isr_param_p Parameter passed to interrupt service routine.
 * @param flags_p Architecture-specific IRQ configuration flags (not used for
 *        shared-irqs).
 *
 * @return N/A
*/
#define SHARED_IRQ_CONNECT_IRQ_BY_NAME_COND(node_id, irq_name, isr_p,          \
					    isr_param_p, flags_p)              \
	COND_CODE_1(SHARED_IRQ_HAS_NAME(node_id, irq_name),                    \
		    (SHARED_IRQ_CONNECT_BY_NAME(node_id, irq_name,             \
						isr_p, isr_param_p)),          \
		    (IRQ_CONNECT_BY_NAME(node_id, irq_name, isr_p, isr_param_p,\
					 flags_p)))

/**
 * @brief Connect to a DT_DRV_COMPAT shared-irq by name if it exists,
 * else connect to a irq by the given name.
 *
 * This routine connects to a DT_DRV_COMPAT shared-irq with the given name,
 * if the given name exists.
 * Otherwise it initializes an interrupt handler for an irq with the given name.
 * The IRQ must be subsequently enabled before the interrupt handler
 * begins servicing interrupts.
 *
 * @warning
 * Although this routine is invoked at run-time, all of its arguments must be
 * computable by the compiler at build time.
 *
 * @param inst peripheral node identifier which has either a shared-irqs
 *             phandle-array or a interrupts property.
 * @param irq_name lowercase-and-underscores device shared-irqs name if exists,
 *                 irq name otherwise.
 * @param isr_p Address of interrupt service routine.
 * @param isr_param_p Parameter passed to interrupt service routine.
 * @param flags_p Architecture-specific IRQ configuration flags (not used for
 *        shared-irq).
 *
 * @return N/A
 */
#define SHARED_IRQ_INST_CONNECT_IRQ_BY_NAME_COND(inst, irq_name, isr_p,        \
						 isr_param_p, flags_p)         \
	SHARED_IRQ_CONNECT_IRQ_BY_NAME_COND(DT_DRV_INST(inst), irq_name,       \
						 isr_p, isr_param_p, flags_p)

/**
 * @brief Enable an devicetree nodes shared-irq by name if it exists,
 *        or an irq by name otherwise.
 *
 * This routine enables a device nodes interrupt with the matching irq_name.
 * In case a shared-irqs element with a matching name exists it is enabled,
 * otherwise a irq with a matching name is enabled.
 *
 * @param node_id Node identifier of a device which may have a shared-irqs
 *                phandle-array property.
 * @param irq_name lowercase-and-underscores device shared-irqs name if it
 *                 exists, irq name otherwise.
 *
 * @return N/A
 */
#define SHARED_IRQ_ENABLE_BY_NAME_COND(node_id, irq_name)                      \
	COND_CODE_1(SHARED_IRQ_HAS_NAME(node_id, irq_name),                    \
		    (shared_irq_enable(SHARED_IRQ_BY_NAME(node_id, irq_name),  \
				       DEVICE_DT_GET(node_id))),               \
		    (irq_enable(DT_IRQ_BY_NAME(node_id, irq_name, irq))))      \

/**
 * @brief Enable an DT_DRV_COMPAT shared-irq by name if it exists,
 *        or an irq by name otherwise.
 *
 * This routine enables a DT_DRV_COMPAT device nodes' interrupt with the
 * matching irq_name.
 * In case a shared-irqs element with a matching name exists it is enabled,
 * otherwise a irq with a matching name is enabled.
 *
 * @param inst Instance number of a DT_DRV_COMPAT device which may have a
 *             shared-irqs phandle-array property.
 * @param irq_name lowercase-and-underscores device shared-irqs name if exists,
 *                 irq name otherwise.
 *
 * @return N/A
 */
#define SHARED_IRQ_INST_ENABLE_BY_NAME_COND(inst, irq_name)                    \
	SHARED_IRQ_ENABLE_BY_NAME_COND(DT_DRV_INST(inst), irq_name)            \

/**
 * @brief Disable an devicetree nodes shared-irq by name if it exists,
 *        or an irq by name otherwise.
 *
 * This routine disables a device nodes interrupt with the matching irq_name.
 * In case a shared-irqs element with a matching name exists it is disalbed,
 * otherwise a irq with a matching name is disabled.
 *
 * @param node_id Node identifier of a device which may have a shared-irqs
 *                phandle-array property.
 * @param irq_name lowercase-and-underscores device shared-irqs name if it
 *                 exists, irq name otherwise.
 *
 * @return N/A
 */
#define SHARED_IRQ_DISABLE_BY_NAME_COND(node_id, irq_name)                     \
	COND_CODE_1(SHARED_IRQ_HAS_NAME(node_id, irq_name),                    \
		    (shared_irq_disable(SHARED_IRQ_BY_NAME(node_id, irq_name), \
				       DEVICE_DT_GET(node_id))),               \
		    (irq_disable(DT_IRQ_BY_NAME(node_id, irq_name, irq))))     \

/**
 * @brief Disable an DT_DRV_COMPAT shared-irq by name if it exists,
 *        or an irq by name otherwise.
 *
 * This routine disables a DT_DRV_COMPAT device nodes' interrupt with the
 * matching irq_name.
 * In case a shared-irqs element with a matching name exists it is disabled,
 * otherwise a irq with a matching name is disabled.
 *
 * @param inst Instance number of a DT_DRV_COMPAT device which may have a
 *             shared-irqs phandle-array property.
 * @param irq_name lowercase-and-underscores device shared-irqs name if exists,
 *                 irq name otherwise.
 *
 * @return N/A
 */
#define SHARED_IRQ_INST_DISABLE_BY_NAME_COND(inst, irq_name)                   \
	SHARED_IRQ_DISABLE_BY_NAME_COND(DT_DRV_INST(inst), irq_name)           \


typedef int (*isr_t)(const struct device *dev);

/* driver API definition */
typedef int (*shared_irq_register_t)(const struct device *dev,
				     isr_t isr_func,
				     const struct device *isr_dev);
typedef int (*shared_irq_enable_t)(const struct device *dev,
				   const struct device *isr_dev);
typedef int (*shared_irq_disable_t)(const struct device *dev,
				    const struct device *isr_dev);

struct shared_irq_driver_api {
	shared_irq_register_t isr_register;
	shared_irq_enable_t enable;
	shared_irq_disable_t disable;
};

/**
 *  @brief Register a device ISR
 *  @param dev Pointer to device structure for SHARED_IRQ driver instance.
 *  @param isr_func Pointer to the ISR function for the device.
 *  @param isr_dev Pointer to the device that will service the interrupt.
 */
static inline int shared_irq_isr_register(const struct device *dev,
					  isr_t isr_func,
					  const struct device *isr_dev)
{
	const struct shared_irq_driver_api *api =
		(const struct shared_irq_driver_api *)dev->api;

	return api->isr_register(dev, isr_func, isr_dev);
}

/**
 *  @brief Enable ISR for device
 *  @param dev Pointer to device structure for SHARED_IRQ driver instance.
 *  @param isr_dev Pointer to the device that will service the interrupt.
 */
static inline int shared_irq_enable(const struct device *dev,
				    const struct device *isr_dev)
{
	const struct shared_irq_driver_api *api =
		(const struct shared_irq_driver_api *)dev->api;

	return api->enable(dev, isr_dev);
}

/**
 *  @brief Disable ISR for device
 *  @param dev Pointer to device structure for SHARED_IRQ driver instance.
 *  @param isr_dev Pointer to the device that will service the interrupt.
 */
static inline int shared_irq_disable(const struct device *dev,
				     const struct device *isr_dev)
{
	const struct shared_irq_driver_api *api =
		(const struct shared_irq_driver_api *)dev->api;

	return api->disable(dev, isr_dev);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SHARED_IRQ_H_ */
