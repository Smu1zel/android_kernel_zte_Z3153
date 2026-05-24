/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include "regulator.h"
#include "upmu_common.h"

#include <mt-plat/aee.h>
#include <asm/siginfo.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>

#if defined(HI1634Z6250CC142_MIPI_RAW)
static int regulator_status[IMGSENSOR_SENSOR_IDX_MAX_NUM][REGULATOR_TYPE_MAX_NUM];
static void check_for_regulator_get(struct REGULATOR *preg, struct device *pdevice,
			enum IMGSENSOR_SENSOR_IDX sensor_idx, int reg_type);
static void check_for_regulator_put(struct REGULATOR *preg,
			enum IMGSENSOR_SENSOR_IDX sensor_idx, int reg_type);
static DEFINE_MUTEX(g_regulator_state_mutex);
struct device *cam_device = NULL;
static struct device_node *of_node_record = NULL;
#endif

static const int regulator_voltage[] = {
	REGULATOR_VOLTAGE_0,
	REGULATOR_VOLTAGE_1000,
	REGULATOR_VOLTAGE_1100,
	REGULATOR_VOLTAGE_1200,
	REGULATOR_VOLTAGE_1210,
	REGULATOR_VOLTAGE_1220,
	REGULATOR_VOLTAGE_1500,
	REGULATOR_VOLTAGE_1800,
	REGULATOR_VOLTAGE_2500,
	REGULATOR_VOLTAGE_2800,
	REGULATOR_VOLTAGE_2900,
};

struct REGULATOR_CTRL regulator_control[REGULATOR_TYPE_MAX_NUM] = {
	{"vcama"},
	{"vcamd"},
	{"vcamio"},
};

static const int int_oc_type[REGULATOR_TYPE_MAX_NUM] = {
	INT_VCAMA_OC,
	INT_VCAMD_OC,
	INT_VCAMIO_OC,
};


static struct REGULATOR reg_instance;

static void imgsensor_oc_handler1(void)
{
	pr_debug("[regulator]%s enter vcama oc %d\n",
		__func__,
		gimgsensor.status.oc);
	gimgsensor.status.oc = 1;
	aee_kernel_warning("Imgsensor OC", "Over current");
	if (reg_instance.pid != -1 &&
		pid_task(find_get_pid(reg_instance.pid), PIDTYPE_PID) != NULL)
		force_sig(SIGKILL,
				pid_task(find_get_pid(reg_instance.pid),
						PIDTYPE_PID));

}
static void imgsensor_oc_handler2(void)
{
	pr_debug("[regulator]%s enter vcamd oc %d\n",
		__func__,
		gimgsensor.status.oc);
	gimgsensor.status.oc = 1;
	aee_kernel_warning("Imgsensor OC", "Over current");
	if (reg_instance.pid != -1 &&
		pid_task(find_get_pid(reg_instance.pid), PIDTYPE_PID) != NULL)
		force_sig(SIGKILL,
				pid_task(find_get_pid(reg_instance.pid),
						PIDTYPE_PID));
}
static void imgsensor_oc_handler3(void)
{
	pr_debug("[regulator]%s enter vcamio oc %d\n",
		__func__,
		gimgsensor.status.oc);
	gimgsensor.status.oc = 1;
	aee_kernel_warning("Imgsensor OC", "Over current");
	if (reg_instance.pid != -1 &&
		pid_task(find_get_pid(reg_instance.pid), PIDTYPE_PID) != NULL)
		force_sig(SIGKILL,
				pid_task(find_get_pid(reg_instance.pid),
						PIDTYPE_PID));

}

#define OC_MODULE "camera"
enum IMGSENSOR_RETURN imgsensor_oc_interrupt(
	enum IMGSENSOR_SENSOR_IDX sensor_idx, bool enable)
{
	struct regulator *preg = NULL;
	struct device *pdevice = gimgsensor_device;
	char str_regulator_name[LENGTH_FOR_SNPRINTF];
	int i = 0;
	gimgsensor.status.oc = 0;

	if (enable) {
		mdelay(5);
		for (i = 0; i < REGULATOR_TYPE_MAX_NUM; i++) {
			snprintf(str_regulator_name,
					sizeof(str_regulator_name),
					"cam%d_%s",
					sensor_idx,
					regulator_control[i].pregulator_type);
			preg = regulator_get(pdevice, str_regulator_name);
			if (preg && regulator_is_enabled(preg)) {
				pmic_enable_interrupt(
					int_oc_type[i], 1, OC_MODULE);
				regulator_put(preg);
				pr_debug(
					"[regulator] %s idx=%d %s enable=%d\n",
					__func__,
					sensor_idx,
					regulator_control[i].pregulator_type,
					enable);
			}
		}
		rcu_read_lock();
		reg_instance.pid = current->tgid;
		rcu_read_unlock();
	} else {
		reg_instance.pid = -1;
		/* Disable interrupt before power off */

		for (i = 0; i < REGULATOR_TYPE_MAX_NUM; i++) {
			snprintf(str_regulator_name,
					sizeof(str_regulator_name),
					"cam%d_%s",
					sensor_idx,
					regulator_control[i].pregulator_type);
			preg = regulator_get(pdevice, str_regulator_name);
			if (preg) {
				pmic_enable_interrupt(
					int_oc_type[i], 0, OC_MODULE);
				regulator_put(preg);
				pr_debug("[regulator] %s idx=%d %s enable=%d\n",
					__func__,
					sensor_idx,
					regulator_control[i].pregulator_type,
					enable);
			}
		}

	}

	return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN imgsensor_oc_init(void)
{
	/* Register your interrupt handler of OC interrupt at first */
	pmic_register_interrupt_callback(INT_VCAMA_OC, imgsensor_oc_handler1);
	pmic_register_interrupt_callback(INT_VCAMD_OC, imgsensor_oc_handler2);
	pmic_register_interrupt_callback(INT_VCAMIO_OC, imgsensor_oc_handler3);

	gimgsensor.status.oc  = 0;
	gimgsensor.imgsensor_oc_irq_enable = imgsensor_oc_interrupt;
	reg_instance.pid = -1;

	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN regulator_init(void *pinstance)
{
	struct REGULATOR *preg = (struct REGULATOR *)pinstance;
	struct device            *pdevice;
	struct device_node       *pof_node;
	int j, i;
	char str_regulator_name[LENGTH_FOR_SNPRINTF];

	pdevice  = gimgsensor_device;
	pof_node = pdevice->of_node;
	pdevice->of_node =
		of_find_compatible_node(NULL, NULL, "mediatek,camera_hw");

	if (pdevice->of_node == NULL) {
		pr_err("regulator get cust camera node failed!\n");
		pdevice->of_node = pof_node;
		return IMGSENSOR_RETURN_ERROR;
	}

#if defined(HI1634Z6250CC142_MIPI_RAW)
		cam_device = pdevice;
		of_node_record = pdevice->of_node;
#endif

	for (j = IMGSENSOR_SENSOR_IDX_MIN_NUM;
		j < IMGSENSOR_SENSOR_IDX_MAX_NUM;
		j++) {
		for (i = 0; i < REGULATOR_TYPE_MAX_NUM; i++) {
			snprintf(str_regulator_name,
				sizeof(str_regulator_name),
				"cam%d_%s",
				j,
				regulator_control[i].pregulator_type);
			preg->pregulator[j][i] =
			    regulator_get(pdevice, str_regulator_name);

			if (preg->pregulator[j][i] == NULL)
				pr_err("regulator[%d][%d]  %s fail!\n",
					j, i, str_regulator_name);

			atomic_set(&preg->enable_cnt[j][i], 0);
#if defined(HI1634Z6250CC142_MIPI_RAW)
			regulator_status[j][i] = 1;
#endif
		}
	}
	pdevice->of_node = pof_node;
	imgsensor_oc_init();
	return IMGSENSOR_RETURN_SUCCESS;
}
static enum IMGSENSOR_RETURN regulator_release(void *pinstance)
{
	struct REGULATOR *preg = (struct REGULATOR *)pinstance;
	int type, idx;
	struct regulator *pregulator = NULL;
	atomic_t *enable_cnt = NULL;

	for (idx = IMGSENSOR_SENSOR_IDX_MIN_NUM;
		idx < IMGSENSOR_SENSOR_IDX_MAX_NUM;
		idx++) {

		for (type = 0; type < REGULATOR_TYPE_MAX_NUM; type++) {
			pregulator = preg->pregulator[idx][type];
			enable_cnt = &preg->enable_cnt[idx][type];
			if (pregulator != NULL) {
				for (; atomic_read(enable_cnt) > 0; ) {
					regulator_disable(pregulator);
					atomic_dec(enable_cnt);
#if defined(HI1634Z6250CC142_MIPI_RAW)
				check_for_regulator_put(preg, idx, type);
#endif
				}
			}
		}
	}
	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN regulator_set(
	void *pinstance,
	enum IMGSENSOR_SENSOR_IDX   sensor_idx,
	enum IMGSENSOR_HW_PIN       pin,
	enum IMGSENSOR_HW_PIN_STATE pin_state)
{
	struct regulator     *pregulator;
	struct REGULATOR     *preg = (struct REGULATOR *)pinstance;
	int reg_type_offset;
	atomic_t             *enable_cnt;


	if (pin > IMGSENSOR_HW_PIN_DOVDD   ||
		pin < IMGSENSOR_HW_PIN_AVDD    ||
		pin_state < IMGSENSOR_HW_PIN_STATE_LEVEL_0 ||
		pin_state >= IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH)
		return IMGSENSOR_RETURN_ERROR;

	reg_type_offset = REGULATOR_TYPE_VCAMA;

#if defined(HI1634Z6250CC142_MIPI_RAW)
		check_for_regulator_get(preg, cam_device, sensor_idx,
			reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD);
#endif

	pregulator =
		preg->pregulator[sensor_idx][
			reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD];

	enable_cnt =
		&preg->enable_cnt[sensor_idx][
			reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD];

#if defined(HI1634Z6250CC142_MIPI_RAW)
	if ((pin == IMGSENSOR_HW_PIN_DVDD ) && ( sensor_idx == IMGSENSOR_SENSOR_IDX_SUB )) {

		if (pin_state == IMGSENSOR_HW_PIN_STATE_LEVEL_1200) {
			pr_info("[test] %s:set GC8034Z6250CC137 sensor DVDD to 1.25V\n", __func__);
			pmic_config_interface(MT6357_PMIC_VCAMD, 5, 0xf, 0);
		} else {
			pr_info("[test] %s:set GC8034Z6250CC137 sensor DVDD to 1.2V\n", __func__);
			pmic_config_interface(MT6357_PMIC_VCAMD, 0, 0xf, 0);
		}
	}

#endif

	if (pregulator) {
		if (pin_state != IMGSENSOR_HW_PIN_STATE_LEVEL_0) {

			if (regulator_set_voltage(
				pregulator,
				regulator_voltage[
				    pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0],
				regulator_voltage[
				 pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0])) {

				pr_err(
				    "[regulator]fail to regulator_set_voltage, powertype:%d powerId:%d\n",
				    pin,
				    regulator_voltage[
				   pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
			}
			if (regulator_enable(pregulator)) {
				pr_err(
				    "[regulator]fail to regulator_enable, powertype:%d powerId:%d\n",
				    pin,
				    regulator_voltage[
				   pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);

				return IMGSENSOR_RETURN_ERROR;
			}
			atomic_inc(enable_cnt);
		} else {
			if (regulator_is_enabled(pregulator)) {
				/*pr_debug("[regulator]%d is enabled\n", pin);*/

				if (regulator_disable(pregulator)) {
					pr_err(
					    "[regulator]fail to regulator_disable, powertype: %d\n",
					    pin);
					return IMGSENSOR_RETURN_ERROR;
				}

#if defined(HI1634Z6250CC142_MIPI_RAW)
				check_for_regulator_put(preg, sensor_idx, (reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD));
#endif
			}
			atomic_dec(enable_cnt);
		}
	} else {
		pr_err("regulator == NULL %d %d %d\n",
		    reg_type_offset,
		    pin,
		    IMGSENSOR_HW_PIN_AVDD);
	}

	return IMGSENSOR_RETURN_SUCCESS;
}

#if defined(HI1634Z6250CC142_MIPI_RAW)
static void check_for_regulator_get(struct REGULATOR *preg, struct device *pdevice,
			enum IMGSENSOR_SENSOR_IDX sensor_idx, int reg_type)
{
	struct device_node *pof_node;
	char str_regulator_name[LENGTH_FOR_SNPRINTF];

	if(reg_type == REGULATOR_TYPE_VCAMD){
		pr_info("[test] %s regulator_status[%d][%d] = %d \n",
			__func__, sensor_idx, reg_type, regulator_status[sensor_idx][reg_type]);
	}
	else{
		return;
	}
	mutex_lock(&g_regulator_state_mutex);
	if (regulator_status[sensor_idx][reg_type] == 0) {
		if (of_node_record != NULL && pdevice != NULL) {
			pof_node = pdevice->of_node;
			pdevice->of_node = of_node_record;

			snprintf(str_regulator_name,
				sizeof(str_regulator_name),
				"cam%d_%s",
				sensor_idx,
				regulator_control[reg_type].pregulator_type);
			preg->pregulator[sensor_idx][reg_type] =
			    regulator_get(pdevice, str_regulator_name);

			pdevice->of_node = pof_node;
			regulator_status[sensor_idx][reg_type] = 1;
		} else {
			pr_err("[test] %s:regulator == NULL\n", __func__);
		}
	}
	mutex_unlock(&g_regulator_state_mutex);
}

static void check_for_regulator_put(struct REGULATOR *preg,
			enum IMGSENSOR_SENSOR_IDX sensor_idx, int reg_type)
{
	if(reg_type == REGULATOR_TYPE_VCAMD){
		pr_info("[test] %s regulator_status[%d][%d] = %d \n",
			__func__, sensor_idx, reg_type, regulator_status[sensor_idx][reg_type]);
	}
	else
		return;

	mutex_lock(&g_regulator_state_mutex);
	if (regulator_status[sensor_idx][reg_type] == 1) {
		if (!IS_ERR_OR_NULL(preg->pregulator[sensor_idx][reg_type])){
			regulator_put(preg->pregulator[sensor_idx][reg_type]);
			preg->pregulator[sensor_idx][reg_type] = NULL;
		}
		regulator_status[sensor_idx][reg_type] = 0;
		pr_debug("[test] %s sensor_idx = %d, regulator_put %s\n",
			__func__, sensor_idx, regulator_control[reg_type].pregulator_type);
	}
	mutex_unlock(&g_regulator_state_mutex);

}
#endif

static struct IMGSENSOR_HW_DEVICE device = {
	.pinstance = (void *)&reg_instance,
	.init      = regulator_init,
	.set       = regulator_set,
	.release   = regulator_release,
	.id        = IMGSENSOR_HW_ID_REGULATOR
};

enum IMGSENSOR_RETURN imgsensor_hw_regulator_open(
	struct IMGSENSOR_HW_DEVICE **pdevice)
{
	*pdevice = &device;
	return IMGSENSOR_RETURN_SUCCESS;
}

