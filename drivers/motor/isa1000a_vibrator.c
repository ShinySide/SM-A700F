/* drivers/motor/isa1000a_vibrator.c

 * Copyright (C) 2014 Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/timed_output.h>
#include <linux/hrtimer.h>
#include <linux/pwm.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/isa1000a_vibrator.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/sec_sysfs.h>

struct isa1000a_vibrator_data {
    struct device* dev;
	struct isa1000a_vibrator_platform_data *pdata;
	struct pwm_device *pwm;
	struct regulator *regulator;
	struct timed_output_dev tout_dev;
	struct hrtimer timer;
	unsigned int timeout;
	struct work_struct work;
	spinlock_t lock;
	bool running;
};

static struct device *motor_dev;
struct isa1000a_vibrator_data *g_hap_data;
static int prev_duty;

static int haptic_get_time(struct timed_output_dev *tout_dev)
{
	struct isa1000a_vibrator_data *hap_data
		= container_of(tout_dev, struct  isa1000a_vibrator_data, tout_dev);

	struct hrtimer *timer = &hap_data->timer;
	if (hrtimer_active(timer)) {
		ktime_t remain = hrtimer_get_remaining(timer);
		struct timeval t = ktime_to_timeval(remain);
		return t.tv_sec * 1000 + t.tv_usec / 1000;
	} else
		return 0;
}

static void haptic_enable(struct timed_output_dev *tout_dev, int value)
{
	struct isa1000a_vibrator_data *hap_data
		= container_of(tout_dev, struct isa1000a_vibrator_data, tout_dev);

	struct hrtimer *timer = &hap_data->timer;
	unsigned long flags;


	cancel_work_sync(&hap_data->work);
	hrtimer_cancel(timer);
	hap_data->timeout = value;
	schedule_work(&hap_data->work);
	spin_lock_irqsave(&hap_data->lock, flags);
	if (value > 0 ) {
		pr_info("[VIB] %s : value %d\n", __func__, value);
		if (value > hap_data->pdata->max_timeout)
			value = hap_data->pdata->max_timeout;
		hrtimer_start(timer, ns_to_ktime((u64)value * NSEC_PER_MSEC),
			HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&hap_data->lock, flags);
}

static enum hrtimer_restart haptic_timer_func(struct hrtimer *timer)
{
	struct isa1000a_vibrator_data *hap_data
		= container_of(timer, struct isa1000a_vibrator_data, timer);

	hap_data->timeout = 0;

	schedule_work(&hap_data->work);
	return HRTIMER_NORESTART;
}

static int vibetonz_clk_on(struct device *dev, bool en)
{
	struct clk *vibetonz_clk = NULL;
#if defined(CONFIG_OF)
	struct device_node *np;
	np = of_find_node_by_name(NULL,"pwm");
	if (np == NULL) {
		pr_err("[VIB] %s : pwm error to get dt node\n", __func__);
		return -EINVAL;
	}
	vibetonz_clk = of_clk_get_by_name(np, "gate_timers");
	if (!vibetonz_clk) {
		pr_err("[VIB] %s : fail to get the vibetonz_clk\n", __func__);
		return -EINVAL;
	}
#else
	vibetonz_clk = clk_get(dev, "timers");
#endif
	pr_info("[VIB] %s : DEV NAME %s %lu\n", __func__,
		 dev_name(dev), clk_get_rate(vibetonz_clk));

	if (IS_ERR(vibetonz_clk)) {
		pr_err("[VIB] %s : failed to get clock for the motor\n", __func__);
		goto err_clk_get;
	}

	if (en)
		clk_enable(vibetonz_clk);
	else
		clk_disable(vibetonz_clk);

	clk_put(vibetonz_clk);
	return 0;

err_clk_get:
	clk_put(vibetonz_clk);
	return -EINVAL;
}

static void haptic_work(struct work_struct *work)
{
	struct isa1000a_vibrator_data *hap_data
		= container_of(work, struct isa1000a_vibrator_data, work);
	int ret;
	if (hap_data->timeout == 0) {
		if (!hap_data->running)
			return;
		hap_data->running = false;
#ifdef CONFIG_MOTOR_DRV_ISA1000A_WITH_MOTOR_EN
		gpio_set_value(hap_data->pdata->en, 0);
#endif
		regulator_disable(hap_data->regulator);
		pwm_disable(hap_data->pwm);
	} else {
		if (hap_data->running)
			return;
		pwm_config(hap_data->pwm, hap_data->pdata->duty,
			   hap_data->pdata->period);
		pwm_enable(hap_data->pwm);
		ret = regulator_enable(hap_data->regulator);
		if (ret)
			goto error_reg_enable;
#ifdef CONFIG_MOTOR_DRV_ISA1000A_WITH_MOTOR_EN
		gpio_set_value(hap_data->pdata->en, 1);
#endif
		hap_data->running = true;
	}
	return;
error_reg_enable :
	pr_err("[VIB] %s : Failed to enable vdd.\n", __func__);
}

#ifdef CONFIG_VIBETONZ
void vibtonz_en(bool en)
{
	int ret;

	pr_info("[VIB] %s %s\n", __func__, en ? "on" : "off");

	if (g_hap_data == NULL) {
		pr_err("[VIB] %s : the motor is not ready!!!", __func__);
		return ;
	}
	if (en) {
		if (g_hap_data->running)
			return;

		pwm_config(g_hap_data->pwm, prev_duty, g_hap_data->pdata->period);
		pwm_enable(g_hap_data->pwm);
		ret = regulator_enable(g_hap_data->regulator);
		if (ret)
			goto error_reg_enable;
#ifdef CONFIG_MOTOR_DRV_ISA1000A_WITH_MOTOR_EN
		gpio_set_value(g_hap_data->pdata->en, 1);
#endif
		g_hap_data->running = true;
	} else {
		if (!g_hap_data->running)
			return;
#ifdef CONFIG_MOTOR_DRV_ISA1000A_WITH_MOTOR_EN
		gpio_set_value(g_hap_data->pdata->en, 0);
#endif
		regulator_disable(g_hap_data->regulator);
		pwm_disable(g_hap_data->pwm);
		g_hap_data->running = false;
	}
	return;
error_reg_enable :
	pr_err("[VIB] %s: Failed to enable vdd.\n", __func__);
}
EXPORT_SYMBOL(vibtonz_en);

void vibtonz_pwm(int nForce)
{
	int pwm_period = 0, pwm_duty = 0;

	if (g_hap_data == NULL) {
		pr_err("[VIB] %s : the motor is not ready!!!", __func__);
		return ;
	}

	pwm_period = g_hap_data->pdata->period;
	pwm_duty = pwm_period / 2 + ((pwm_period / 2 - 2) * nForce) / 127;

	if (pwm_duty > g_hap_data->pdata->duty)
		pwm_duty = g_hap_data->pdata->duty;
	else if (pwm_period - pwm_duty > g_hap_data->pdata->duty)
		pwm_duty = pwm_period - g_hap_data->pdata->duty;

	/* add to avoid the glitch issue */
	if (prev_duty != pwm_duty) {
		prev_duty = pwm_duty;
		pwm_config(g_hap_data->pwm, pwm_duty, pwm_period);
	}
}
EXPORT_SYMBOL(vibtonz_pwm);
#endif

#if defined(CONFIG_OF)
static int of_isa1000a_vibrator_dt(struct isa1000a_vibrator_platform_data *pdata)
{
	struct device_node *np_haptic;
	int temp;
	const char *temp_str;
	int ret;

	pr_info("[VIB] ++ %s\n", __func__);

	np_haptic = of_find_node_by_path("/haptic");
	if (np_haptic == NULL) {
		pr_err("[VIB] %s : error to get dt node\n", __func__);
		goto err_parsing_dt;
	}

	ret = of_property_read_u32(np_haptic, "haptic,max_timeout", &temp);
	if (IS_ERR_VALUE(ret)) {
		pr_err("[VIB] %s : error to get dt node max_timeout\n", __func__);
		goto err_parsing_dt;
	}
	pdata->max_timeout = temp;

	ret = of_property_read_u32(np_haptic, "haptic,duty", &temp);
	if (IS_ERR_VALUE(ret)) {
		pr_err("[VIB] %s : error to get dt node duty\n", __func__);
		goto err_parsing_dt;
	}
	pdata->duty = temp;

	ret = of_property_read_u32(np_haptic, "haptic,period", &temp);
	if (IS_ERR_VALUE(ret)) {
		pr_err("[VIB] %s : error to get dt node period\n", __func__);
		goto err_parsing_dt;
	}
	pdata->period = temp;

	ret = of_property_read_u32(np_haptic, "haptic,pwm_id", &temp);
	if (IS_ERR_VALUE(ret)) {
		pr_err("[VIB] %s : error to get dt node pwm_id\n", __func__);
		goto err_parsing_dt;
	}
	pdata->pwm_id = temp;

	ret = of_property_read_string(np_haptic, "haptic,regulator_name", &temp_str);
	if (IS_ERR_VALUE(ret)) {
		pr_err("[VIB] %s : error to get dt node regulator_name\n", __func__);
		goto err_parsing_dt;
	}
	pdata->regulator_name = (char *)temp_str;

#ifdef CONFIG_MOTOR_DRV_ISA1000A_WITH_MOTOR_EN
	ret = of_get_named_gpio(np_haptic, "haptic,en", 0);
	if (IS_ERR_VALUE(ret)) {
		pr_err("[VIB] %s : error to get dt node en\n", __func__);
		goto err_parsing_dt;
	}

	pdata->en = (unsigned int)ret;
#endif

	/* debugging */
	pr_info("[VIB] %s : max_timeout = %d\n", __func__, pdata->max_timeout);
	pr_info("[VIB] %s : duty = %d\n", __func__, pdata->duty);
	pr_info("[VIB] %s : period = %d\n", __func__, pdata->period);
	pr_info("[VIB] %s : pwm_id = %d\n", __func__, pdata->pwm_id);
	pr_info("[VIB] %s : regulator_name = %s\n", __func__, pdata->regulator_name);

	return 0;

err_parsing_dt:

	return -EINVAL;
}
#endif /* CONFIG_OF */

static ssize_t store_duty(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	char buff[10] = {0,};
	int cnt, ret;
	u16 duty;

	cnt = count;
	cnt = (buf[cnt-1] == '\n') ? cnt-1 : cnt;
	memcpy(buff, buf, cnt);
	buff[cnt] = '\0';

	ret = kstrtou16(buff, 0, &duty);
	if (ret != 0) {
		dev_err(dev, "[VIB] fail to get duty.\n");
		return count;
	}
	g_hap_data->pdata->duty = (u16)duty;

	return count;
}

static ssize_t store_period(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	char buff[10] = {0,};
	int cnt, ret;
	u16 period;

	cnt = count;
	cnt = (buf[cnt-1] == '\n') ? cnt-1 : cnt;
	memcpy(buff, buf, cnt);
	buff[cnt] = '\0';

	ret = kstrtou16(buff, 0, &period);
	if (ret != 0) {
		dev_err(dev, "[VIB] fail to get period.\n");
		return count;
	}
	g_hap_data->pdata->period = (u16)period;

	return count;
}

static ssize_t show_duty(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", g_hap_data->pdata->duty);
}

static ssize_t show_period(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", g_hap_data->pdata->period);
}

/* below nodes is SAMSUNG specific nodes */
static DEVICE_ATTR(duty, 0660, show_duty, store_duty);
static DEVICE_ATTR(period, 0660, show_period, store_period);

static struct attribute *sec_motor_attributes[] = {
	&dev_attr_duty.attr,
	&dev_attr_period.attr,
	NULL,
};

static struct attribute_group sec_motor_attr_group = {
	.attrs = sec_motor_attributes,
};

static int isa1000a_vibrator_probe(struct platform_device *pdev)
{
	int ret, error = 0;
#if !defined(CONFIG_OF)
	struct isa1000a_vibrator_platform_data *isa1000a_pdata
		= dev_get_platdata(&pdev->dev);
#endif
	struct isa1000a_vibrator_data *hap_data;

	pr_info("[VIB] ++ %s\n", __func__);

	hap_data = kzalloc(sizeof(struct isa1000a_vibrator_data), GFP_KERNEL);
	if (!hap_data)
		return -ENOMEM;

#if defined(CONFIG_OF)
	hap_data->pdata = kzalloc(sizeof(struct isa1000a_vibrator_data), GFP_KERNEL);
	if (!hap_data->pdata) {
		kfree(hap_data);
		return -ENOMEM;
	}

	ret = of_isa1000a_vibrator_dt(hap_data->pdata);
	if (ret < 0) {
		pr_info("[VIB] %s : not found haptic dt! ret[%d]\n",
				 __func__, ret);
		kfree(hap_data->pdata);
		kfree(hap_data);
		return ret;
	}
#else
	hap_data->pdata = isa1000a_pdata;
	if (hap_data->pdata == NULL) {
		pr_err("[VIB] %s : no pdata\n", __func__);
		kfree(hap_data);
		return -ENODEV;
	}
#endif /* CONFIG_OF */

#ifdef CONFIG_MOTOR_DRV_ISA1000A_WITH_MOTOR_EN
	ret = devm_gpio_request_one(&pdev->dev, hap_data->pdata->en,
					GPIOF_OUT_INIT_LOW, "motor_en");
	if (ret) {
		pr_err("[VIB] %s : motor_en gpio request error\n", __func__);
		kfree(hap_data->pdata);
		kfree(hap_data);
		return ret;
	}
#endif

	platform_set_drvdata(pdev, hap_data);
	g_hap_data = hap_data;
	hap_data->dev = &pdev->dev;
	INIT_WORK(&(hap_data->work), haptic_work);
	spin_lock_init(&(hap_data->lock));
	hap_data->pwm = pwm_request(hap_data->pdata->pwm_id, "vibrator");
	if (IS_ERR(hap_data->pwm)) {
		pr_err("[VIB] %s : Failed to request pwm\n", __func__);
		error = -EFAULT;
		goto err_pwm_request;
	}

	pwm_config(hap_data->pwm, hap_data->pdata->period / 2, hap_data->pdata->period);
	prev_duty = hap_data->pdata->period / 2;

	vibetonz_clk_on(&pdev->dev, true);

	hap_data->regulator
			= regulator_get(NULL, hap_data->pdata->regulator_name);

	if (IS_ERR(hap_data->regulator)) {
		pr_err("[VIB] %s : Failed to get vmoter regulator.\n", __func__);
		error = -EFAULT;
		goto err_regulator_get;
	}
	/* hrtimer init */
	hrtimer_init(&hap_data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hap_data->timer.function = haptic_timer_func;

	/* timed_output_dev init*/
	hap_data->tout_dev.name = "vibrator";
	hap_data->tout_dev.get_time = haptic_get_time;
	hap_data->tout_dev.enable = haptic_enable;

	motor_dev = sec_device_create(hap_data, "motor");
	if (IS_ERR(motor_dev)) {
		error = -ENODEV;
		pr_err("[VIB] %s : Failed to create device\
				for samsung specific motor, err num: %d\n", __func__, error);
		goto exit_sec_devices;
	}

	error = sysfs_create_group(&motor_dev->kobj, &sec_motor_attr_group);
	if (error) {
		error = -ENODEV;
		pr_err("[VIB] %s : Failed to create sysfs group\
				for samsung specific motor, err num: %d\n", __func__, error);
		goto exit_sysfs;
	}

	error = timed_output_dev_register(&hap_data->tout_dev);
	if (error < 0) {
		pr_err("[VIB] %s : Failed to register timed_output : %d\n", __func__, error);
		error = -EFAULT;
		goto err_timed_output_register;
	}

	pr_info("[VIB] -- %s\n", __func__);

	return error;

err_timed_output_register:
	sysfs_remove_group(&motor_dev->kobj, &sec_motor_attr_group);
exit_sysfs:
	sec_device_destroy(motor_dev->devt);
exit_sec_devices:
	regulator_put(hap_data->regulator);
err_regulator_get:
	pwm_free(hap_data->pwm);
err_pwm_request:
	kfree(hap_data->pdata);
	kfree(hap_data);
	g_hap_data = NULL;
	return error;
}

static int __devexit isa1000a_vibrator_remove(struct platform_device *pdev)
{
	struct isa1000a_vibrator_data *data = platform_get_drvdata(pdev);

	timed_output_dev_unregister(&data->tout_dev);
	regulator_put(data->regulator);
	pwm_free(data->pwm);
	kfree(data->pdata);
	kfree(data);
	g_hap_data = NULL;
	return 0;
}
#if defined(CONFIG_OF)
static struct of_device_id haptic_dt_ids[] = {
	{ .compatible = "isa1000a-vibrator" },
	{ },
};
MODULE_DEVICE_TABLE(of, haptic_dt_ids);
#endif /* CONFIG_OF */
static int isa1000a_vibrator_suspend(struct platform_device *pdev,
			pm_message_t state)
{
	pr_info("[VIB] %s\n", __func__);
	if (g_hap_data != NULL) {
		cancel_work_sync(&g_hap_data->work);
		hrtimer_cancel(&g_hap_data->timer);
	}
	vibetonz_clk_on(&pdev->dev, false);
	return 0;
}

static int isa1000a_vibrator_resume(struct platform_device *pdev)
{
	pr_info("[VIB] %s\n", __func__);
	vibetonz_clk_on(&pdev->dev, true);
	return 0;
}

static struct platform_driver isa1000a_vibrator_driver = {
	.probe		= isa1000a_vibrator_probe,
	.remove		= isa1000a_vibrator_remove,
	.suspend	= isa1000a_vibrator_suspend,
	.resume		= isa1000a_vibrator_resume,
	.driver = {
		.name	= "isa1000a-vibrator",
		.owner	= THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table	= haptic_dt_ids,
#endif /* CONFIG_OF */
	},
};

static int __init isa1000a_vibrator_init(void)
{
	pr_debug("[VIB] %s\n", __func__);
	return platform_driver_register(&isa1000a_vibrator_driver);
}
module_init(isa1000a_vibrator_init);

static void __exit isa1000a_vibrator_exit(void)
{
	platform_driver_unregister(&isa1000a_vibrator_driver);
}
module_exit(isa1000a_vibrator_exit);

MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ISA1000A motor driver");
