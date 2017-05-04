/*
 *  linux/drivers/mmc/core/host.c
 *
 *  Copyright (C) 2003 Russell King, All Rights Reserved.
 *  Copyright (C) 2007-2008 Pierre Ossman
 *  Copyright (C) 2010 Linus Walleij
 *  Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  MMC host class device management
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/idr.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pagemap.h>
#include <linux/export.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/pm_runtime.h>

#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/slot-gpio.h>

#include "core.h"
#include "host.h"

#define cls_dev_to_mmc_host(d)	container_of(d, struct mmc_host, class_dev)

static void mmc_host_classdev_release(struct device *dev)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	mutex_destroy(&host->slot.lock);
	if (host->mmc_circbuf.buf)
		kfree(host->mmc_circbuf.buf);
	kfree(host->wlock_name);
	kfree(host);
}

#ifdef CONFIG_PM_RUNTIME
static int mmc_host_runtime_suspend(struct device *dev)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	int ret = 0;

	if (!mmc_use_core_runtime_pm(host))
		return 0;

	ret = mmc_suspend_host(host);
	if (ret < 0 && ret != -ENOMEDIUM)
		pr_err("%s: %s: suspend host failed: %d\n", mmc_hostname(host),
		       __func__, ret);

	if (ret == -ENOMEDIUM)
		ret = 0;

	return ret;
}

static int mmc_host_runtime_resume(struct device *dev)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	int ret = 0;

	if (!mmc_use_core_runtime_pm(host))
		return 0;

	host->crc_count = 0;
	ret = mmc_resume_host(host);
	if (ret < 0) {
		pr_err("%s: %s: resume host: failed: ret: %d\n",
		       mmc_hostname(host), __func__, ret);
		if (pm_runtime_suspended(dev))
			BUG_ON(1);
	}

	return ret;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int mmc_host_suspend(struct device *dev)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	int ret = 0;
	unsigned long flags;

	if (!mmc_use_core_pm(host))
		return 0;
	pr_info("%s: %s\n", mmc_hostname(host), __func__);
	spin_lock_irqsave(&host->clk_lock, flags);
	host->dev_status = DEV_SUSPENDING;
	spin_unlock_irqrestore(&host->clk_lock, flags);
	if (!pm_runtime_suspended(dev) || mmc_is_sd_host(host)) {
		mmc_disable_clk_delay_scaling(host);
		ret = mmc_suspend_host(host);
		if (ret < 0)
			pr_err("%s: %s: failed: ret: %d\n", mmc_hostname(host),
			       __func__, ret);
	}
	if (!ret && host->card && mmc_card_sdio(host->card) &&
	    host->ios.clock) {
		spin_lock_irqsave(&host->clk_lock, flags);
		host->clk_old = host->ios.clock;
		host->ios.clock = 0;
		host->clk_gated = true;
		spin_unlock_irqrestore(&host->clk_lock, flags);
		mmc_set_ios(host);
	}
	spin_lock_irqsave(&host->clk_lock, flags);
	host->dev_status = DEV_SUSPENDED;
	spin_unlock_irqrestore(&host->clk_lock, flags);
	return ret;
}

static int mmc_host_resume(struct device *dev)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	int ret = 0;

	if (!mmc_use_core_pm(host))
		return 0;

	pr_info("%s: %s\n", mmc_hostname(host), __func__);
	if (!pm_runtime_suspended(dev)) {
		ret = mmc_resume_host(host);
		if (ret < 0)
			pr_err("%s: %s: failed: ret: %d\n", mmc_hostname(host),
			       __func__, ret);
	}
	host->dev_status = DEV_RESUMED;
	return ret;
}
#endif

static const struct dev_pm_ops mmc_host_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mmc_host_suspend, mmc_host_resume)
	SET_RUNTIME_PM_OPS(mmc_host_runtime_suspend, mmc_host_runtime_resume,
			   pm_generic_runtime_idle)
};

static struct class mmc_host_class = {
	.name		= "mmc_host",
	.dev_release	= mmc_host_classdev_release,
	.pm		= &mmc_host_pm_ops,
};

int mmc_register_host_class(void)
{
	return class_register(&mmc_host_class);
}

void mmc_unregister_host_class(void)
{
	class_unregister(&mmc_host_class);
}

static DEFINE_IDR(mmc_host_idr);
static DEFINE_SPINLOCK(mmc_host_lock);

#ifdef CONFIG_MMC_CLKGATE
static ssize_t clkgate_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	return snprintf(buf, PAGE_SIZE, "%lu\n", host->clkgate_delay);
}

static ssize_t clkgate_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	unsigned long flags, value;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	spin_lock_irqsave(&host->clk_lock, flags);
	host->clkgate_delay = value;
	spin_unlock_irqrestore(&host->clk_lock, flags);
	return count;
}

static void mmc_host_clk_gate_delayed(struct mmc_host *host)
{
	unsigned long tick_ns;
	unsigned long freq = host->ios.clock;
	unsigned long flags;

	if (!freq) {
		pr_debug("%s: frequency set to 0 in disable function, "
			 "this means the clock is already disabled.\n",
			 mmc_hostname(host));
		return;
	}
	spin_lock_irqsave(&host->clk_lock, flags);

	if (!host->clk_requests) {
		spin_unlock_irqrestore(&host->clk_lock, flags);
		tick_ns = DIV_ROUND_UP(1000000000, freq);
		ndelay(host->clk_delay * tick_ns);
	} else {
		
		spin_unlock_irqrestore(&host->clk_lock, flags);
		return;
	}
	mutex_lock(&host->clk_gate_mutex);
	spin_lock_irqsave(&host->clk_lock, flags);
	if (!host->clk_requests) {
		spin_unlock_irqrestore(&host->clk_lock, flags);
		
		mmc_gate_clock(host);
		spin_lock_irqsave(&host->clk_lock, flags);
		pr_debug("%s: gated MCI clock\n", mmc_hostname(host));
	}
	spin_unlock_irqrestore(&host->clk_lock, flags);
	mutex_unlock(&host->clk_gate_mutex);
}

static void mmc_host_clk_gate_work(struct work_struct *work)
{
	struct mmc_host *host = container_of(work, struct mmc_host,
					      clk_gate_work.work);

	mmc_host_clk_gate_delayed(host);
}

void mmc_host_clk_hold(struct mmc_host *host)
{
	unsigned long flags;

	
	cancel_delayed_work_sync(&host->clk_gate_work);
	mutex_lock(&host->clk_gate_mutex);
	spin_lock_irqsave(&host->clk_lock, flags);
	if (host->clk_gated) {
		spin_unlock_irqrestore(&host->clk_lock, flags);
		mmc_ungate_clock(host);

		
		mmc_reset_clk_scale_stats(host);
		spin_lock_irqsave(&host->clk_lock, flags);
		pr_debug("%s: ungated MCI clock\n", mmc_hostname(host));
	}
	host->clk_requests++;
	spin_unlock_irqrestore(&host->clk_lock, flags);
	mutex_unlock(&host->clk_gate_mutex);
}

bool mmc_host_may_gate_card(struct mmc_card *card)
{
	
	if (!card)
		return true;

	if (mmc_card_sdio(card) && card->cccr.async_intr_sup)
		return true;

	return !(card->quirks & MMC_QUIRK_BROKEN_CLK_GATING);
}

void mmc_host_clk_release(struct mmc_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->clk_lock, flags);
	host->clk_requests--;
	if (mmc_host_may_gate_card(host->card) &&
	    !host->clk_requests)
		schedule_delayed_work(&host->clk_gate_work,
				      msecs_to_jiffies(host->clkgate_delay));
	spin_unlock_irqrestore(&host->clk_lock, flags);
}

unsigned int mmc_host_clk_rate(struct mmc_host *host)
{
	unsigned long freq;
	unsigned long flags;

	spin_lock_irqsave(&host->clk_lock, flags);
	if (host->clk_gated)
		freq = host->clk_old;
	else
		freq = host->ios.clock;
	spin_unlock_irqrestore(&host->clk_lock, flags);
	return freq;
}

static inline void mmc_host_clk_init(struct mmc_host *host)
{
	host->clk_requests = 0;
	
	host->clk_delay = 8;
	host->clkgate_delay = 0;
	host->clk_gated = false;
	INIT_DELAYED_WORK(&host->clk_gate_work, mmc_host_clk_gate_work);
	spin_lock_init(&host->clk_lock);
	mutex_init(&host->clk_gate_mutex);
}

static inline void mmc_host_clk_exit(struct mmc_host *host)
{
	if (cancel_delayed_work_sync(&host->clk_gate_work))
		mmc_host_clk_gate_delayed(host);
	if (host->clk_gated)
		mmc_host_clk_hold(host);
	
	WARN_ON(host->clk_requests > 1);
}

static inline void mmc_host_clk_sysfs_init(struct mmc_host *host)
{
	host->clkgate_delay_attr.show = clkgate_delay_show;
	host->clkgate_delay_attr.store = clkgate_delay_store;
	sysfs_attr_init(&host->clkgate_delay_attr.attr);
	host->clkgate_delay_attr.attr.name = "clkgate_delay";
	host->clkgate_delay_attr.attr.mode = S_IRUGO | S_IWUSR;
	if (device_create_file(&host->class_dev, &host->clkgate_delay_attr))
		pr_err("%s: Failed to create clkgate_delay sysfs entry\n",
				mmc_hostname(host));
}
#else

static inline void mmc_host_clk_init(struct mmc_host *host)
{
}

static inline void mmc_host_clk_exit(struct mmc_host *host)
{
}

static inline void mmc_host_clk_sysfs_init(struct mmc_host *host)
{
}

#endif

void mmc_of_parse(struct mmc_host *host)
{
	struct device_node *np;
	u32 bus_width;
	bool explicit_inv_wp, gpio_inv_wp = false;
	enum of_gpio_flags flags;
	int len, ret, gpio;

	if (!host->parent || !host->parent->of_node)
		return;

	np = host->parent->of_node;

	
	if (of_property_read_u32(np, "bus-width", &bus_width) < 0) {
		dev_dbg(host->parent,
			"\"bus-width\" property is missing, assuming 1 bit.\n");
		bus_width = 1;
	}

	switch (bus_width) {
	case 8:
		host->caps |= MMC_CAP_8_BIT_DATA;
		
	case 4:
		host->caps |= MMC_CAP_4_BIT_DATA;
		break;
	case 1:
		break;
	default:
		dev_err(host->parent,
			"Invalid \"bus-width\" value %ud!\n", bus_width);
	}

	
	of_property_read_u32(np, "max-frequency", &host->f_max);


	
	if (of_find_property(np, "non-removable", &len)) {
		host->caps |= MMC_CAP_NONREMOVABLE;
	} else {
		bool explicit_inv_cd, gpio_inv_cd = false;

		explicit_inv_cd = of_property_read_bool(np, "cd-inverted");

		if (of_find_property(np, "broken-cd", &len))
			host->caps |= MMC_CAP_NEEDS_POLL;

		gpio = of_get_named_gpio_flags(np, "cd-gpios", 0, &flags);
		if (gpio_is_valid(gpio)) {
			if (!(flags & OF_GPIO_ACTIVE_LOW))
				gpio_inv_cd = true;

			ret = mmc_gpio_request_cd(host, gpio);
			if (ret < 0)
				dev_err(host->parent,
					"Failed to request CD GPIO #%d: %d!\n",
					gpio, ret);
			else
				dev_info(host->parent, "Got CD GPIO #%d.\n",
					 gpio);
		}

		if (explicit_inv_cd ^ gpio_inv_cd)
			host->caps2 |= MMC_CAP2_CD_ACTIVE_HIGH;
	}

	
	explicit_inv_wp = of_property_read_bool(np, "wp-inverted");

	gpio = of_get_named_gpio_flags(np, "wp-gpios", 0, &flags);
	if (gpio_is_valid(gpio)) {
		if (!(flags & OF_GPIO_ACTIVE_LOW))
			gpio_inv_wp = true;

		ret = mmc_gpio_request_ro(host, gpio);
		if (ret < 0)
			dev_err(host->parent,
				"Failed to request WP GPIO: %d!\n", ret);
	}
	if (explicit_inv_wp ^ gpio_inv_wp)
		host->caps2 |= MMC_CAP2_RO_ACTIVE_HIGH;

	if (of_find_property(np, "cap-sd-highspeed", &len))
		host->caps |= MMC_CAP_SD_HIGHSPEED;
	if (of_find_property(np, "cap-mmc-highspeed", &len))
		host->caps |= MMC_CAP_MMC_HIGHSPEED;
	if (of_find_property(np, "cap-power-off-card", &len))
		host->caps |= MMC_CAP_POWER_OFF_CARD;
	if (of_find_property(np, "cap-sdio-irq", &len))
		host->caps |= MMC_CAP_SDIO_IRQ;
	if (of_find_property(np, "keep-power-in-suspend", &len))
		host->pm_caps |= MMC_PM_KEEP_POWER;
	if (of_find_property(np, "enable-sdio-wakeup", &len))
		host->pm_caps |= MMC_PM_WAKE_SDIO_IRQ;
}

EXPORT_SYMBOL(mmc_of_parse);

struct mmc_host *mmc_alloc_host(int extra, struct device *dev)
{
	int err;
	struct mmc_host *host;

	host = kzalloc(sizeof(struct mmc_host) + extra, GFP_KERNEL);
	if (!host)
		return NULL;

	
	host->rescan_disable = 1;
	idr_preload(GFP_KERNEL);
	spin_lock(&mmc_host_lock);
	err = idr_alloc(&mmc_host_idr, host, 0, 0, GFP_NOWAIT);
	if (err >= 0)
		host->index = err;
	spin_unlock(&mmc_host_lock);
	idr_preload_end();
	if (err < 0)
		goto free;

	dev_set_name(&host->class_dev, "mmc%d", host->index);

	host->parent = dev;
	host->class_dev.parent = dev;
	host->class_dev.class = &mmc_host_class;
	device_initialize(&host->class_dev);

	mmc_host_clk_init(host);

	mutex_init(&host->slot.lock);
	host->slot.cd_irq = -EINVAL;

	spin_lock_init(&host->lock);
	init_waitqueue_head(&host->wq);
	host->wlock_name = kasprintf(GFP_KERNEL,
			"%s_detect", mmc_hostname(host));
	wake_lock_init(&host->detect_wake_lock, WAKE_LOCK_SUSPEND,
			host->wlock_name);
	INIT_DELAYED_WORK(&host->detect, mmc_rescan);
	INIT_DELAYED_WORK(&host->debounce, mmc_debounce2);
	INIT_DELAYED_WORK(&host->stats_work, mmc_stats);
#ifdef CONFIG_PM
	host->pm_notify.notifier_call = mmc_pm_notify;
#endif

	host->max_segs = 1;
	host->max_seg_size = PAGE_CACHE_SIZE;

	host->max_req_size = PAGE_CACHE_SIZE;
	host->max_blk_size = 512;
	host->max_blk_count = PAGE_CACHE_SIZE / 512;

	return host;

free:
	kfree(host);
	return NULL;
}

EXPORT_SYMBOL(mmc_alloc_host);

static ssize_t show_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);

	if (!host)
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "%d\n", mmc_can_scale_clk(host));
}

static ssize_t store_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	unsigned long value, freq;
	int retval = -EINVAL;

	if (!host)
		goto out;

	
	if (host->card)
		mmc_rpm_hold(host, &host->card->dev);

	mmc_claim_host(host);
	if (!host->card || kstrtoul(buf, 0, &value))
		goto err;

	if (value && !mmc_can_scale_clk(host)) {
		host->caps2 |= MMC_CAP2_CLK_SCALE;
		mmc_init_clk_scaling(host);

		if (!mmc_can_scale_clk(host)) {
			host->caps2 &= ~MMC_CAP2_CLK_SCALE;
			goto err;
		}
	} else if (!value && mmc_can_scale_clk(host)) {
		host->caps2 &= ~MMC_CAP2_CLK_SCALE;
		mmc_disable_clk_scaling(host);

		
		if (host->bus_ops && host->bus_ops->change_bus_speed &&
				host->clk_scaling.state == MMC_LOAD_LOW) {
			freq = mmc_get_max_frequency(host);
			if (host->bus_ops->change_bus_speed(host, &freq))
				goto err;
		}
		if (host->ops->notify_load &&
				host->ops->notify_load(host, MMC_LOAD_HIGH))
			goto err;
		host->clk_scaling.state = MMC_LOAD_HIGH;
		host->clk_scaling.initialized = false;
	}
	retval = count;
err:
	mmc_release_host(host);

	
	if (host->card)
		mmc_rpm_release(host, &host->card->dev);
out:
	return retval;
}

static ssize_t show_up_threshold(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);

	if (!host)
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "%d\n", host->clk_scaling.up_threshold);
}

#define MAX_PERCENTAGE	100
static ssize_t store_up_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	unsigned long value;

	if (!host || kstrtoul(buf, 0, &value) || (value > MAX_PERCENTAGE))
		return -EINVAL;

	host->clk_scaling.up_threshold = value;

	pr_debug("%s: clkscale_up_thresh set to %lu\n",
			mmc_hostname(host), value);
	return count;
}

static ssize_t show_down_threshold(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);

	if (!host)
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			host->clk_scaling.down_threshold);
}

static ssize_t store_down_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	unsigned long value;

	if (!host || kstrtoul(buf, 0, &value) || (value > MAX_PERCENTAGE))
		return -EINVAL;

	host->clk_scaling.down_threshold = value;

	pr_debug("%s: clkscale_down_thresh set to %lu\n",
			mmc_hostname(host), value);
	return count;
}

static ssize_t show_polling(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);

	if (!host)
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "%lu milliseconds\n",
			host->clk_scaling.polling_delay_ms);
}

static ssize_t store_polling(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	unsigned long value;

	if (!host || kstrtoul(buf, 0, &value))
		return -EINVAL;

	host->clk_scaling.polling_delay_ms = value;

	pr_debug("%s: clkscale_polling_delay_ms set to %lu\n",
			mmc_hostname(host), value);
	return count;
}

DEVICE_ATTR(enable, S_IRUGO | S_IWUSR,
		show_enable, store_enable);
DEVICE_ATTR(polling_interval, S_IRUGO | S_IWUSR,
		show_polling, store_polling);
DEVICE_ATTR(up_threshold, S_IRUGO | S_IWUSR,
		show_up_threshold, store_up_threshold);
DEVICE_ATTR(down_threshold, S_IRUGO | S_IWUSR,
		show_down_threshold, store_down_threshold);

static struct attribute *clk_scaling_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_up_threshold.attr,
	&dev_attr_down_threshold.attr,
	&dev_attr_polling_interval.attr,
	NULL,
};

static struct attribute_group clk_scaling_attr_grp = {
	.name = "clk_scaling",
	.attrs = clk_scaling_attrs,
};

static ssize_t
show_perf(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	int64_t rtime_drv, wtime_drv;
	unsigned long rbytes_drv, wbytes_drv;

	spin_lock(&host->lock);

	rbytes_drv = host->perf.rbytes_drv;
	wbytes_drv = host->perf.wbytes_drv;

	rtime_drv = ktime_to_us(host->perf.rtime_drv);
	wtime_drv = ktime_to_us(host->perf.wtime_drv);

	spin_unlock(&host->lock);

	return snprintf(buf, PAGE_SIZE, "Write performance at driver Level:"
					"%lu bytes in %lld microseconds\n"
					"Read performance at driver Level:"
					"%lu bytes in %lld microseconds\n",
					wbytes_drv, wtime_drv,
					rbytes_drv, rtime_drv);
}

static ssize_t
set_perf(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	int64_t value;

	sscanf(buf, "%lld", &value);
	spin_lock(&host->lock);
	if (!value) {
		memset(&host->perf, 0, sizeof(host->perf));
		host->perf_enable = false;
	} else {
		host->perf_enable = true;
	}
	spin_unlock(&host->lock);

	return count;
}

static DEVICE_ATTR(perf, S_IRUGO | S_IWUSR,
		show_perf, set_perf);

static ssize_t
show_debug(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	if (!host)
		return 0;
	return sprintf(buf, "%d", host->debug_flag);
}

static ssize_t
set_debug(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int value;
	struct mmc_host *host = cls_dev_to_mmc_host(dev);

	sscanf(buf, "%d", &value);
	host->debug_flag = value;
	pr_info("%s: set debug flag 0x%x\n", mmc_hostname(host), value);

	return count;
}
static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR,
		show_debug, set_debug);


static struct attribute *dev_attrs[] = {
	&dev_attr_perf.attr,
	&dev_attr_debug.attr,
	NULL,
};
static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};

static int mmc_proc_acc_perf_read(struct seq_file *m, void *v)
{
	struct mmc_host *host = (struct mmc_host*) m->private;

	int head;
	u64 val;
	unsigned long rbytes, rtime, wbytes, wtime;
	unsigned long total_wbytes, total_rbytes, total_wtime, total_rtime;
	unsigned int rperf = 0, wperf = 0, acc_time;
	unsigned long flags;
	char user_buf[128];
	ssize_t count = 0;
	int cnt;

	if (!host || (host && !host->circbuf_enable))
		return 0;

	head = host->mmc_circbuf.head;
	seq_printf(m, "time interval (min):   ~1  ");
	for (cnt = 2; cnt < 10; cnt ++)
		seq_printf(m, "   %d~%d  ", cnt - 1, cnt);
	seq_printf(m, "   9~10  \n");
	seq_printf(m, "%s read (KB/s)   :", mmc_hostname(host));

	memset(user_buf, 128, 0);
	cnt = snprintf(user_buf, sizeof(user_buf), "%s write (KB/s)  :", mmc_hostname(host));

	total_rbytes = total_wbytes = total_wtime = total_rtime = 0;
	acc_time = 0;

	spin_lock_irqsave(&host->lock, flags);
	while (head != host->mmc_circbuf.tail) {
		if (head == 0)
			head = CIRC_BUFFER_SIZE;

		if (head < sizeof(rbytes) + sizeof(wbytes) + sizeof(rtime) + sizeof(wtime))
			break;

		head = head - sizeof(wtime);
		memcpy(&wtime, host->mmc_circbuf.buf + head, sizeof(wtime));
		head = head - sizeof(wbytes);
		memcpy(&wbytes, host->mmc_circbuf.buf + head, sizeof(wbytes));
		head = head - sizeof(rtime);
		memcpy(&rtime, host->mmc_circbuf.buf + head, sizeof(rtime));
		head = head - sizeof(rbytes);
		memcpy(&rbytes, host->mmc_circbuf.buf + head, sizeof(rbytes));

		total_wbytes += wbytes;
		total_rbytes += rbytes;
		total_wtime += wtime;
		total_rtime += rtime;
		acc_time += MMC_STATS_INTERVAL;
		
		if (!(acc_time % (60 * 1000)) || (head == host->mmc_circbuf.tail)) {
			if (total_wtime) {
				val = ((u64)total_wbytes / 1024) * 1000000;
				do_div(val, total_wtime);
				wperf = (unsigned int)val;
			}

			if (total_rtime) {
				val = ((u64)total_rbytes / 1024) * 1000000;
				do_div(val, total_rtime);
				rperf = (unsigned int)val;
			}
			seq_printf(m, "%6u, ", rperf);
			cnt += snprintf(&user_buf[cnt], sizeof(user_buf) - cnt, "%6u, ",
					wperf);
			total_rbytes = total_wbytes = total_wtime = total_rtime = 0;
			wperf = rperf = 0;
		}

		if (acc_time >= 10 * 60 * 1000)
			break;
		if ((count + cnt) >= PAGE_SIZE || cnt >= sizeof(user_buf))
			break;
	}
	spin_unlock_irqrestore(&host->lock, flags);

	seq_printf(m, "\n%s\n", user_buf);
	return 0;
}

static int mmc_proc_acc_perf_open(struct inode *inode, struct file *file)
{
	return single_open(file, mmc_proc_acc_perf_read, PDE_DATA(inode));
}

static const struct file_operations mmc_proc_acc_perf_fops = {
	.open           = mmc_proc_acc_perf_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

int mmc_add_host(struct mmc_host *host)
{
	int err;

	WARN_ON((host->caps & MMC_CAP_SDIO_IRQ) &&
		!host->ops->enable_sdio_irq);

	err = pm_runtime_set_active(&host->class_dev);
	if (err)
		pr_err("%s: %s: failed setting runtime active: err: %d\n",
		       mmc_hostname(host), __func__, err);
	else if (mmc_use_core_runtime_pm(host))
		pm_runtime_enable(&host->class_dev);

	err = device_add(&host->class_dev);
	if (err)
		return err;

	device_enable_async_suspend(&host->class_dev);
	led_trigger_register_simple(dev_name(&host->class_dev), &host->led);

#ifdef CONFIG_DEBUG_FS
	mmc_add_host_debugfs(host);
#endif
	mmc_host_clk_sysfs_init(host);

	host->clk_scaling.up_threshold = 35;
	host->clk_scaling.down_threshold = 5;
	host->clk_scaling.polling_delay_ms = 100;

	host->circbuf_enable = false;
	host->mmc_circbuf.buf = kzalloc(CIRC_BUFFER_SIZE, GFP_KERNEL);
	if (!host->mmc_circbuf.buf) {
		pr_err("%s: failed to allocate memory for circular buffer.\n",
				mmc_hostname(host));
	} else {
		host->mmc_circbuf.head = host->mmc_circbuf.tail = 0;
		if (mmc_is_mmc_host(host)) {
			host->mmc_circ_proc = proc_create_data("emmc_acc_perf", 0444, NULL,
					&mmc_proc_acc_perf_fops, host);
		} else if (mmc_is_sd_host(host)) {
			host->mmc_circ_proc = proc_create_data("sd_acc_perf", 0444, NULL,
					&mmc_proc_acc_perf_fops, host);
		}
	}

	err = sysfs_create_group(&host->class_dev.kobj, &clk_scaling_attr_grp);
	if (err)
		pr_err("%s: failed to create clk scale sysfs group with err %d\n",
				__func__, err);

	err = sysfs_create_group(&host->class_dev.kobj, &dev_attr_grp);
	if (err)
		pr_err("%s: failed to create sysfs group with err %d\n",
							 __func__, err);

	mmc_start_host(host);
	if (!(host->pm_flags & MMC_PM_IGNORE_PM_NOTIFY))
		register_pm_notifier(&host->pm_notify);
	return 0;
}

EXPORT_SYMBOL(mmc_add_host);

void mmc_remove_host(struct mmc_host *host)
{
	if (!(host->pm_flags & MMC_PM_IGNORE_PM_NOTIFY))
		unregister_pm_notifier(&host->pm_notify);
	mmc_stop_host(host);

#ifdef CONFIG_DEBUG_FS
	mmc_remove_host_debugfs(host);
#endif
	if (mmc_is_sd_host(host))
		remove_proc_entry("sd_acc_perf", NULL);
	else if (mmc_is_mmc_host(host))
		remove_proc_entry("emmc_acc_perf", NULL);
	sysfs_remove_group(&host->parent->kobj, &dev_attr_grp);
	sysfs_remove_group(&host->class_dev.kobj, &clk_scaling_attr_grp);

	device_del(&host->class_dev);

	led_trigger_unregister_simple(host->led);

	mmc_host_clk_exit(host);
}

EXPORT_SYMBOL(mmc_remove_host);

void mmc_free_host(struct mmc_host *host)
{
	spin_lock(&mmc_host_lock);
	idr_remove(&mmc_host_idr, host->index);
	spin_unlock(&mmc_host_lock);
	wake_lock_destroy(&host->detect_wake_lock);

	put_device(&host->class_dev);
}

EXPORT_SYMBOL(mmc_free_host);
