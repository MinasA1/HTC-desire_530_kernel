/*
 * Copyright (C) 2015 HTC, Inc.
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/seq_file.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/unistd.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/of.h>

#define DEVICE_TREE_MISC_PATH "/chosen/misc"
#define FUSE_ROW_WIDTH_BYTE 8

#define PROC_ANTI_ROLLBACK_LSB_1 "driver/anti_rollback_lsb_1"
#define PROC_ANTI_ROLLBACK_MSB_1 "driver/anti_rollback_msb_1"
#define PROC_ANTI_ROLLBACK_LSB_2 "driver/anti_rollback_lsb_2"
#define PROC_ANTI_ROLLBACK_MSB_2 "driver/anti_rollback_msb_2"

#define LABEL_ANTI_ROLLBACK_1_LSB "QFPROM_CORR_ANTI_ROLLBACK_1_LSB"
#define LABEL_ANTI_ROLLBACK_1_MSB "QFPROM_CORR_ANTI_ROLLBACK_1_MSB"
#define LABEL_ANTI_ROLLBACK_2_LSB "QFPROM_CORR_ANTI_ROLLBACK_2_LSB"
#define LABEL_ANTI_ROLLBACK_2_MSB "QFPROM_CORR_ANTI_ROLLBACK_2_MSB"

#if 0
#define SECMSG(s...) pr_info("[SECURITY] "s)
#else
#define SECMSG(s...) do{} while(0)
#endif

static char htc_anti_rollback_1_lsb[FUSE_ROW_WIDTH_BYTE+1]={0};
static char htc_anti_rollback_1_msb[FUSE_ROW_WIDTH_BYTE+1]={0};
static char htc_anti_rollback_2_lsb[FUSE_ROW_WIDTH_BYTE+1]={0};
static char htc_anti_rollback_2_msb[FUSE_ROW_WIDTH_BYTE+1]={0};

/* ANTI-ROLLBACK-LSB-1 */
static int htc_anti_rollback_read_lsb_1(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%s\n", htc_anti_rollback_1_lsb);
    return 0;
}

static int htc_anti_rollback_open_lsb_1(struct inode *inode, struct file *file)
{
    return single_open(file, htc_anti_rollback_read_lsb_1, NULL);
}

static const struct file_operations htc_anti_rollback_fops_lsb_1 = {
    .owner      = THIS_MODULE,
    .open       = htc_anti_rollback_open_lsb_1,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

/* ANTI-ROLLBACK-MSB-1 */
static int htc_anti_rollback_read_msb_1(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%s\n", htc_anti_rollback_1_msb);
    return 0;
}

static int htc_anti_rollback_open_msb_1(struct inode *inode, struct file *file)
{
    return single_open(file, htc_anti_rollback_read_msb_1, NULL);
}

static const struct file_operations htc_anti_rollback_fops_msb_1 = {
    .owner      = THIS_MODULE,
    .open       = htc_anti_rollback_open_msb_1,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

/* ANTI-ROLLBACK-LSB-2 */
static int htc_anti_rollback_read_lsb_2(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%s\n", htc_anti_rollback_2_lsb);
    return 0;
}

static int htc_anti_rollback_open_lsb_2(struct inode *inode, struct file *file)
{
    return single_open(file, htc_anti_rollback_read_lsb_2, NULL);
}

static const struct file_operations htc_anti_rollback_fops_lsb_2 = {
    .owner      = THIS_MODULE,
    .open       = htc_anti_rollback_open_lsb_2,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

/* ANTI-ROLLBACK-MSB-2 */
static int htc_anti_rollback_read_msb_2(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%s\n",htc_anti_rollback_2_msb);
    return 0;
}

static int htc_anti_rollback_open_msb_2(struct inode *inode, struct file *file)
{
    return single_open(file, htc_anti_rollback_read_msb_2, NULL);
}

static const struct file_operations htc_anti_rollback_fops_msb_2 = {
    .owner      = THIS_MODULE,
    .open       = htc_anti_rollback_open_msb_2,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};


static void init_from_device_tree(void)
{
    struct device_node *misc_node = NULL;
    char *data = NULL;
    int property_size = 0;

    misc_node = of_find_node_by_path(DEVICE_TREE_MISC_PATH);
    if(NULL == misc_node)
        return;

	{
		data = (char *)of_get_property(misc_node, LABEL_ANTI_ROLLBACK_1_LSB, &property_size);
		pr_debug("%s - length: %d\n", __func__, property_size);
		if(property_size < sizeof(htc_anti_rollback_1_lsb))
			return;

		pr_info("%s - LABEL_ANTI_ROLLBACK_1_LSB: %s\n", __func__, data);
		memcpy(htc_anti_rollback_1_lsb, data, sizeof(htc_anti_rollback_1_lsb));
	}
	{
		data = (char *)of_get_property(misc_node, LABEL_ANTI_ROLLBACK_1_MSB, &property_size);
		pr_debug("%s - length: %d\n", __func__, property_size);
		if(property_size < sizeof(htc_anti_rollback_1_msb))
			return;

		pr_info("%s - LABEL_ANTI_ROLLBACK_1_MSB: %s\n", __func__, data);
		memcpy(htc_anti_rollback_1_msb, data, sizeof(htc_anti_rollback_1_msb));
	}
	{
		data = (char *)of_get_property(misc_node, LABEL_ANTI_ROLLBACK_2_LSB, &property_size);
		pr_debug("%s - length: %d\n", __func__, property_size);
		if(property_size < sizeof(htc_anti_rollback_2_lsb))
			return;

		pr_info("%s - LABEL_ANTI_ROLLBACK_2_LSB: %s\n", __func__, data);
		memcpy(htc_anti_rollback_2_lsb, data, sizeof(htc_anti_rollback_2_lsb));
	}
	{
		data = (char *)of_get_property(misc_node, LABEL_ANTI_ROLLBACK_2_MSB, &property_size);
		pr_debug("%s - length: %d\n", __func__, property_size);
		if(property_size < sizeof(htc_anti_rollback_2_msb))
			return;

		pr_info("%s - LABEL_ANTI_ROLLBACK_2_MSB: %s\n", __func__, data);
		memcpy(htc_anti_rollback_2_msb, data, sizeof(htc_anti_rollback_2_msb));
	}
}

static int __init anti_rollback_proc_init(void)
{
    struct proc_dir_entry *entry = NULL;

    pr_info("%s: Init HTC anti-rollback proc interface.\r\n", __func__);

    init_from_device_tree();

    /* NOTE: kernel 3.10 use proc_create_data to create /proc file node */
    entry = proc_create_data(PROC_ANTI_ROLLBACK_LSB_1, 0664, NULL, &htc_anti_rollback_fops_lsb_1, NULL);
    if (entry == NULL) {
        printk(KERN_ERR "%s: unable to create /proc%s entry\n", __func__, PROC_ANTI_ROLLBACK_LSB_1);
        return -ENOMEM;
    }
    entry = proc_create_data(PROC_ANTI_ROLLBACK_MSB_1, 0664, NULL, &htc_anti_rollback_fops_msb_1, NULL);
    if (entry == NULL) {
        printk(KERN_ERR "%s: unable to create /proc%s entry\n", __func__, PROC_ANTI_ROLLBACK_MSB_1);
        return -ENOMEM;
    }
    entry = proc_create_data(PROC_ANTI_ROLLBACK_LSB_2, 0664, NULL, &htc_anti_rollback_fops_lsb_2, NULL);
    if (entry == NULL) {
        printk(KERN_ERR "%s: unable to create /proc%s entry\n", __func__, PROC_ANTI_ROLLBACK_LSB_2);
        return -ENOMEM;
    }
    entry = proc_create_data(PROC_ANTI_ROLLBACK_MSB_2, 0664, NULL, &htc_anti_rollback_fops_msb_2, NULL);
    if (entry == NULL) {
        printk(KERN_ERR "%s: unable to create /proc%s entry\n", __func__, PROC_ANTI_ROLLBACK_MSB_2);
        return -ENOMEM;
    }

    return 0;
}

module_init(anti_rollback_proc_init);
MODULE_AUTHOR("HTC");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
