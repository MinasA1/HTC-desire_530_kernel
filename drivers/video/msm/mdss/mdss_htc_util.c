/* Copyright (c) 2009-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/debugfs.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/debug_display.h>
#include "mdss_htc_util.h"
#include "mdss_dsi.h"
#include "mdp3.h"

#define CABC_INDEX	 0
struct attribute_status htc_cabc_level[] = {
	{"cabc_level_ctl", 1, 1, 1},
};

struct attribute_status htc_mdss_csc_cfg_mv0[] = {
	{"mdss_csc_cfg_mv0", 512, 512, 512},
};
struct attribute_status htc_mdss_csc_cfg_mv1[] = {
	{"mdss_csc_cfg_mv1", 0, 0, 0},
};
struct attribute_status htc_mdss_csc_cfg_mv2[] = {
	{"mdss_csc_cfg_mv2", 0, 0, 0},
};
struct attribute_status htc_mdss_csc_cfg_mv3[] = {
	{"mdss_csc_cfg_mv3", 0, 0, 0},
};
struct attribute_status htc_mdss_csc_cfg_mv4[] = {
	{"mdss_csc_cfg_mv4", 512, 512, 512},
};
struct attribute_status htc_mdss_csc_cfg_mv5[] = {
	{"mdss_csc_cfg_mv5", 0, 0, 0},
};
struct attribute_status htc_mdss_csc_cfg_mv6[] = {
	{"mdss_csc_cfg_mv6", 0, 0, 0},
};
struct attribute_status htc_mdss_csc_cfg_mv7[] = {
	{"mdss_csc_cfg_mv7", 0, 0, 0},
};
struct attribute_status htc_mdss_csc_cfg_mv8[] = {
	{"mdss_csc_cfg_mv8", 512, 512, 512},
};

static struct delayed_work dimming_work;

struct msm_fb_data_type *mfd_instance;
#define DEBUG_BUF   2048
#define MIN_COUNT   9
#define DCS_MAX_CNT   128

static char debug_buf[DEBUG_BUF];
struct mdss_dsi_ctrl_pdata *ctrl_instance = NULL;
static char dcs_cmds[DCS_MAX_CNT];
static char *tmp;
static struct dsi_cmd_desc debug_cmd = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, 1}, dcs_cmds
};
static ssize_t dsi_cmd_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	u32 type, value;
	int cnt, i;
	struct dcs_cmd_req cmdreq;

	if (count >= sizeof(debug_buf) || count < MIN_COUNT)
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	if (!ctrl_instance)
		return count;

	
	debug_buf[count] = 0;

	
	cnt = (count) / 3 - 1;
	debug_cmd.dchdr.dlen = cnt;

	
	sscanf(debug_buf, "%x", &type);

	if (type == DTYPE_DCS_LWRITE)
		debug_cmd.dchdr.dtype = DTYPE_DCS_LWRITE;
	else if (type == DTYPE_GEN_LWRITE)
		debug_cmd.dchdr.dtype = DTYPE_GEN_LWRITE;
	else
		return -EFAULT;

	PR_DISP_INFO("%s: cnt=%d, type=0x%x\n", __func__, cnt, type);

	
	for (i = 0; i < cnt; i++) {
		if (i >= DCS_MAX_CNT) {
			PR_DISP_INFO("%s: DCS command count over DCS_MAX_CNT, Skip these commands.\n", __func__);
			break;
		}
		tmp = debug_buf + (3 * (i + 1));
		sscanf(tmp, "%x", &value);
		dcs_cmds[i] = value;
		PR_DISP_INFO("%s: value=0x%x\n", __func__, dcs_cmds[i]);
	}

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &debug_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl_instance, &cmdreq);
	PR_DISP_INFO("%s %d\n", __func__, count);
	return count;
}

static const struct file_operations dsi_cmd_fops = {
        .write = dsi_cmd_write,
};

static struct kobject *android_disp_kobj;
static char panel_name[MDSS_MAX_PANEL_LEN] = {0};
static ssize_t disp_vendor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	ret = snprintf(buf, PAGE_SIZE, "%s\n", panel_name);
	return ret;
}
static DEVICE_ATTR(vendor, S_IRUGO, disp_vendor_show, NULL);

char *disp_vendor(void){
	return panel_name;
}
EXPORT_SYMBOL(disp_vendor);

void htc_panel_info(const char *panel)
{
	android_disp_kobj = kobject_create_and_add("android_display", NULL);
	if (!android_disp_kobj) {
		PR_DISP_ERR("%s: subsystem register failed\n", __func__);
		return ;
	}

	if (sysfs_create_file(android_disp_kobj, &dev_attr_vendor.attr)) {
		PR_DISP_ERR("Fail to create sysfs file (vendor)\n");
		return ;
	}
	strlcpy(panel_name, panel, MDSS_MAX_PANEL_LEN);
}

static struct kobject *mdp3_perf_kobj;
static u32 mdp3_mode=0;

static ssize_t mdp3_perf_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", mdp3_mode);
}

static ssize_t mdp3_perf_store(struct kobject *kobj,
			struct kobj_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d", &mdp3_mode);
	return count;
}

static struct kobj_attribute mpd3_clock =  __ATTR(mdp3_clock, 0664, mdp3_perf_show, mdp3_perf_store);

void htc_mdp3_get_mode(u32 *mode)
{
   *mode = mdp3_mode;
}

void htc_mdp3_perf(void)
{
	mdp3_perf_kobj = kobject_create_and_add("mdp3_perf", NULL);
	if (!mdp3_perf_kobj) {
			PR_DISP_ERR("%s: subsystem register failed\n", __func__);
			return ;
	}

	if (sysfs_create_file(mdp3_perf_kobj, &mpd3_clock.attr)) {
			PR_DISP_ERR("Fail to create sysfs file (mdp3_clock)\n");
			return ;
	}
}

void htc_debugfs_init(struct msm_fb_data_type *mfd)
{
	struct mdss_panel_data *pdata;
	struct dentry *dent = debugfs_create_dir("htc_debug", NULL);

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_instance = container_of(pdata, struct mdss_dsi_ctrl_pdata,
						panel_data);

	if (IS_ERR(dent)) {
		pr_err(KERN_ERR "%s(%d): debugfs_create_dir fail, error %ld\n",
			__FILE__, __LINE__, PTR_ERR(dent));
		return;
	}

	if (debugfs_create_file("dsi_cmd", 0644, dent, 0, &dsi_cmd_fops)
			== NULL) {
		pr_err(KERN_ERR "%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
		return;
	}
	return;
}

static unsigned backlightvalue = 0;
static ssize_t camera_bl_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	ssize_t ret =0;
	sprintf(buf,"%s%u\n", "BL_CAM_MIN=", backlightvalue);
	ret = strlen(buf) + 1;
	return ret;
}

static ssize_t attrs_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int i;

	for (i = 0 ; i < ARRAY_SIZE(htc_cabc_level); i++) {
		if (strcmp(attr->attr.name, htc_cabc_level[i].title) == 0) {
			sprintf(buf,"%d\n", htc_cabc_level[i].cur_value);
		        ret = strlen(buf) + 1;
			break;
		}
	}

	if (strcmp(attr->attr.name, htc_mdss_csc_cfg_mv0[0].title) == 0) {
		sprintf(buf,"%d\n", htc_mdss_csc_cfg_mv0[0].cur_value);
		ret = strlen(buf) + 1;
	}
	if (strcmp(attr->attr.name, htc_mdss_csc_cfg_mv1[0].title) == 0) {
		sprintf(buf,"%d\n", htc_mdss_csc_cfg_mv1[0].cur_value);
		ret = strlen(buf) + 1;
	}
	if (strcmp(attr->attr.name, htc_mdss_csc_cfg_mv2[0].title) == 0) {
		sprintf(buf,"%d\n", htc_mdss_csc_cfg_mv2[0].cur_value);
		ret = strlen(buf) + 1;
	}
	if (strcmp(attr->attr.name, htc_mdss_csc_cfg_mv3[0].title) == 0) {
		sprintf(buf,"%d\n", htc_mdss_csc_cfg_mv3[0].cur_value);
		ret = strlen(buf) + 1;
	}
	if (strcmp(attr->attr.name, htc_mdss_csc_cfg_mv4[0].title) == 0) {
		sprintf(buf,"%d\n", htc_mdss_csc_cfg_mv4[0].cur_value);
		ret = strlen(buf) + 1;
	}
	if (strcmp(attr->attr.name, htc_mdss_csc_cfg_mv5[0].title) == 0) {
		sprintf(buf,"%d\n", htc_mdss_csc_cfg_mv5[0].cur_value);
		ret = strlen(buf) + 1;
	}
	if (strcmp(attr->attr.name, htc_mdss_csc_cfg_mv6[0].title) == 0) {
		sprintf(buf,"%d\n", htc_mdss_csc_cfg_mv6[0].cur_value);
		ret = strlen(buf) + 1;
	}
	if (strcmp(attr->attr.name, htc_mdss_csc_cfg_mv7[0].title) == 0) {
		sprintf(buf,"%d\n", htc_mdss_csc_cfg_mv7[0].cur_value);
		ret = strlen(buf) + 1;
	}
	if (strcmp(attr->attr.name, htc_mdss_csc_cfg_mv8[0].title) == 0) {
		sprintf(buf,"%d\n", htc_mdss_csc_cfg_mv8[0].cur_value);
		ret = strlen(buf) + 1;
	}

	return ret;
}

static ssize_t attr_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
	unsigned long res;
	int rc, i;

	rc = strict_strtoul(buf, 10, &res);
	if (rc) {
		pr_err("invalid parameter, %s %d\n", buf, rc);
		count = -EINVAL;
		goto err_out;
	}

	for (i = 0 ; i < ARRAY_SIZE(htc_cabc_level); i++) {
		if (strcmp(attr->attr.name, htc_cabc_level[i].title) == 0) {
			htc_cabc_level[i].req_value = res;
			break;
		}
	}

	if (strcmp(attr->attr.name, htc_mdss_csc_cfg_mv0[0].title) == 0) {
		htc_mdss_csc_cfg_mv0[0].req_value = res;
	}
	if (strcmp(attr->attr.name, htc_mdss_csc_cfg_mv1[0].title) == 0) {
		htc_mdss_csc_cfg_mv1[0].req_value = res;
	}
	if (strcmp(attr->attr.name, htc_mdss_csc_cfg_mv2[0].title) == 0) {
		htc_mdss_csc_cfg_mv2[0].req_value = res;
	}
	if (strcmp(attr->attr.name, htc_mdss_csc_cfg_mv3[0].title) == 0) {
		htc_mdss_csc_cfg_mv3[0].req_value = res;
	}
	if (strcmp(attr->attr.name, htc_mdss_csc_cfg_mv4[0].title) == 0) {
		htc_mdss_csc_cfg_mv4[0].req_value = res;
	}
	if (strcmp(attr->attr.name, htc_mdss_csc_cfg_mv5[0].title) == 0) {
		htc_mdss_csc_cfg_mv5[0].req_value = res;
	}
	if (strcmp(attr->attr.name, htc_mdss_csc_cfg_mv6[0].title) == 0) {
		htc_mdss_csc_cfg_mv6[0].req_value = res;
	}
	if (strcmp(attr->attr.name, htc_mdss_csc_cfg_mv7[0].title) == 0) {
		htc_mdss_csc_cfg_mv7[0].req_value = res;
	}
	if (strcmp(attr->attr.name, htc_mdss_csc_cfg_mv8[0].title) == 0) {
		htc_mdss_csc_cfg_mv8[0].req_value = res;
	}

err_out:
	return count;
}

static DEVICE_ATTR(backlight_info, S_IRUGO, camera_bl_show, NULL);
static DEVICE_ATTR(cabc_level_ctl, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(mdss_csc_cfg_mv0, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(mdss_csc_cfg_mv1, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(mdss_csc_cfg_mv2, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(mdss_csc_cfg_mv3, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(mdss_csc_cfg_mv4, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(mdss_csc_cfg_mv5, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(mdss_csc_cfg_mv6, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(mdss_csc_cfg_mv7, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(mdss_csc_cfg_mv8, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static struct attribute *htc_extend_attrs[] = {
	&dev_attr_backlight_info.attr,
	&dev_attr_cabc_level_ctl.attr,
	&dev_attr_mdss_csc_cfg_mv0.attr,
	&dev_attr_mdss_csc_cfg_mv1.attr,
	&dev_attr_mdss_csc_cfg_mv2.attr,
	&dev_attr_mdss_csc_cfg_mv3.attr,
	&dev_attr_mdss_csc_cfg_mv4.attr,
	&dev_attr_mdss_csc_cfg_mv5.attr,
	&dev_attr_mdss_csc_cfg_mv6.attr,
	&dev_attr_mdss_csc_cfg_mv7.attr,
	&dev_attr_mdss_csc_cfg_mv8.attr,
	NULL,
};

static struct attribute_group htc_extend_attr_group = {
	.attrs = htc_extend_attrs,
};

void htc_register_attrs(struct kobject *led_kobj, struct msm_fb_data_type *mfd)
{
	int rc;
	struct mdss_panel_info *panel_info = mfd->panel_info;

	rc = sysfs_create_group(led_kobj, &htc_extend_attr_group);
	if (rc)
		pr_err("sysfs group creation failed, rc=%d\n", rc);

	
	if ((panel_info->csc_type == CSC_TYPE_COLOR_TEMPERATURE) ||
			(panel_info->csc_type == CSC_TYPE_COLOR_SATURATION) ||
			(panel_info->csc_type == CSC_TYPE_COLOR_SATURATION_AND_HUE)) {
		htc_mdss_csc_cfg_mv0[0].req_value = panel_info->csc_mv0;
		htc_mdss_csc_cfg_mv4[0].req_value = panel_info->csc_mv4;
		htc_mdss_csc_cfg_mv8[0].req_value = panel_info->csc_mv8;
		if ((panel_info->csc_type == CSC_TYPE_COLOR_SATURATION) ||
				(panel_info->csc_type == CSC_TYPE_COLOR_SATURATION_AND_HUE)) {
			htc_mdss_csc_cfg_mv1[0].req_value = panel_info->csc_mv1;
			htc_mdss_csc_cfg_mv2[0].req_value = panel_info->csc_mv2;
			htc_mdss_csc_cfg_mv3[0].req_value = panel_info->csc_mv3;
			htc_mdss_csc_cfg_mv5[0].req_value = panel_info->csc_mv5;
			htc_mdss_csc_cfg_mv6[0].req_value = panel_info->csc_mv6;
			htc_mdss_csc_cfg_mv7[0].req_value = panel_info->csc_mv7;
		}
	}

	return;
}

void htc_reset_status(void)
{
	int i;

	for (i = 0 ; i < ARRAY_SIZE(htc_cabc_level); i++) {
		htc_cabc_level[i].cur_value = htc_cabc_level[i].def_value;
	}

	htc_mdss_csc_cfg_mv0[0].cur_value = htc_mdss_csc_cfg_mv0[0].def_value;
	htc_mdss_csc_cfg_mv1[0].cur_value = htc_mdss_csc_cfg_mv1[0].def_value;
	htc_mdss_csc_cfg_mv2[0].cur_value = htc_mdss_csc_cfg_mv2[0].def_value;
	htc_mdss_csc_cfg_mv3[0].cur_value = htc_mdss_csc_cfg_mv3[0].def_value;
	htc_mdss_csc_cfg_mv4[0].cur_value = htc_mdss_csc_cfg_mv4[0].def_value;
	htc_mdss_csc_cfg_mv5[0].cur_value = htc_mdss_csc_cfg_mv5[0].def_value;
	htc_mdss_csc_cfg_mv6[0].cur_value = htc_mdss_csc_cfg_mv6[0].def_value;
	htc_mdss_csc_cfg_mv7[0].cur_value = htc_mdss_csc_cfg_mv7[0].def_value;
	htc_mdss_csc_cfg_mv8[0].cur_value = htc_mdss_csc_cfg_mv8[0].def_value;

	return;
}

void htc_register_camera_bkl(int level)
{
	backlightvalue = level;
}

void htc_set_cabc(struct msm_fb_data_type *mfd)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct dcs_cmd_req cmdreq;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	if ((htc_cabc_level[CABC_INDEX].req_value > 2) || (htc_cabc_level[CABC_INDEX].req_value < 0))
		return;

	if (!ctrl_pdata->cabc_off_cmds.cmds)
		return;

	if (!ctrl_pdata->cabc_ui_cmds.cmds)
		return;

	if (!ctrl_pdata->cabc_video_cmds.cmds)
		return;

	if (htc_cabc_level[CABC_INDEX].req_value == htc_cabc_level[CABC_INDEX].cur_value)
		return;

	memset(&cmdreq, 0, sizeof(cmdreq));

	if (htc_cabc_level[CABC_INDEX].req_value == 0) {
		cmdreq.cmds = ctrl_pdata->cabc_off_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->cabc_off_cmds.cmd_cnt;
	} else if (htc_cabc_level[CABC_INDEX].req_value == 1) {
		cmdreq.cmds = ctrl_pdata->cabc_ui_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->cabc_ui_cmds.cmd_cnt;
	} else if (htc_cabc_level[CABC_INDEX].req_value == 2) {
		cmdreq.cmds = ctrl_pdata->cabc_video_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->cabc_video_cmds.cmd_cnt;
	} else {
		cmdreq.cmds = ctrl_pdata->cabc_ui_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->cabc_ui_cmds.cmd_cnt;
	}

	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	htc_cabc_level[CABC_INDEX].cur_value = htc_cabc_level[CABC_INDEX].req_value;
	PR_DISP_INFO("%s cabc mode=%d\n", __func__, htc_cabc_level[CABC_INDEX].cur_value);
	return;
}

struct mdp_csc_cfg_data htc_set_csc_cfg(bool *csc_update, int csc_type)
{
	int i;
	struct mdp_csc_cfg_data data;

	if (csc_type == CSC_TYPE_NONE) {
		*csc_update = false;
		
		return data;
	}

	if ((htc_mdss_csc_cfg_mv0[0].req_value == htc_mdss_csc_cfg_mv0[0].cur_value) &&
			(htc_mdss_csc_cfg_mv1[0].req_value == htc_mdss_csc_cfg_mv1[0].cur_value) &&
			(htc_mdss_csc_cfg_mv2[0].req_value == htc_mdss_csc_cfg_mv2[0].cur_value) &&
			(htc_mdss_csc_cfg_mv3[0].req_value == htc_mdss_csc_cfg_mv3[0].cur_value) &&
			(htc_mdss_csc_cfg_mv4[0].req_value == htc_mdss_csc_cfg_mv4[0].cur_value) &&
			(htc_mdss_csc_cfg_mv5[0].req_value == htc_mdss_csc_cfg_mv5[0].cur_value) &&
			(htc_mdss_csc_cfg_mv6[0].req_value == htc_mdss_csc_cfg_mv6[0].cur_value) &&
			(htc_mdss_csc_cfg_mv7[0].req_value == htc_mdss_csc_cfg_mv7[0].cur_value) &&
			(htc_mdss_csc_cfg_mv8[0].req_value == htc_mdss_csc_cfg_mv8[0].cur_value)) {
		*csc_update = false;
		
		return data;
	}

	
	data.csc_data.csc_mv[0] = htc_mdss_csc_cfg_mv0[0].req_value;
	data.csc_data.csc_mv[4] = htc_mdss_csc_cfg_mv4[0].req_value;
	data.csc_data.csc_mv[8] = htc_mdss_csc_cfg_mv8[0].req_value;
	if ((csc_type == CSC_TYPE_COLOR_SATURATION) ||
			((csc_type == CSC_TYPE_COLOR_SATURATION_AND_HUE))) {
		data.csc_data.csc_mv[1] = htc_mdss_csc_cfg_mv1[0].req_value;
		data.csc_data.csc_mv[2] = htc_mdss_csc_cfg_mv2[0].req_value;
		data.csc_data.csc_mv[3] = htc_mdss_csc_cfg_mv3[0].req_value;
		data.csc_data.csc_mv[5] = htc_mdss_csc_cfg_mv5[0].req_value;
		data.csc_data.csc_mv[6] = htc_mdss_csc_cfg_mv6[0].req_value;
		data.csc_data.csc_mv[7] = htc_mdss_csc_cfg_mv7[0].req_value;
	} else { 
		data.csc_data.csc_mv[1] = htc_mdss_csc_cfg_mv1[0].req_value = 0;
		data.csc_data.csc_mv[2] = htc_mdss_csc_cfg_mv2[0].req_value = 0;
		data.csc_data.csc_mv[3] = htc_mdss_csc_cfg_mv3[0].req_value = 0;
		data.csc_data.csc_mv[5] = htc_mdss_csc_cfg_mv5[0].req_value = 0;
		data.csc_data.csc_mv[6] = htc_mdss_csc_cfg_mv6[0].req_value = 0;
		data.csc_data.csc_mv[7] = htc_mdss_csc_cfg_mv7[0].req_value = 0;
	}

	
	for (i = 0 ; i < 3 ; i++)
		data.csc_data.csc_pre_bv[i] = 0;

	
	for (i = 0 ; i < 3 ; i++)
		data.csc_data.csc_post_bv[i] = 0;

	
	for (i = 0 ; i < 6 ; i++) {
		if (i == 0 || i == 2 || i == 4)
			data.csc_data.csc_pre_lv[i] = 0;
		else
			data.csc_data.csc_pre_lv[i] = 255;
	}

	
	for (i = 0 ; i < 6 ; i++) {
		if (i == 0 || i == 2 || i == 4)
			data.csc_data.csc_post_lv[i] = 0;
		else
			data.csc_data.csc_post_lv[i] = 255;
	}

	htc_mdss_csc_cfg_mv0[0].cur_value = htc_mdss_csc_cfg_mv0[0].req_value;
	htc_mdss_csc_cfg_mv4[0].cur_value = htc_mdss_csc_cfg_mv4[0].req_value;
	htc_mdss_csc_cfg_mv8[0].cur_value = htc_mdss_csc_cfg_mv8[0].req_value;
	if ((csc_type == CSC_TYPE_COLOR_SATURATION) ||
			(csc_type == CSC_TYPE_COLOR_SATURATION_AND_HUE)) {
		htc_mdss_csc_cfg_mv1[0].cur_value = htc_mdss_csc_cfg_mv1[0].req_value;
		htc_mdss_csc_cfg_mv2[0].cur_value = htc_mdss_csc_cfg_mv2[0].req_value;
		htc_mdss_csc_cfg_mv3[0].cur_value = htc_mdss_csc_cfg_mv3[0].req_value;
		htc_mdss_csc_cfg_mv5[0].cur_value = htc_mdss_csc_cfg_mv5[0].req_value;
		htc_mdss_csc_cfg_mv6[0].cur_value = htc_mdss_csc_cfg_mv6[0].req_value;
		htc_mdss_csc_cfg_mv7[0].cur_value = htc_mdss_csc_cfg_mv7[0].req_value;
	}

	*csc_update = true;

	PR_DISP_INFO("htc_set_csc_cfg: %d %d %d %d %d %d %d %d %d\n",
		htc_mdss_csc_cfg_mv0[0].req_value,
		htc_mdss_csc_cfg_mv1[0].req_value,
		htc_mdss_csc_cfg_mv2[0].req_value,
		htc_mdss_csc_cfg_mv3[0].req_value,
		htc_mdss_csc_cfg_mv4[0].req_value,
		htc_mdss_csc_cfg_mv5[0].req_value,
		htc_mdss_csc_cfg_mv6[0].req_value,
		htc_mdss_csc_cfg_mv7[0].req_value,
		htc_mdss_csc_cfg_mv8[0].req_value);

	return data;
}

static void dimming_do_work(struct work_struct *work)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct dcs_cmd_req cmdreq;

	pdata = dev_get_platdata(&mfd_instance->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	memset(&cmdreq, 0, sizeof(cmdreq));

	cmdreq.cmds = ctrl_pdata->dimming_on_cmds.cmds;
	cmdreq.cmds_cnt = ctrl_pdata->dimming_on_cmds.cmd_cnt;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	PR_DISP_INFO("dimming on\n");
}

void htc_dimming_on(struct msm_fb_data_type *mfd)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	if (!ctrl_pdata->dimming_on_cmds.cmds)
		return;

	mfd_instance = mfd;

	INIT_DELAYED_WORK(&dimming_work, dimming_do_work);

	schedule_delayed_work(&dimming_work, msecs_to_jiffies(1000));
	return;
}

void htc_dimming_off(void)
{
	
	cancel_delayed_work_sync(&dimming_work);
}
