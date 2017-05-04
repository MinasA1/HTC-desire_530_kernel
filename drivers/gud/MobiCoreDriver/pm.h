/*
 * Copyright (c) 2013 TRUSTONIC LIMITED
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#ifndef _MC_PM_H_
#define _MC_PM_H_

#include "main.h"


#define NO_SLEEP_REQ	0
#define REQ_TO_SLEEP	1

#define NORMAL_EXECUTION	0
#define READY_TO_SLEEP		1

#define DAEMON_BACKOFF_TIME	500

int mc_pm_initialize(struct mc_context *context);
int mc_pm_free(void);
int mc_pm_clock_initialize(void);
void mc_pm_clock_finalize(void);
int mc_pm_clock_enable(void);
void mc_pm_clock_disable(void);
bool mc_pm_sleep_ready(void);

#endif 
