/*
 * Copyright (c) 2014 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#ifndef _RMI_FILTER_PALM_H
#define _RMI_FILTER_PALM_H
#include "rmi_input_dev.h"
#include "rmi_driver.h"
#include "rmi_touchpad.h"


#define RMI_OBJECT_NONE 0
#define RMI_OBJECT_FINGER 1
#define RMI_OBJECT_STYLUS 2
#define RMI_OBJECT_PALM 3
#define RMI_OBJECT_UNCLASSIFIED 4

#define RMI_PALM_FILTER_TIMEOUT_MS 0


struct rmi_filter_data_palm{
	enum rmi_filter_state * palm_state;
	unsigned long *last_palm_time_stamp;
	bool suppress_until_leave;
};

void rmi_palm_filter_create(struct rmi_input_dev *rmi_input);


#endif

