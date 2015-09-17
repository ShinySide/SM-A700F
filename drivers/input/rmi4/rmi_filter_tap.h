
/*
 * Copyright (c) 2014 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#ifndef _RMI_FILTER_TAP_H
#define _RMI_FILTER_TAP_H
#include "rmi_input_dev.h"
#include "rmi_driver.h"
#include "rmi_touchpad.h"


#define MAX_TAP_TIME_IN_MS			   150
#define MAX_TAP_DISTANCE			   80



struct rmi_filter_data_tap {
	bool finger_down;
	bool finger_up;
	int init_x;
	int init_y;
	int tap_x_maximum;
	int tap_y_maximum;
	struct timeval timeval_down;
	bool start_filter;
	u8 finger_state;
	unsigned long tap_first_down;
	};

void rmi_tap_filter_create(struct rmi_input_dev *rmi_input);

#endif

