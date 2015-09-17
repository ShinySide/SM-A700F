/*
 * Copyright (c) 2014 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#ifndef _RMI_FILTER_CLICK_H
#define _RMI_FILTER_CLICK_H
#include "rmi_input_dev.h"
#include "rmi_driver.h"
#include "rmi_touchpad.h"


struct rmi_filter_data_click{
	int click_down;
	int click_up;
	int click_down_x;
	int click_down_y;
	int click_down_z;
	bool start_filter;
	u8 finger_state;
};

void rmi_click_filter_create(struct rmi_input_dev *rmi_input);


#endif


