/*
 * Copyright (c) 2014 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#ifndef _RMI_FILTER_ZONE_H
#define _RMI_FILTER_ZONE_H
#include "rmi_input_dev.h"
#include "rmi_driver.h"
#include "rmi_touchpad.h"

#define RMI_ZONE_FILTER_MULTIFINGER_TIMEOUT_MS 0
#define RMI_ZONE_STATIONARY_AREA 100

#define RMI_ZONE_STATE_IDLE


struct rmi_filter_data_zone {
	int max_button_zone_y;
	struct rmi_input_abs_pos init_pos;
	enum rmi_filter_state zone_state;
	u8 finger_state;
	u8 filter_index;
	};

void rmi_zone_filter_create(struct rmi_input_dev *rmi_input);

#endif


