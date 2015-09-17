/*
 * Copyright (c) 2014 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

/* Zone filter for TouchPads and ClickPads */

#include "rmi_filter_zone.h"
EXPORT_SYMBOL_GPL(rmi_zone_filter_create);

static bool rmi_filter_zone_down_proc(struct rmi_filter * zone_filter,
					struct rmi_input_abs_pos *abs_pos)
{
	struct rmi_filter_data_zone *zone_data = (struct rmi_filter_data_zone *)zone_filter->filter_data;
	u8 current_finger_state = 0;
	bool report_to_os = true;
	current_finger_state = zone_data->finger_state | (1 << abs_pos->n_finger);
	switch( zone_data ->zone_state)
	{
		case rmi_state_idle:
			//if(zone_data->finger_state == 0) // first down
			if(rmi_finger_state_to_count(current_finger_state)  >  rmi_finger_state_to_count(zone_data->finger_state) ) // this is new finger
			{
				if(abs_pos->y > zone_data->max_button_zone_y)
				{
					if(rmi_finger_state_to_count(zone_data->finger_state) == 0)
						zone_data->zone_state = rmi_state_prepare;
					else if(rmi_finger_state_to_count(zone_data->finger_state) == 1)
					{
						if(time_after(abs_pos->time_stamp, zone_data->init_pos.time_stamp + msecs_to_jiffies(RMI_ZONE_FILTER_MULTIFINGER_TIMEOUT_MS)))
						{
							zone_data->zone_state = rmi_state_active_zone_2nd_finger;
							report_to_os = false;
						}
					}
					else
						zone_data->zone_state = rmi_state_idle;
					zone_data->filter_index = abs_pos->n_finger;
					pr_debug("zone suppress check. state->%d y=%d %d\n",zone_data->zone_state, abs_pos->y, zone_data->max_button_zone_y);
				}
				memcpy(&zone_data->init_pos, abs_pos, sizeof(struct rmi_input_abs_pos));
			}
			break;
		case rmi_state_prepare:
			if(rmi_finger_state_to_count(current_finger_state) > 1)
			{
				if(time_after_eq(abs_pos->time_stamp, zone_data->init_pos.time_stamp + msecs_to_jiffies(RMI_ZONE_FILTER_MULTIFINGER_TIMEOUT_MS)))
				{
					struct rmi_input_abs_pos abs_fake;
					memset(&abs_fake, 0, sizeof(struct rmi_input_abs_pos));
					zone_data->zone_state = rmi_state_faking;
					rmi_touchpad_inject(zone_filter->rmi_input,&abs_fake);
					zone_data->zone_state = rmi_state_active;
					pr_debug("start zone filter \n");

				}
				else
				{
					zone_data->zone_state = rmi_state_idle;
				}
			}
			else
			{
				if(abs(abs_pos->x - zone_data->init_pos.x) > RMI_ZONE_STATIONARY_AREA || abs(abs_pos->y - zone_data->init_pos.y) > RMI_ZONE_STATIONARY_AREA)
				{
					zone_data->zone_state = rmi_state_idle;
					pr_debug("zone filter disable. not stationary.\n");
				}
			}
			break;
		case rmi_state_active_zone_2nd_finger:
			if(abs_pos->n_finger == zone_data->filter_index)
			{
				if(abs(abs_pos->x - zone_data->init_pos.x) > RMI_ZONE_STATIONARY_AREA || abs(abs_pos->y - zone_data->init_pos.y) > RMI_ZONE_STATIONARY_AREA)
				{
					zone_data->zone_state = rmi_state_idle;
					pr_debug("zone filter disable. 2nd finger not stationary.\n");
				}
				else
				{
					report_to_os = false;
				}
			}
			break;
		case rmi_state_faking:
			break;
		case rmi_state_active:
			if(abs_pos->n_finger == zone_data->filter_index)
			{
				if(abs(abs_pos->x - zone_data->init_pos.x) > RMI_ZONE_STATIONARY_AREA || abs(abs_pos->y - zone_data->init_pos.y) > RMI_ZONE_STATIONARY_AREA)
				{
					zone_data->zone_state = rmi_state_idle;
					pr_debug("zone filter disable. finger achieve threshold.\n");
				}
				else
				{
					report_to_os = false;
				}
			}
			break;
		default:
			break;
	}
	if(current_finger_state != zone_data->finger_state) zone_data->finger_state = current_finger_state;
	return report_to_os;
}

static bool rmi_filter_zone_up_proc(struct rmi_filter * zone_filter,
					struct rmi_input_abs_pos *abs_pos)
{
	struct rmi_filter_data_zone *zone_data = (struct rmi_filter_data_zone *)zone_filter->filter_data;
	u8 current_finger_state = 0;
	if(zone_data->zone_state != rmi_state_faking)
	{
		current_finger_state = zone_data->finger_state & ~(1 << abs_pos->n_finger);
		if(current_finger_state != zone_data->finger_state)
		{
			zone_data->finger_state = current_finger_state;
		}
		if(current_finger_state == 0)
		{
			zone_data->zone_state = rmi_state_idle;
		}
		if(current_finger_state == 1 && zone_data->zone_state == rmi_state_active_zone_2nd_finger)
		{
			zone_data->zone_state = rmi_state_idle;
		}
	}
	return true;
}


void rmi_zone_filter_create(struct rmi_input_dev *rmi_input)
{
	struct rmi_filter *zone_filter;
	struct device *dev = rmi_input->input_dev->dev.parent;
	struct rmi_touchpad * tp = (struct rmi_touchpad *)rmi_input->context;
	struct rmi_filter_data_zone *zone_data ;

	zone_filter = devm_kzalloc(dev, sizeof(struct rmi_filter), GFP_KERNEL);
	if (!zone_filter)
		return;
	zone_filter->rmi_input = rmi_input;
	zone_filter->filter_data= devm_kzalloc(dev, sizeof(struct rmi_filter_data_zone), GFP_KERNEL);
	if(!zone_filter->filter_data)
		return;
	zone_data=(struct rmi_filter_data_zone *)zone_filter->filter_data;
	list_add_tail(&zone_filter->node, &tp->filter_list);
	zone_filter->pre_down_proc= rmi_filter_zone_down_proc;
	zone_filter->up_proc= rmi_filter_zone_up_proc;
	zone_data->max_button_zone_y= tp->y_max *3 / 4;
	return;
}

