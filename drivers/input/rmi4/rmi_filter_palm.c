/*
 * Copyright (c) 2014 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

/* Palm filter for TouchPads and ClickPads */
#include "rmi_filter_palm.h"
EXPORT_SYMBOL_GPL(rmi_palm_filter_create);
bool inline is_palm_packet(struct rmi_input_abs_pos * abs_pos)
{
	if(abs_pos->type == RMI_OBJECT_PALM)
		return true;
	else
		return false;
}

static bool rmi_filter_palm_down_proc(struct rmi_filter * palm_filter,
					struct rmi_input_abs_pos *abs_pos)
{
	bool palm_suppress = false;
	struct rmi_filter_data_palm *palm_data = (struct rmi_filter_data_palm*)palm_filter->filter_data;
	switch (palm_data->palm_state[abs_pos->n_finger])
	{
		case rmi_state_idle:
			if(is_palm_packet(abs_pos))
			{
				struct rmi_touchpad * tp = (struct rmi_touchpad *)palm_filter->rmi_input->context;
				struct rmi_touchpad_finger * finger = &tp->fingers[abs_pos->n_finger];
				if(!finger->reporting)
				{
					rmi_touchpad_reset_finger(palm_filter->rmi_input, abs_pos);
				}
				palm_data->palm_state[abs_pos->n_finger] = rmi_state_active;
				palm_data->last_palm_time_stamp[abs_pos->n_finger]= abs_pos->time_stamp;
				palm_suppress = true;
				pr_debug("%s: finger (%d) palm start suppress \n",__func__,abs_pos->n_finger);
			}
			break;
		case rmi_state_active:
			if(!is_palm_packet(abs_pos) && !palm_data->suppress_until_leave)
			{
				if(time_after_eq(abs_pos->time_stamp, palm_data->last_palm_time_stamp[abs_pos->n_finger] + msecs_to_jiffies(RMI_PALM_FILTER_TIMEOUT_MS)))
				{
					palm_data->palm_state[abs_pos->n_finger] = rmi_state_idle;
					palm_suppress = false;
				}
			}
			else
			{
				palm_data->last_palm_time_stamp[abs_pos->n_finger] = abs_pos->time_stamp;
				palm_suppress = true;
			}
			break;
		default:
			break;
	}
	return !palm_suppress;
}

static bool rmi_filter_palm_up_proc(struct rmi_filter * palm_filter,
					struct rmi_input_abs_pos *abs_pos)
{
	struct rmi_filter_data_palm *palm_data = (struct rmi_filter_data_palm*)palm_filter->filter_data;
	palm_data->palm_state[abs_pos->n_finger] = rmi_state_idle;
	return true;
}

void rmi_palm_filter_create(struct rmi_input_dev *rmi_input)
{
	struct rmi_filter *palm_filter;
	struct device *dev = rmi_input->input_dev->dev.parent;
	struct rmi_touchpad * tp = (struct rmi_touchpad *)rmi_input->context;
	struct rmi_filter_data_palm *palm_data ;

	palm_filter = devm_kzalloc(dev, sizeof(struct rmi_filter), GFP_KERNEL);
	if (!palm_filter)
		return;
	palm_filter->rmi_input = rmi_input;
	palm_filter->filter_data= devm_kzalloc(dev, sizeof(struct rmi_filter_data_palm), GFP_KERNEL);
	if(!palm_filter->filter_data)
		return;
	palm_data=(struct rmi_filter_data_palm *)palm_filter->filter_data;
	palm_data->palm_state = devm_kzalloc(dev, sizeof(enum rmi_filter_state)*(tp->finger_count),
					GFP_KERNEL);
	if(!palm_data->palm_state)
		return;
	palm_data->last_palm_time_stamp= devm_kzalloc(dev, sizeof(unsigned long)*(tp->finger_count),
					GFP_KERNEL);
	if(!palm_data->last_palm_time_stamp)
		return;
	palm_data->suppress_until_leave = true;
	list_add_tail(&palm_filter->node, &tp->filter_list);
	palm_filter->pre_down_proc= rmi_filter_palm_down_proc;
	palm_filter->up_proc= rmi_filter_palm_up_proc;
	return;
}


