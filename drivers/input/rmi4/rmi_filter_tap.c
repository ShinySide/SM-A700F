/*
 * Copyright (c) 2014 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

/* Tap filter for TouchPads and ClickPads */

#include "rmi_filter_tap.h"
EXPORT_SYMBOL_GPL(rmi_tap_filter_create);
#ifdef TAP_FILTER2
static bool rmi_filter_tap_down_proc(struct rmi_filter * tap_filter,
					struct rmi_input_abs_pos *abs_pos)
{
	struct rmi_touchpad * tp = tap_filter->rmi_input->context;
	struct rmi_touchpad_finger * finger = &tp->fingers[abs_pos->n_finger];
	struct rmi_input_abs_pos last_pkt;

	memset(&last_pkt, 0, sizeof(struct rmi_input_abs_pos));
	last_pkt.n_finger = abs_pos->n_finger;
	last_pkt.tool_type = abs_pos->tool_type;
	rmi_touchpad_queue_peek(&finger->finger_fifo, &last_pkt, 0);

	if (  finger->tap_status == 0 &&  abs_pos->z != 0 ) {	
		finger->finger_down_pkt = *abs_pos;	
		finger->tap_status = 1;
		pr_debug("aaa 5 finger status 1- x:%d y:%d z:%d wx:%d wy:%d\n",
                         abs_pos->x, abs_pos->y, abs_pos->z,
			abs_pos->w_x,abs_pos->w_y);
		return false;
	}
	if ( finger->tap_status == 1 && abs_pos->z != 0 ) {

		//tracking pre-palm
		if ( abs_pos->w_max >= tp->highw_threshold ) {
			pr_debug("aaa 5 palm pkt- x:%d y:%d z:%d wx:%d wy:%d\n",
                        	abs_pos->x, abs_pos->y, abs_pos->z,
				abs_pos->w_x,abs_pos->w_y);
			finger->last_palm_time_stamp = abs_pos->time_stamp;
			return false;		
			//*abs_pos = last_pkt;
			//return true;

		}
		//tap --make sure time interval and distance in range
		if ( 	time_diff_ms (abs_pos->time_stamp,  finger->finger_down_pkt.time_stamp) 
			< MAX_TAP_TIME_IN_MS  &&
			abs(abs_pos->x - finger->finger_down_pkt.x) < MAX_TAP_DISTANCE &&
			abs(abs_pos->y - finger->finger_down_pkt.y) < MAX_TAP_DISTANCE )
		{
			pr_debug("aaa 5 tap pkt- x:%d y:%d z:%d wx:%d wy:%d\n",
                        	 abs_pos->x, abs_pos->y, abs_pos->z,
				abs_pos->w_x,abs_pos->w_y);
			return false;

			//*abs_pos = last_pkt;
			//return true;
		}

		// if detect palm, ignore a few packet after palm
		if (	time_diff_ms(abs_pos->time_stamp,
			finger->last_palm_time_stamp) < MAX_TAP_TIME_IN_MS )
		{
			pr_debug("aaa 5 ignore pkt - x:%d y:%d z:%d wx:%d wy:%d\n",
                        	 abs_pos->x, abs_pos->y, abs_pos->z,
				abs_pos->w_x,abs_pos->w_y);
			return false;

			//*abs_pos = last_pkt;
			//return true;
		}
		// normal finger move packet
		finger->tap_status = 2;
		pr_debug("aaa 5 normal status 2- x:%d y:%d z:%d wx:%d wy:%d, time: %u\n",
                         abs_pos->x, abs_pos->y, abs_pos->z,
			abs_pos->w_x,abs_pos->w_y, 
			time_diff_ms (abs_pos->time_stamp,  finger->finger_down_pkt.time_stamp));
		return true;
	}

	//finger up, this will generate tap 
	if ( finger->tap_status == 1  && abs_pos->z == 0 
		&& time_diff_ms(abs_pos->time_stamp,
		finger->finger_down_pkt.time_stamp) < MAX_TAP_TIME_IN_MS) 
	{  
		_rmi_input_report_abs(tap_filter->rmi_input, &finger->finger_down_pkt);
		finger->tap_status = 3;

		pr_debug("aaa 5 tap first pkt- x:%d y:%d z:%d wx:%d wy:%d\n",
               	         finger->finger_down_pkt.x, finger->finger_down_pkt.y,
			finger->finger_down_pkt.z,
			finger->finger_down_pkt.w_x,finger->finger_down_pkt.w_y);
		return false;
	} else if (finger->tap_status == 3 && abs_pos->z ==0) {
		finger->tap_status = 0;
		pr_debug("aaa 5 tap up pkt\n");
		return true;
	} 
	else if (abs_pos->z == 0) {
		finger->tap_status = 0;
	}

	return true;
}

static bool rmi_filter_tap_up_proc(struct rmi_filter * tap_filter, struct rmi_input_abs_pos * abs)
{
	return rmi_filter_tap_down_proc(tap_filter,abs);
}

#else
static inline bool is_in_tap_area(struct rmi_filter * tap_filter, struct rmi_input_abs_pos * abs)
{
	if(abs(tap_filter->init_x - abs->x) <= MAX_TAP_DISTANCE && abs(tap_filter->init_y - abs->y) <= MAX_TAP_DISTANCE + 10 )
	{
		return true;
	}
	else
		return false;
}

static inline bool is_in_tap_duration(struct rmi_filter_data_tap* tap_data, struct rmi_input_abs_pos * abs)
{
	if(jiffies_to_msecs(abs->time_stamp - tap_data->tap_first_down) < MAX_TAP_TIME_IN_MS)
	{
		return true;
	}
	else
		return false;
}


static bool rmi_filter_tap_down_proc(struct rmi_filter * tap_filter, struct rmi_input_abs_pos * abs)
{
	struct rmi_filter_data_tap *tap_data = (struct rmi_filter_data_tap *)tap_filter->filter_data;
	u8 current_finger_state = 0;
	current_finger_state = tap_data->finger_state | (1 << abs->n_finger);
	if(!tap_filter->start_filter)
	{
		if(tap_data->finger_state == 0) // first finger down.
		{
			pr_debug("%s tap down (%d,%d)%d \n",__func__,abs->x,abs->y,current_finger_state);
			tap_filter->finger_down = true;
			tap_filter->init_x = abs->x;
			tap_filter->init_y = abs->y;
			tap_filter->start_filter = true;
			tap_data->tap_first_down = abs->time_stamp;
		}
	}
	else
	{
		if(rmi_finger_state_to_count(current_finger_state) > 1 )
		{
			pr_debug("more than one finger, tap filter disabled \n");
			tap_filter->start_filter = false;
		}
		else
		{
			if(is_in_tap_duration(tap_data,abs))
			{
				if(is_in_tap_area(tap_filter,abs))
				{
					abs->x= tap_filter->init_x;
					abs->y= tap_filter->init_y;
				}
				else
				{
					pr_debug("%s x_diff=%ld, y_diff=%ld x_max=%d, y max=%d \n",__func__,abs(tap_filter->init_x - abs->x),abs(tap_filter->init_y - abs->y),tap_data->tap_x_maximum,tap_data->tap_y_maximum);
					tap_filter->start_filter = false;
				}
			}
			else
			{
				pr_debug("%s time exceed %d ms\n",__func__,jiffies_to_msecs(abs->time_stamp - tap_data->tap_first_down));
				tap_filter->start_filter = false;
			}
		}
	}
	if(tap_data->finger_state != current_finger_state)
	{
		tap_data->finger_state = current_finger_state;
	}
	return true;
}
static bool rmi_filter_tap_up_proc(struct rmi_filter * tap_filter, struct rmi_input_abs_pos * abs)
{
	struct rmi_filter_data_tap *tap_data = (struct rmi_filter_data_tap *)tap_filter->filter_data;
	u8 current_finger_state = 0;
	current_finger_state = tap_data->finger_state & ~( 1 << abs->n_finger);
	if(tap_data->finger_state !=0 && current_finger_state == 0)
	{
		tap_filter->start_filter = false;
	}
	if(tap_data->finger_state != current_finger_state)
	{
		tap_data->finger_state = current_finger_state;
	}
	return true;
}
#endif

void rmi_tap_filter_create(struct rmi_input_dev *rmi_input)
{
	struct rmi_filter *tap_filter;
	struct device *dev = rmi_input->input_dev->dev.parent;
	struct rmi_touchpad * tp = (struct rmi_touchpad *)rmi_input->context;
	struct rmi_filter_data_tap *tap_data ;

	tap_filter = devm_kzalloc(dev, sizeof(struct rmi_filter), GFP_KERNEL);
	if (!tap_filter)
		return;
	tap_filter->rmi_input = rmi_input;
	tap_filter->filter_data= devm_kzalloc(dev, sizeof(struct rmi_filter_data_tap), GFP_KERNEL);
	if(!tap_filter->filter_data)
		return;
	tap_data=(struct rmi_filter_data_tap *)tap_filter->filter_data;
	list_add_tail(&tap_filter->node, &tp->filter_list);
	tap_filter->down_proc= rmi_filter_tap_down_proc;
	tap_filter->up_proc= rmi_filter_tap_up_proc;
	tap_data->tap_x_maximum= tp->x_max / 50;
	tap_data->tap_y_maximum= tp->y_max / 30;
	tap_data->finger_state = 0;
	return;
}

