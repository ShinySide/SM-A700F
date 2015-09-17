/*
 * Copyright (c) 2014 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

/* Click for TouchPads and ClickPads */

#include "rmi_filter_click.h"
EXPORT_SYMBOL_GPL(rmi_click_filter_create);
#ifdef CLICK_FILTER2
static void clickJump_pos_rollback(struct rmi_filter * click_filter, struct rmi_input_abs_pos * abs_pos)
{
	struct rmi_touchpad * tp = click_filter->rmi_input->context;
	struct rmi_touchpad_finger * finger = &tp->fingers[abs_pos->n_finger];
	struct rmi_filter_data_click *click_data = (struct rmi_filter_data_click*)click_filter->filter_data;
	int index;
	int pre_x = 0;
	int pre_y = 0;
	int i;

	int min_move = 0;
	int pre_move = 0;
	int min_index = 0;
	int dx, dy;
	int cur_move = 0;
	int head = 0;
	int big_jump = 0;

	head = finger->finger_buffer.queue_head;
	index = (head - (PACKET_SEARCH_LENGTH+1 + 1)) &
			(RMI_TOUCHPAD_FINGER_QUEUE_SIZE - 1);
	pre_x = finger->finger_buffer.pos_data[index].x;
	pre_y = finger->finger_buffer.pos_data[index].y;

	for (i = PACKET_SEARCH_LENGTH; i >= 0; i--) {
		index = (head - (i + 1)) &(RMI_TOUCHPAD_FINGER_QUEUE_SIZE - 1);
		finger->finger_buffer.pos_data[index].dx = abs(((int)finger->finger_buffer.pos_data[index].x) - 
							pre_x);
		pre_x = finger->finger_buffer.pos_data[index].x;
		finger->finger_buffer.pos_data[index].dy = abs(((int)finger->finger_buffer.pos_data[index].y) - 
							pre_y);
		pre_y = finger->finger_buffer.pos_data[index].y;
	}
	for (i=0; i< PACKET_SEARCH_LENGTH; i++) {
		index = (head - (i + 1)) &(RMI_TOUCHPAD_FINGER_QUEUE_SIZE - 1);
		if (finger->finger_buffer.pos_data[index].dx > 100 ||
			finger->finger_buffer.pos_data[index].dy > 100 )
		{
			big_jump = 1;
			min_index = i - 1;
			break;
		}
	}
	if (big_jump == 0) {
		for (i=0;i<PACKET_WINDOW_SIZE;i++) { //calculate first value
			index = (head - (i + 1)) & (RMI_TOUCHPAD_FINGER_QUEUE_SIZE - 1);
			dx=abs(finger->finger_buffer.pos_data[index].dx);
			dy=abs(finger->finger_buffer.pos_data[index].dy);
			min_move=min_move+max(dx, dy);
		}
		pre_move = min_move;
		for (i=PACKET_WINDOW_SIZE;i<PACKET_SEARCH_LENGTH;i++) {
			index = (head - (i + 1)) &(RMI_TOUCHPAD_FINGER_QUEUE_SIZE - 1);
			dx = abs(finger->finger_buffer.pos_data[index].dx);
			dy = abs(finger->finger_buffer.pos_data[index].dy);
			cur_move = pre_move+max(dx, dy);
			index = (head - (i + 1)+PACKET_WINDOW_SIZE) &
				(RMI_TOUCHPAD_FINGER_QUEUE_SIZE - 1);
			dx = abs(finger->finger_buffer.pos_data[index].dx);
			dy = abs(finger->finger_buffer.pos_data[index].dy);
			cur_move = cur_move- max(dx,dy);
			if (cur_move< min_move) {
				min_move = cur_move;
				min_index = i;
			}
			pre_move = cur_move;
			}
	}
	index = (head - (min_index + 1)) &(RMI_TOUCHPAD_FINGER_QUEUE_SIZE - 1);
	click_data->click_down_x= finger->finger_buffer.pos_data[index].x;
	click_data->click_down_y = finger->finger_buffer.pos_data[index].y;
	click_data->click_down_z= finger->finger_buffer.pos_data[index].z;
	pr_debug("aaa min_move : %d, min_index= %d,"
		" org_x= %d, org_y= %d, org_z= %d, big_jump= %d\n",
		min_move, min_index, click_data->click_down_x, click_data->click_down_y, click_data->click_down_z, big_jump);
	for (i = 0; i < RMI_TOUCHPAD_FINGER_QUEUE_SIZE; i++) {
		index = (head - (i + 1)) &(RMI_TOUCHPAD_FINGER_QUEUE_SIZE - 1);
		pr_debug("aaa dump: x= %d, y= %d, z= %d, dx= %d, dy= %d\n",
			finger->finger_buffer.pos_data[index].x,
			finger->finger_buffer.pos_data[index].y,
			finger->finger_buffer.pos_data[index].z,
			finger->finger_buffer.pos_data[index].dx,
			finger->finger_buffer.pos_data[index].dy
			);
	}
}



static bool rmi_filter_click_down_proc(struct rmi_filter * click_filter, struct rmi_input_abs_pos * abs_pos)
{
	struct rmi_filter_data_click *click_data = (struct rmi_filter_data_click*)click_filter->filter_data;

	if (abs_pos->n_finger != 0) return true;
	if (abs_pos->z == 0 ) {
		click_data->click_down = 0;
		click_data->click_up = 0;
		return true;
	}
	if (click_data->click_down == 1) {
		pr_debug("aaa start to find original click position\n" );
		clickJump_pos_rollback(click_filter, abs_pos);
	}

	if (click_data->click_down > 0 &&
		(abs(abs_pos->x - click_data->click_down_x) <
		RMI_TOUCHPAD_CLICK_JUMP_THRESHOLD + abs_pos->z - click_data->click_down_z) &&
		(abs(abs_pos->y - click_data->click_down_y) <
		RMI_TOUCHPAD_CLICK_JUMP_THRESHOLD + abs_pos->z - click_data->click_down_z) )
	{
		pr_debug("aaa try to correct position\n");
		if (abs_pos->x != click_data->click_down_x || abs_pos->y != click_data->click_down_y) {
			pr_debug("aaa 1 x: %d, y: %d  -> x: %d, y: %d\n", abs_pos->x, abs_pos->y, 
			click_data->click_down_x, click_data->click_down_y);
			abs_pos->x = click_data->click_down_x;
			abs_pos->y = click_data->click_down_y;
		}
	}
	if ( click_data->click_down ==0 && click_data->click_up ==  RMI_TOUCHPAD_CLICK_JUMP_PKT_TRACK )
	{
		
		clickJump_pos_rollback(click_filter, abs_pos);
	}
	if ( click_data->click_down == 0 && click_data->click_up > 0  &&
		(abs(abs_pos->x - click_data->click_down_x) <
		RMI_TOUCHPAD_CLICK_JUMP_THRESHOLD + abs(abs_pos->z - click_data->click_down_z)) &&
		(abs(abs_pos->y - click_data->click_down_y) <
		RMI_TOUCHPAD_CLICK_JUMP_THRESHOLD + abs(abs_pos->z - click_data->click_down_z) )  )
	{
		pr_debug("aaa try to correct finger up position!!!, %d\n", click_data->click_up);
		if (abs_pos->x != click_data->click_down_x || abs_pos->y != click_data->click_down_y) {
			pr_debug("aaa 2 x: %d, y: %d  -> x: %d, y: %d\n", 
				abs_pos->x, abs_pos->y, click_data->click_down_x, click_data->click_down_y);
			abs_pos->x = click_data->click_down_x;
			abs_pos->y = click_data->click_down_y;
		}
	}
	if (click_data->click_down ==0 && click_data->click_up ==  RMI_TOUCHPAD_CLICK_JUMP_PKT_TRACK-1 ) {
		_rmi_input_report_key(click_filter->rmi_input, BTN_LEFT, 0);
	}
	if (click_data->click_down == 0 && click_data->click_up > 0 ) {
		click_data->click_up--;
	}

	return true;

}


// return value: true --- report to os
//               false -- not report
static bool rmi_filter_click_key_proc(struct rmi_filter * click_filter, unsigned int code, int value)
{
	struct rmi_filter_data_click *click_data = (struct rmi_filter_data_click*)click_filter->filter_data;

	if (code == BTN_LEFT) {
		//dev_info(rmi_input->input_dev->dev.parent, "%s\n", __func__);

		if (value) {
			++click_data->click_down;
			pr_debug("aaa button clicked %d\n", click_data->click_down);
			if (click_data->click_down <3) {
				return false;  //ignore first two click, so cursor will be move to right position
			}
		}
		else {
			pr_debug("aaa button released value: %d, \n", value);
			click_data->click_down = 0;
			click_data->click_up = RMI_TOUCHPAD_CLICK_JUMP_PKT_TRACK;
			return false;
		}
	}
	return true;
}

static bool rmi_filter_click_up_proc(struct rmi_filter * click_filter, struct rmi_input_abs_pos * abs)
{
	return true;
}


#else

static bool rmi_filter_click_up_proc(struct rmi_filter * click_filter, struct rmi_input_abs_pos * abs)
{
	struct rmi_filter_data_click *click_data = (struct rmi_filter_data_click*)click_filter->filter_data;
	u8 current_finger_state = 0;
	current_finger_state = click_data->finger_state & ~(1 << abs->n_finger);

	if(current_finger_state == 0) //last finger leave
	{
		if(click_data->click_up)
		{
			pr_debug("%s click release \n", __func__);
			click_data->click_up = 0;
			click_data->start_filter = false ;
		}
	}
	if(current_finger_state != click_data->finger_state)
	{
		click_data->finger_state = current_finger_state;
	}
	return true;
}
static bool rmi_filter_click_down_proc(struct rmi_filter * click_filter, struct rmi_input_abs_pos * abs)
{
	struct rmi_filter_data_click *click_data = (struct rmi_filter_data_click*)click_filter->filter_data;
	u8 current_finger_state = 0;
	current_finger_state = click_data->finger_state | (1 << abs->n_finger);


	if(rmi_finger_state_to_count(current_finger_state) > 1)
	{
		if(abs->n_finger != 0)
		{
			return true;
		}
	}

	if(click_data->click_down == 1 )
	{
		pr_debug("%s click down (%d,%d) %d \n", __func__, abs->x,abs->y,click_data->click_up);
		if(click_data->click_up > 0)
		{
			if((abs(abs->x - click_data->click_down_x) < RMI_TOUCHPAD_CLICK_JUMP_THRESHOLD + abs(abs->z - click_data->click_down_z)) && 
			(abs(abs->y - click_data->click_down_y) < RMI_TOUCHPAD_CLICK_JUMP_THRESHOLD + abs(abs->z - click_data->click_down_z)) ) 
			{
				abs->x = click_data->click_down_x;
				abs->y = click_data->click_down_y;
			}
			else
			{
				click_data->click_down_x = abs->x;
				click_data->click_down_y = abs->y;
				click_data->click_down_z = abs->z;
			}
		}
		else
		{
			click_data->click_down_x = abs->x;
			click_data->click_down_y = abs->y;
			click_data->click_down_z = abs->z;
		}
		click_data->start_filter = true;
	}

	if (click_data->click_down> 0 && click_data->start_filter)
	{
		if((abs(abs->x - click_data->click_down_x) < RMI_TOUCHPAD_CLICK_JUMP_THRESHOLD + abs(abs->z - click_data->click_down_z)) && 
			(abs(abs->y - click_data->click_down_y) < RMI_TOUCHPAD_CLICK_JUMP_THRESHOLD + abs(abs->z - click_data->click_down_z)) ) 
		{
			abs->x=click_data->click_down_x;
			abs->y=click_data->click_down_y;
		}
		else
		{
			click_data->start_filter = false;
			pr_debug("%s: diff(%ld,%ld) threshold=%ld\n", __func__, abs(abs->x - click_data->click_down_x),abs(abs->y - click_data->click_down_y),RMI_TOUCHPAD_CLICK_JUMP_THRESHOLD + abs(abs->z - click_data->click_down_z));
		}
		click_data->click_down ++;
	}
	if (click_data->click_down == 0 && click_data->click_up > 0 && click_data->start_filter)
	{
		if((abs(abs->x - click_data->click_down_x) < RMI_TOUCHPAD_CLICK_JUMP_THRESHOLD + abs(abs->z - click_data->click_down_z)) && 
			(abs(abs->y - click_data->click_down_y) < RMI_TOUCHPAD_CLICK_JUMP_THRESHOLD + abs(abs->z - click_data->click_down_z)) ) 
		{
			abs->x=click_data->click_down_x;
			abs->y=click_data->click_down_y;
			click_data->click_up --;
		}
		else
		{
			click_data->start_filter = false ;
			pr_debug("%s: diff2(%ld,%ld) threshold=%ld\n", __func__, abs(abs->x - click_data->click_down_x),abs(abs->y - click_data->click_down_y),RMI_TOUCHPAD_CLICK_JUMP_THRESHOLD + abs(abs->z - click_data->click_down_z));
		}
	}
	if(current_finger_state != click_data->finger_state)
	{
		click_data->finger_state = current_finger_state;
	}
	return true;
}

// return value: true --- report to os
//               false -- not report
static bool rmi_filter_click_key_proc(struct rmi_filter * click_filter, unsigned int code, int value)
{
	struct rmi_filter_data_click *click_data = (struct rmi_filter_data_click*)click_filter->filter_data;
	if (code == BTN_LEFT) {

		if (value) {
			++click_data->click_down;
			pr_debug("aaa button clicked %d\n", click_data->click_down);
			/* remove click delay due to some touchpad won't send continuous interrupt
			if (tp->clickfilter->click_down <3) {
				return false;  //ignore first two click, so cursor will be move to right position
			}*/
		}
		else {
			pr_debug("aaa button released \n");
			click_data->click_down = 0;
			click_data->click_up = RMI_TOUCHPAD_CLICK_JUMP_PKT_TRACK;
		}
	}
	return true;
}

#endif
void rmi_click_filter_create(struct rmi_input_dev *rmi_input)
{
	struct rmi_filter *click_filter;
	struct device *dev = rmi_input->input_dev->dev.parent;
	struct rmi_touchpad * tp =
			(struct rmi_touchpad *)rmi_input->context;
	struct rmi_filter_data_click *click_data;

	click_filter = devm_kzalloc(dev, sizeof(struct rmi_filter), GFP_KERNEL);
	if (!click_filter)
		return;
	click_filter->rmi_input = rmi_input;
	click_filter->filter_data = devm_kzalloc(dev, sizeof(struct rmi_filter_data_click), GFP_KERNEL);
	if(!click_filter->filter_data)
		return;
	click_data = (struct rmi_filter_data_click *)click_filter->filter_data;
	list_add_tail(&click_filter->node, &tp->filter_list);
	click_filter->down_proc= rmi_filter_click_down_proc;
	click_filter->up_proc= rmi_filter_click_up_proc;
	click_filter->key_proc= rmi_filter_click_key_proc;
	return;

}


