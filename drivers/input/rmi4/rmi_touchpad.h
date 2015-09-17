/*
 * Copyright (c) 2013 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _RMI_TOUCHPAD_H
#define _RMI_TOUCHPAD_H


#include "rmi_input_dev.h"
#include "rmi_driver.h"


enum rmi_filter_state {
	rmi_state_idle = 0,
	rmi_state_prepare,
	rmi_state_faking,
	rmi_state_active,
	rmi_state_active_zone_2nd_finger
};

#include "rmi_filter_tap.h"
#include "rmi_filter_click.h"
#include "rmi_filter_zone.h"
#include "rmi_filter_palm.h"



#define RMI_TOUCHPAD_FINGER_QUEUE_SIZE		32
#define RMI_TOUCHPAD_SUPPRESS_HIGHW_PKTS	8
#ifdef CLICK_FILTER2
#define RMI_TOUCHPAD_CLICK_JUMP_THRESHOLD	34
#define RMI_TOUCHPAD_CLICK_JUMP_PKT_TRACK	10
#else
#define RMI_TOUCHPAD_CLICK_JUMP_THRESHOLD	250
#define RMI_TOUCHPAD_CLICK_JUMP_PKT_TRACK	30
#endif
#define PACKET_SEARCH_LENGTH			20
#define PACKET_WINDOW_SIZE			7


struct rmi_pos_queue{
	int queue_head;
	int queue_tail;
	struct rmi_input_abs_pos * pos_data;
};
struct rmi_touchpad_finger {

	struct rmi_pos_queue finger_fifo;
	struct rmi_pos_queue finger_buffer;

	enum rmi_input_finger_state finger_state;
	int suppress_count;
	int reporting;
	int highw_violation;

	struct rmi_input_abs_pos finger_down_pkt;
	int tap_status;
	unsigned long last_palm_time_stamp;
};

typedef struct rmi_filter {
	struct rmi_input_dev *rmi_input;
	bool finger_down;
	bool finger_up;
	int init_x;
	int init_y;
	struct timeval timeval_down;
	bool start_filter;
	bool (*pre_down_proc)(struct rmi_filter * , struct rmi_input_abs_pos * abs);
	bool (*down_proc)(struct rmi_filter * , struct rmi_input_abs_pos * abs);
	bool (*up_proc)(struct rmi_filter * , struct rmi_input_abs_pos * abs);
	bool (*key_proc)(struct rmi_filter *, unsigned int code, int value);
	struct list_head node;
	void * filter_data;
	
}rmi_filter;


struct rmi_touchpad {
	struct rmi_touchpad_finger *fingers;
	struct rmi_pos_queue key_fifo;
	int finger_count;
	u8 suppress_finger;
	u8 suppress_click;
	u8 suppress_highw;
	u8 highw_threshold;
	int highw_count;
	struct rmi_filter_click *clickfilter;
	//struct rmi_filter_tap *tapfilter;
	struct list_head filter_list;
	int x_max;
	int y_max;
	unsigned int packet_delay_count;
	bool has_dribble;
};


int rmi_touchpad_create(struct rmi_input_dev *rmi_input, int finger_count, int x_max, int y_max);
int rmi_touchpad_queue_peek(struct rmi_pos_queue * pos_queue,
				struct rmi_input_abs_pos * abs_pos, int i);
unsigned int time_diff_ms(unsigned long big, unsigned long small) ;
void rmi_touchpad_inject(struct rmi_input_dev *rmi_input,struct rmi_input_abs_pos *abs);
u8 rmi_finger_state_to_count(u8 x);
void rmi_touchpad_reset_finger(struct rmi_input_dev *rmi_input,struct rmi_input_abs_pos *abs);
void rmi_tp_control_init(struct rmi_input_dev *rmi_input);

#endif
