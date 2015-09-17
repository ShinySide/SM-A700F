/*
 * Copyright (c) 2013 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _RMI_INPUT_DEV_H
#define _RMI_INPUT_DEV_H

#include <linux/input.h>
#include <linux/input/mt.h>


#define NAME_BUFFER_SIZE 256

enum rmi_input_finger_state {
        RMI_INPUT_NO_FINGER		= 0x00,
        RMI_INPUT_FINGER_PRESENT	= 0x01,
};

struct rmi_input_res_pos {
	s8 x;
	s8 y;
};

struct rmi_input_abs_pos {
	u8 n_finger;
	u16 x;
	u16 y;
	u8 z;
	u16 w_x;
	u16 w_y;
	u16 w_min;
	u16 w_max;
	u8 orient;
	unsigned int tool_type;

	u16 dx;
	u16 dy;
	unsigned int key_code;
	int key_value;

	unsigned long time_stamp;
	u8 type;
};

struct rmi_input_dev {
	struct input_dev *input_dev;
	char input_phys[NAME_BUFFER_SIZE];

	void (*report_key)(struct rmi_input_dev *dev, unsigned int code, int value);
	void (*report_rel)(struct rmi_input_dev *dev, struct rmi_input_res_pos * rel);
	void (*report_abs)(struct rmi_input_dev *dev, struct rmi_input_abs_pos * abs);
	void (*report_state)(struct rmi_input_dev *rmi_input, 
                enum rmi_input_finger_state state, struct rmi_input_abs_pos *abs);

	void * context;
};

struct rmi_input_dev *rmi_input_dev_alloc(struct device *dev, void * data,
						const char * phys_fmt,
						void *input_drvdata);

int rmi_input_dev_register(struct rmi_input_dev *rmi_input);

static inline void rmi_input_dev_free(struct rmi_input_dev *rmi_input)
{
	input_free_device(rmi_input->input_dev);
	rmi_input->input_dev = NULL;
}

static inline void rmi_input_dev_unregister(struct rmi_input_dev *rmi_input)
{
	input_unregister_device(rmi_input->input_dev);
	rmi_input->input_dev = NULL;
}

static inline void rmi_input_report_key(struct rmi_input_dev *rmi_input,
					unsigned int code, int value)
{
	rmi_input->report_key(rmi_input, code, value);
}

static inline void rmi_input_report_rel(struct rmi_input_dev *rmi_input,
					struct rmi_input_res_pos * rel)
{
	rmi_input->report_rel(rmi_input, rel);
}

static inline void rmi_input_report_abs(struct rmi_input_dev *rmi_input,
					struct rmi_input_abs_pos * abs)
{
	rmi_input->report_abs(rmi_input, abs);
}

static inline void rmi_input_report_state(struct rmi_input_dev *rmi_input,
						enum rmi_input_finger_state state,
						struct rmi_input_abs_pos *abs)
{
        if (rmi_input->report_state)
                rmi_input->report_state(rmi_input, state, abs);
}

static inline void _rmi_input_report_key(struct rmi_input_dev *rmi_input,
                                                unsigned int code, int value)
{
        input_report_key(rmi_input->input_dev, code, value);
}

static inline void _rmi_input_report_rel(struct rmi_input_dev *rmi_input,
                                                struct rmi_input_res_pos * rel)
{
        input_report_rel(rmi_input->input_dev, REL_X, rel->x);
        input_report_rel(rmi_input->input_dev, REL_Y, rel->y);
}

static inline void _rmi_input_report_abs(struct rmi_input_dev *rmi_input,
                                                struct rmi_input_abs_pos * abs)
{
	input_mt_slot(rmi_input->input_dev, abs->n_finger);
	if (abs->z) 
		input_mt_report_slot_state(rmi_input->input_dev, abs->tool_type, RMI_INPUT_FINGER_PRESENT);
	else 
		input_mt_report_slot_state(rmi_input->input_dev, abs->tool_type, RMI_INPUT_NO_FINGER);
	input_report_abs(rmi_input->input_dev, ABS_MT_PRESSURE, abs->z);
	input_report_abs(rmi_input->input_dev, ABS_MT_TOUCH_MAJOR, abs->w_max);
	input_report_abs(rmi_input->input_dev, ABS_MT_TOUCH_MINOR, abs->w_min);
	input_report_abs(rmi_input->input_dev, ABS_MT_ORIENTATION, abs->orient);
	input_report_abs(rmi_input->input_dev, ABS_MT_POSITION_X, abs->x);
	input_report_abs(rmi_input->input_dev, ABS_MT_POSITION_Y, abs->y);
}

static inline void _rmi_input_report_state(struct rmi_input_dev *rmi_input, 
                                                enum rmi_input_finger_state state, 
                                                struct rmi_input_abs_pos *abs)
{
	input_mt_slot(rmi_input->input_dev, abs->n_finger);
	input_mt_report_slot_state(rmi_input->input_dev, abs->tool_type, state);
}

#endif
