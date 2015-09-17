/*
 * Copyright (c) 2012, 2013 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#define FUNCTION_NUMBER 0x12

#include <linux/kernel.h>
#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/flex_array.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/rmi.h>
#include <linux/version.h>
#include "rmi_driver.h"
#include "rmi_f12.h"
#include "rmi_touchpad.h"

/**
 * Conveniently returns true if the register is present.
 */
static bool rmi_has_register(struct rmi_reg_descriptor *desc, int reg)
{
	return test_bit(reg, desc->presence);
}

/**
 * Conveniently returns the number of bytes in a given register.
 */
static int rmi_register_size(struct rmi_reg_descriptor *desc, int reg)
{
	struct rmi_register_desc *rdesc;

	rdesc = flex_array_get(desc->structure, reg);
	if (rdesc)
		return rdesc->size;
	return 0;
}

/**
 * Conveniently returns the offset of a given register.
 */
static int rmi_register_offset(struct rmi_reg_descriptor *desc, int reg)
{
	struct rmi_register_desc *rdesc;

	rdesc = flex_array_get(desc->structure, reg);
	if (rdesc)
		return rdesc->offset;
	return 0;
}

/**
 * Conveniently returns the number of defined subpackets in a given register.
 * Note that this is different than nr_subpackets, which is the maximum number
 * of subpackets that could be defined in the descriptor.
 */
static int rmi_register_subpackets(struct rmi_reg_descriptor *desc, int reg)
{
	struct rmi_register_desc *rdesc;

	rdesc = flex_array_get(desc->structure, reg);
	if (rdesc)
		return bitmap_weight(rdesc->subpackets, rdesc->nr_subpackets);
	return 0;
}

/**
 * Conveniently returns true if the specified register contains the desired
 * subpacket.
 */
static bool rmi_register_has_subpacket(struct rmi_reg_descriptor *desc, int reg, int sp)
{
	struct rmi_register_desc *rdesc;

	rdesc = flex_array_get(desc->structure, reg);
	if (rdesc && sp < rdesc->nr_subpackets)
		return test_bit(sp, rdesc->subpackets);
	return false;
}

/**
 * Reads and parses the register descriptor.
 *
 * @fn - the function device we're working with
 * @desc - will be filled in with the parsed data
 * @address - the starting address for the register descriptor
 */
static int rmi_read_reg_descriptor(struct rmi_function *fn,
				   struct rmi_reg_descriptor *desc,
				   u16 address)
{
	struct rmi_device *rmi_dev = fn->rmi_dev;
	int retval;
	u8 *buf;
	int i, j;
	int offset;
	int reg_offset = 0;
	int nr_regs;
	int reg;
	int bitpos;
	int lastbit;
	struct rmi_register_desc reg_desc;

	retval = rmi_read(rmi_dev, address, &desc->presence_size);
	if (retval < 0)
		return retval;
	address++;

	buf = devm_kzalloc(&fn->dev, desc->presence_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	retval = rmi_read_block(rmi_dev, address, buf, desc->presence_size);
	if (retval < 0)
		goto exit;
	address++;

	desc->structure_size = buf[0];
	offset = 1;

        if (desc->structure_size == 0) {
		desc->structure_size = buf[1] | (buf[2] << 8);
		offset += 2;
	}
	nr_regs = desc->presence_size - offset;
	desc->presence_bits = nr_regs * 8;
	desc->presence = devm_kzalloc(&fn->dev,
			BITS_TO_LONGS(desc->presence_bits)*sizeof(unsigned long), GFP_KERNEL);
	if (!desc->presence) {
		retval = -ENOMEM;
		goto exit;
	}

	bitpos = 0;
	for (j = 0; j < nr_regs; j++) {
		for (i = 0; i < 8; i++) {
			if (buf[offset] & (1 << i)) {
				set_bit(bitpos, desc->presence);
				lastbit = bitpos;
			}
			bitpos++;
		}
		offset++;
	}

	devm_kfree(&fn->dev, buf);
	buf = devm_kzalloc(&fn->dev, desc->structure_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	desc->structure = flex_array_alloc(sizeof(struct rmi_register_desc),
					   desc->presence_bits, GFP_KERNEL);
	if (!desc->structure) {
		retval = -ENOMEM;
		goto exit;
	}

	retval = rmi_read_block(rmi_dev, address, buf, desc->structure_size);
	if (retval < 0)
		goto exit;

	reg = find_first_bit(desc->presence, desc->presence_bits);
	offset = 0;
	while (reg < desc->presence_bits) {
		bool done = false;
		int base;

		reg_desc.nr_subpackets = 0;
		reg_desc.size = buf[offset];
		reg_desc.offset = reg_offset;
		reg_offset += reg_desc.size;
		offset++;
		if (reg_desc.size == 0) {
			reg_desc.size = buf[offset] | (buf[offset+1] << 8);
			offset += 2;
		}
		if (reg_desc.size == 0) {
			reg_desc.size = buf[offset] | (buf[offset+1] << 8) |
				(buf[offset+2] << 16) | (buf[offset+3] << 24);
			offset += 4;
		}
		base = offset;
		while (!done) {
			reg_desc.nr_subpackets += 7;
			done = (buf[offset] & 0x80) == 0;
			offset++;
		}
		reg_desc.subpackets = devm_kzalloc(&fn->dev,
			BITS_TO_LONGS(reg_desc.nr_subpackets)*sizeof(unsigned long), GFP_KERNEL);
		if (!reg_desc.subpackets) {
			retval = -ENOMEM;
			goto exit;
		}
		offset = base;
		bitpos = 0;
		done = false;
		while (!done) {
			for (i = 0; i < 7; i++) {
				if (buf[offset] & (0x01 << i)) {
					set_bit(bitpos, reg_desc.subpackets);
				}
				bitpos++;
			}
			done = (buf[offset] & 0x80) == 0;
			offset++;
		}
		retval = flex_array_put(desc->structure, reg, &reg_desc, GFP_KERNEL);
		if (retval)
			dev_warn(&fn->dev, "Failed to put reg %d structure info. Code: %d.\n", reg, retval);
		reg = find_next_bit(desc->presence, desc->presence_bits, reg+1);
	}

	retval = 0;

exit:
	devm_kfree(&fn->dev, buf);
	return retval;
}

/**
 * Reads and parses a set of register descriptors.
 */
static int rmi_read_descriptors(struct rmi_function *fn,
				struct rmi_descriptors *desc,
				u16 address) {
	int retval;

	retval = rmi_read_reg_descriptor(fn, &desc->query, address);
	if (retval) {
		dev_err(&fn->dev, "Failed to read query descriptor, code %d\n",
			retval);
		return retval;
	}
	address += 3;

	retval = rmi_read_reg_descriptor(fn, &desc->control, address);
	if (retval) {
		dev_err(&fn->dev, "Failed to read control descriptor, code %d\n",
			retval);
		return retval;
	}
	address += 3;

	retval = rmi_read_reg_descriptor(fn, &desc->data, address);
	if (retval)
		dev_err(&fn->dev, "Failed to read data descriptor, code %d\n",
			retval);

	return retval;
}

/**
 * Read the Sensor Tuning control register and set operating parameters
 * appropriately.
 */
static void read_sensor_tuning(struct rmi_function *fn) {
	struct f12_data *f12 = fn->data;
	int buffer_size;
	int offset = 0;
	int retval;

	f12->x_max = f12->y_max = DEFAULT_XY_MAX;

	buffer_size = rmi_register_size(&f12->desc.control, F12_SENSOR_TUNING_REG);
	if (!buffer_size)
		return;

	f12->sensor_tuning.buffer = devm_kzalloc(&fn->dev, buffer_size, GFP_KERNEL);
	if (!f12->sensor_tuning.buffer)
		return;

	retval = rmi_read_block(fn->rmi_dev, fn->fd.control_base_addr, f12->sensor_tuning.buffer, buffer_size);
	if (!retval) {
		dev_warn(&fn->dev, "WARNING: Failed to read sensor tuning.  Using default values.\n");
		return;
	}

	if (rmi_register_has_subpacket(&f12->desc.control, F12_SENSOR_TUNING_REG, F12_COORD_MAX_SP)) {
		f12->sensor_tuning.x_max_le = (u16*) &f12->sensor_tuning.buffer[offset];
		offset += 2;
		f12->sensor_tuning.y_max_le = (u16*) &f12->sensor_tuning.buffer[offset];
		offset += 2;
		f12->x_max = le16_to_cpu(*f12->sensor_tuning.x_max_le);
		f12->y_max = le16_to_cpu(*f12->sensor_tuning.y_max_le);
	}
}

/**
 * enable dribble to push the queued data out
 * 
 */

static void enable_dribble(struct rmi_function *fn) {
	struct f12_data *f12 = fn->data;
	int buffer_size;
	int retval;
	int i;
	int offset = 0;

	if (!rmi_has_register(&f12->desc.control, F12_REPORTING_CONTROL_REG)) {
		dev_warn(&fn->dev, "WARNING: no reporting control reg exists  Using default values.\n");
		return;
	}

	buffer_size = rmi_register_size(&f12->desc.control, F12_REPORTING_CONTROL_REG);
	if (!buffer_size)
		return;
	for(i = 0; i < F12_REPORTING_CONTROL_REG ; i++) {
		if(rmi_has_register(&f12->desc.control, i))
			offset++;
	}
	f12->reporting_control.buffer = devm_kzalloc(&fn->dev, buffer_size, GFP_KERNEL);
	if (!f12->reporting_control.buffer)
		return;

	retval = rmi_read_block(fn->rmi_dev, fn->fd.control_base_addr + offset, f12->reporting_control.buffer, buffer_size);
	if (!retval) {
		dev_warn(&fn->dev, "WARNING: Failed to read reporting control.  Using default values.\n");
		return;
	}

	f12->reporting_control.buffer[2] |= 0x04;
	retval = rmi_write_block(fn->rmi_dev, fn->fd.control_base_addr + offset,f12->reporting_control.buffer, buffer_size);
	if (!retval) {
		dev_warn(&fn->dev, "WARNING: Failed to write reporting control.  Using default values.\n");
		return;
	}
	dev_info(&fn->dev, "dribble enabled\n");

}


static int get_tool_type(struct rmi_f12_object_data *object)
{
	if (object->type == F12_OBJECT_STYLUS)
		return MT_TOOL_PEN;
	return MT_TOOL_FINGER;
}

static void report_one_object(struct f12_data *f12, struct rmi_f12_object_data *object, int slot)
{
	struct input_dev *input = f12->rmi_input->input_dev;
	struct rmi_input_abs_pos abs;
	//if (object->type == F12_OBJECT_PALM && f12->has_ACM25) return;

	memset(&abs, 0, sizeof(struct rmi_input_abs_pos));
	abs.type = object->type;
	abs.n_finger = slot;
	abs.tool_type = get_tool_type(object);
        
	if (object->type) {
		abs.w_x = object->wx;
		abs.w_y = object->wy;
		abs.w_min = abs.w_x;
                abs.w_max = abs.w_y;
                abs.y = le16_to_cpu(object->pos_y);
		abs.x = le16_to_cpu(object->pos_x);
		abs.z = object->z;
		abs.orient = abs.w_x > abs.w_y;
		
		/* flip y! shoudl really have some axis swapping setting like f11 */
		abs.y = max(f12->y_max - abs.y, 0);

                if (object->wx > object->wy) {
                        abs.w_min = object->wy;
                        abs.w_max = object->wx;
                }
		rmi_input_report_abs(f12->rmi_input, &abs);
		if (f12->sensor_type == rmi_sensor_touchpad)
			input_mt_report_pointer_emulation(input, true);

#if 0
		pr_debug(
			"finger[%d]:%d - x:%d y:%d z:%d wx:%d wy:%d\n",
                        slot, object->type, abs.x, abs.y, abs.z, abs.w_x, abs.w_y);
#endif
        } else {
                rmi_input_report_state(f12->rmi_input, RMI_INPUT_NO_FINGER, &abs);
        }
        
}


static void report_objects(struct f12_data *f12)
{
	int i;
	int object_count = 0;

	if (f12->suppress)
		return;
	for (i = 0; i < f12->max_objects; i++) {
		struct rmi_f12_object_data *object = (struct rmi_f12_object_data *)
							&f12->object_buf[i * 8];

		//u16 w_max = object->wx > object->wy ? object->wy : object->wx;
		u16 w_max = int_sqrt(object->wx * object->wx +  object->wy * object->wy);

		if (f12->suppress_highw > 0 && f12->suppress_highw <= w_max)
		{
			dev_info(f12->rmi_input->input_dev->dev.parent, "Suppressing finger (%d)  because of high W (%d)\n",i, w_max);
			return;
		}
	}

	for (i = 0; i < f12->max_objects; i++) {
		struct rmi_f12_object_data *object = (struct rmi_f12_object_data *)
							&f12->object_buf[i * 8];
		if (object->type)
			object_count++;
		report_one_object(f12, object, i);
	}
	// TODO update lines for newer kernel
	rmi_input_report_key(f12->rmi_input, BTN_TOUCH, object_count);
	input_sync(f12->rmi_input->input_dev);
}


static int rmi_f12_attention(struct rmi_function *fn,
			     unsigned long *irq_nr_regs)
{
	int retval;
	struct f12_data *f12 = fn->data;
	struct rmi_device * rmi_dev = fn->rmi_dev;

	if (rmi_dev->xport->attn_data) {
		memcpy(f12->object_buf, rmi_dev->xport->attn_data, f12->buf_size);
		rmi_dev->xport->attn_data += f12->buf_size;
		rmi_dev->xport->attn_size -= f12->buf_size;
	} else {
		retval = rmi_read_block(rmi_dev, f12->object_address,
					f12->object_buf, f12->buf_size);
		if (retval < 0) {
			dev_err(&fn->dev, "Failed to read object data. Code: %d.\n",
				retval);
			return retval;
		}
	}

	report_objects(f12);
	return 0;
}

static int rmi_f12_remove(struct rmi_function *fn)
{
	struct f12_data *f12 = fn->data;
	struct rmi_device_platform_data *pdata = to_rmi_platform_data(fn->rmi_dev);

	if (!pdata->unified_input_device)
		rmi_input_dev_unregister(f12->rmi_input);
	devm_kfree(&fn->dev, f12->object_buf);
	devm_kfree(&fn->dev, f12);
	fn->data = NULL;

	return 0;
}

static int rmi_f12_probe(struct rmi_function *fn)
{
	struct f12_data *f12;
	int retval;
	struct input_dev *input_dev;
	struct rmi_device *rmi_dev = fn->rmi_dev;
	struct rmi_driver_data *driver_data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_device_platform_data *pdata = to_rmi_platform_data(rmi_dev);
	unsigned long input_flags;
	int res_x, res_y;

	f12 = devm_kzalloc(&fn->dev, sizeof(struct f12_data), GFP_KERNEL);
	if (!f12)
		return -ENOMEM;

	fn->data = f12;

	retval = rmi_read_block(fn->rmi_dev, fn->fd.query_base_addr,
		&f12->info, sizeof(f12->info));
	if (retval < 0) {
		dev_err(&fn->dev, "Failed to read general info register, code %d\n",
			retval);
		goto error_free_data;
	}
	if (!f12->info.has_register_descriptors) {
		dev_err(&fn->dev, "Behavior of F12 without register descriptors is undefined.\n");
		retval = -ENODEV;
		goto error_free_data;
	}

	retval = rmi_read_descriptors(fn, &f12->desc,
				      fn->fd.query_base_addr + 1);
	if (retval)
		return retval;

	if (!rmi_has_register(&f12->desc.data, F12_FINGER_DATA_REG)) {
		dev_err(&fn->dev, "Finger data registers are missing!\n");
		retval = -ENODEV;
		goto error_free_data;
	}

	f12->buf_size = rmi_register_size(&f12->desc.data, F12_FINGER_DATA_REG);
	f12->object_buf = devm_kzalloc(&fn->dev, f12->buf_size * sizeof(u8), GFP_KERNEL);
	if (!f12->object_buf) {
		retval = -ENOMEM;
		goto error_free_data;
	}
	f12->max_objects = rmi_register_subpackets(&f12->desc.data, F12_FINGER_DATA_REG);
	f12->object_address = fn->fd.data_base_addr + rmi_register_offset(&f12->desc.data, F12_FINGER_DATA_REG);
	f12->has_ACM25 = false;
        if (rmi_has_register(&f12->desc.data, F12_FINGER_ACM_REG)) {
		f12->has_ACM25 = true;
	}
#if 1
	if (f12->has_ACM25) {
		pr_debug("acm25 is on\n");
	}
	else {
		pr_debug("acm25 is off\n");
	}
#endif
		
	read_sensor_tuning(fn);

	if (f12->info.has_dribble) {
		enable_dribble(fn);
	} else {
		dev_err(&fn->dev, "No dribble support!\n");
	}


	f12->sensor_type = pdata->f12_sensor_data->sensor_type;

	if (pdata->unified_input_device) {
		f12->rmi_input = driver_data->rmi_input;
		input_dev = f12->rmi_input->input_dev;
		rmi_touchpad_create(f12->rmi_input, f12->max_objects, f12->x_max, f12->y_max);
	} else {
		f12->rmi_input = rmi_input_dev_alloc(&rmi_dev->dev, driver_data,
							"%s/input0", f12);
		if (f12->rmi_input) {
			dev_err(&fn->dev, "Error in setting input device.\n");
			goto error_free_data;
		}
		input_dev = f12->rmi_input->input_dev;
	}

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

	if (f12->sensor_type == rmi_sensor_touchpad)
		input_flags = INPUT_PROP_POINTER;
	else
		input_flags = INPUT_PROP_DIRECT;
	set_bit(input_flags, input_dev->propbit);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0))
	input_mt_init_slots(input_dev, f12->max_objects, 0);
#else
	input_mt_init_slots(input_dev, f12->max_objects);
#endif
	input_set_abs_params(input_dev, ABS_X, 0, f12->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, f12->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, DEFAULT_MAX_ABS_MT_PRESSURE, 0, 0);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			0, f12->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			0, f12->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0,
			DEFAULT_MAX_ABS_MT_PRESSURE, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			0, DEFAULT_MAX_ABS_MT_TOUCH, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR,
			0, DEFAULT_MAX_ABS_MT_TOUCH, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
			0, DEFAULT_MAX_ABS_MT_ORIENTATION, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,
			DEFAULT_MIN_ABS_MT_TRACKING_ID,
			f12->max_objects, 0, 0);

	if (pdata->f12_sensor_data->x_mm && pdata->f12_sensor_data->y_mm) {
		res_x = (f12->x_max - 0) / pdata->f12_sensor_data->x_mm;
		res_y = (f12->y_max - 0) / pdata->f12_sensor_data->y_mm;

		input_abs_set_res(input_dev, ABS_X, res_x);
		input_abs_set_res(input_dev, ABS_Y, res_y);

		input_abs_set_res(input_dev, ABS_MT_POSITION_X, res_x);
		input_abs_set_res(input_dev, ABS_MT_POSITION_Y, res_y);
	}

	//input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE,
	//			0, MT_TOOL_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE,
				0, MT_TOOL_FINGER, 0, 0);

	if (f12->sensor_type == rmi_sensor_touchpad) {
		set_bit(EV_KEY, input_dev->evbit);
		set_bit(BTN_LEFT, input_dev->keybit);

		set_bit(BTN_TOOL_FINGER, input_dev->keybit);
		set_bit(BTN_TOOL_DOUBLETAP, input_dev->keybit);
		set_bit(BTN_TOOL_TRIPLETAP, input_dev->keybit);
		set_bit(BTN_TOOL_QUADTAP, input_dev->keybit);
		set_bit(BTN_TOOL_QUINTTAP, input_dev->keybit);
	}

	if (!pdata->unified_input_device) {
		retval = rmi_input_dev_register(f12->rmi_input);
		if (retval < 0)
			goto error_free_dev;
	}

	return 0;

error_free_dev:
	if (!pdata->unified_input_device)
		rmi_input_dev_free(f12->rmi_input);

error_free_data:
	devm_kfree(&fn->dev, f12->object_buf);
	devm_kfree(&fn->dev, f12);
	return retval;
}

static struct rmi_function_driver function_driver = {
	.driver = {
		.name = "rmi_f12",
	},
	.func = FUNCTION_NUMBER,
	.probe = rmi_f12_probe,
	.remove = rmi_f12_remove,
	.attention = rmi_f12_attention,
};

module_rmi_function_driver(function_driver);

MODULE_AUTHOR("Christopher Heiny <cheiny@synaptics.com");
MODULE_DESCRIPTION("F12 2D pointing");
MODULE_LICENSE("GPL");
MODULE_VERSION(RMI_DRIVER_VERSION);
