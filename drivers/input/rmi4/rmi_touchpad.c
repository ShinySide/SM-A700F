/*
 * Copyright (c) 2013 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

/* Basic palm suppression and filtering for TouchPads and ClickPads */

#include <linux/circ_buf.h>
#include <linux/kernel.h>
#include "rmi_touchpad.h"
unsigned int time_diff_ms(unsigned long big, unsigned long small)
{
	unsigned long t1;
	if (big < small) {
		t1 = ULONG_MAX - small + big;
	}
	else {
		t1 = big - small;
	}
	return t1  * 1000 / HZ;
}
EXPORT_SYMBOL_GPL(time_diff_ms);

static inline int rmi_touchpad_touched_count(struct rmi_touchpad *tp)
{
	int i,finger_touched_count = 0;
	for(i = 0; i < tp->finger_count ; i++) {
		if(tp->fingers[i].finger_state == RMI_INPUT_FINGER_PRESENT)
			finger_touched_count ++;
	}
	return finger_touched_count;
}
static inline int rmi_touchpad_reported_count(struct rmi_touchpad *tp)
{
	int i,finger_reported_count = 0;
	for(i = 0; i < tp->finger_count ; i++) {
		if(tp->fingers[i].reporting)
			finger_reported_count ++;
	}
	return finger_reported_count;
}

static void rmi_touchpad_queue_clear(struct rmi_pos_queue * pos_queue)
{
	smp_mb();
	pos_queue->queue_head = 0;
	pos_queue->queue_tail = 0;
}

static void rmi_touchpad_clear_finger(struct rmi_touchpad_finger * finger)
{
	finger->reporting = 0;
	finger->suppress_count = 0;
	finger->highw_violation = 0;
	finger->finger_state = RMI_INPUT_NO_FINGER;
}

static int rmi_touchpad_queue_count(struct rmi_pos_queue * pos_queue)
{
	int head = ACCESS_ONCE(pos_queue->queue_head);
	int tail = ACCESS_ONCE(pos_queue->queue_tail);
	return CIRC_CNT(head, tail, RMI_TOUCHPAD_FINGER_QUEUE_SIZE);
}

static void rmi_touchpad_queue_add(struct rmi_pos_queue * pos_queue,
					struct rmi_input_abs_pos * abs)
{
	struct rmi_input_abs_pos *queue_abs;
	int head = pos_queue->queue_head;
	int tail = pos_queue->queue_tail;
	bool is_queue_full =  (rmi_touchpad_queue_count(pos_queue) + 1) ==
				RMI_TOUCHPAD_FINGER_QUEUE_SIZE;
	if (!pos_queue->pos_data)
		panic("Queue is NULL for finger (%d)\n", abs->n_finger);

	queue_abs = (struct rmi_input_abs_pos *)(((u8*)pos_queue->pos_data)
		+ sizeof(struct rmi_input_abs_pos) * head);
#if 1
	memcpy(queue_abs, abs, sizeof(struct rmi_input_abs_pos));
#endif

	smp_wmb();
	if (is_queue_full) {
	        pos_queue->queue_tail = ((tail + 1)
                        & (RMI_TOUCHPAD_FINGER_QUEUE_SIZE - 1));
	}
	pos_queue->queue_head = ((head + 1)
			& (RMI_TOUCHPAD_FINGER_QUEUE_SIZE - 1));
	
	//pr_info("%s: new head (%d)\n", __func__, finger->finger_input_queue_head);
}

int rmi_touchpad_queue_get(struct rmi_pos_queue * pos_queue,
				struct rmi_input_abs_pos * abs)
{
	struct rmi_input_abs_pos *queue_abs;
	int head = ACCESS_ONCE(pos_queue->queue_head);
	int tail = pos_queue->queue_tail;

	if (CIRC_CNT(head, tail, RMI_TOUCHPAD_FINGER_QUEUE_SIZE)) {
		queue_abs = (struct rmi_input_abs_pos *)(((u8*)pos_queue->pos_data)
			+ sizeof(struct rmi_input_abs_pos) * tail);
		memcpy(abs, queue_abs, sizeof(struct rmi_input_abs_pos));

		smp_mb();
		pos_queue->queue_tail = ((tail + 1)
			& (RMI_TOUCHPAD_FINGER_QUEUE_SIZE - 1));

		return 1;
	}

	return 0;
}

int rmi_touchpad_queue_check(struct rmi_pos_queue * pos_queue,
				struct rmi_input_abs_pos * abs)
{
	struct rmi_input_abs_pos *queue_abs;
	int head = ACCESS_ONCE(pos_queue->queue_head);
	int tail = pos_queue->queue_tail;

	if (CIRC_CNT(head, tail, RMI_TOUCHPAD_FINGER_QUEUE_SIZE)) {
		queue_abs = (struct rmi_input_abs_pos *)(((u8*)pos_queue->pos_data)
			+ sizeof(struct rmi_input_abs_pos) * tail);
		memcpy(abs, queue_abs, sizeof(struct rmi_input_abs_pos));
		return 1;
	}

	return 0;
}

int rmi_touchpad_queue_peek(struct rmi_pos_queue * pos_queue,
				struct rmi_input_abs_pos * abs_pos, int i)
{
	struct rmi_input_abs_pos *queue_abs;
	int head = ACCESS_ONCE(pos_queue->queue_head);
	int tail = pos_queue->queue_tail;
	int index = (head -1 ) & (RMI_TOUCHPAD_FINGER_QUEUE_SIZE - 1);

	if (CIRC_CNT(head, tail, RMI_TOUCHPAD_FINGER_QUEUE_SIZE) >= i+1) {
		queue_abs = (struct rmi_input_abs_pos *)(((u8*)pos_queue->pos_data)
			+ sizeof(struct rmi_input_abs_pos) * index);
		memcpy(abs_pos, queue_abs, sizeof(struct rmi_input_abs_pos));

		return 1;
	}

	return 0;
}

EXPORT_SYMBOL_GPL(rmi_touchpad_queue_peek);

static void rmi_touchpad_report_key(struct rmi_input_dev *rmi_input,
					unsigned int code,
					int value)
{
	struct rmi_touchpad * tp =
			(struct rmi_touchpad *)rmi_input->context;
	bool report_to_os = true;

	if (code == BTN_LEFT) {
		struct rmi_input_abs_pos key_data;
		memset(&key_data, 0, sizeof(struct rmi_input_abs_pos));
		if (tp->suppress_click/*|| tp->suppress_highw*/)
		{
			return;
		}
		if(!list_empty(&tp->filter_list))
		{
			struct rmi_filter *entry;
			list_for_each_entry(entry, &tp->filter_list, node)
			{
				if(entry->key_proc)
					report_to_os = entry->key_proc(entry, code, value);
			}
		}
		if (!report_to_os) {
			return;
		}
		if (value)
		{
			if(!rmi_touchpad_touched_count(tp)) //only suppress button down when no conductive object
				return;
		}
		key_data.key_code = code;
		key_data.key_value = value;
		key_data.time_stamp = jiffies;
		rmi_touchpad_queue_add(&tp->key_fifo, &key_data);
		pr_debug("add key into queue %d \n",key_data.key_value);
		return ;
	}
	if (code == BTN_TOUCH) {
		value = rmi_touchpad_reported_count(tp); /* override the value due to this value doesn't depend on type now */
	}
	_rmi_input_report_key(rmi_input, code, value);
}

static void rmi_touchpad_report_abs(struct rmi_input_dev *rmi_input,
					struct rmi_input_abs_pos *abs)
{
	struct rmi_touchpad *tp =
			(struct rmi_touchpad *)rmi_input->context;
	struct rmi_touchpad_finger * finger = &tp->fingers[abs->n_finger];
	struct rmi_input_abs_pos queue_abs;

	abs->time_stamp = jiffies;

	if(finger->finger_state == RMI_INPUT_NO_FINGER)
	{
		rmi_touchpad_queue_clear(&finger->finger_fifo);
		rmi_touchpad_queue_clear(&finger->finger_buffer);
		if(!rmi_touchpad_touched_count(tp)) // it's the first finger down.
		{
			rmi_touchpad_queue_clear(&tp->key_fifo);
		}
		 /*considering the requirement of filters for release/dribble packet; so clear the queue in first down packet. */
		finger->finger_state = RMI_INPUT_FINGER_PRESENT;
	}

	if(tp->suppress_finger)
		return;
	if (!finger->suppress_count) {
		bool ret = true;
		if(!list_empty(&tp->filter_list))
		{
			struct rmi_filter *entry;
			list_for_each_entry(entry, &tp->filter_list, node) {
				if(entry->pre_down_proc)
					ret = entry->pre_down_proc(entry, abs);
				if(!ret)
					break;
			}
		}
		if(ret)
		{
			if(!list_empty(&tp->filter_list))
			{
				struct rmi_filter *entry;
				list_for_each_entry(entry, &tp->filter_list, node) {
					if(entry->down_proc)
					ret &= entry->down_proc(entry, abs);
				}
			}
			if(ret) /*don't add this packet into report fifo if any filters return false */
			{
				rmi_touchpad_queue_add(&finger->finger_fifo, abs);
			}
		}
		if(!ret) // the packet is not in the fifo. we have to send out key packet to prevent losing it
		{
			struct rmi_input_abs_pos queue_key;
			if(rmi_touchpad_queue_get(&tp->key_fifo, &queue_key))
			{
				_rmi_input_report_key(rmi_input, queue_key.key_code, queue_key.key_value);
				pr_debug("key sent due to suppress pkt value =%d \n",queue_key.key_value);
			}
		}
	}
	if (!finger->reporting
		&& rmi_touchpad_queue_count(&finger->finger_fifo) > tp->packet_delay_count)
	{
		/*
		 * the queue has filled up so we now have enough position
		 * data to begin reporting to the input subsystem.
		 */
		finger->reporting = 1;
	}

	if (finger->reporting) {
		if (rmi_touchpad_queue_get(&finger->finger_fifo, &queue_abs)) {
			struct rmi_input_abs_pos queue_key;
			_rmi_input_report_abs(rmi_input, &queue_abs);
			if(rmi_touchpad_queue_check(&tp->key_fifo,&queue_key))
			{
				if(time_after(queue_abs.time_stamp,queue_key.time_stamp))
				{
					if(rmi_touchpad_queue_get(&tp->key_fifo, &queue_key))
					{
						_rmi_input_report_key(rmi_input, queue_key.key_code, queue_key.key_value);
					}
				}
			}
		}

		rmi_touchpad_queue_add(&finger->finger_buffer, abs);
	}
}

static void rmi_touchpad_report_state(struct rmi_input_dev *rmi_input,
				enum rmi_input_finger_state state, struct rmi_input_abs_pos *abs)
{
	struct rmi_touchpad_finger * finger;
	struct device *dev = rmi_input->input_dev->dev.parent;
	struct rmi_input_abs_pos queue_abs;
	struct rmi_touchpad * tp =
			(struct rmi_touchpad *)rmi_input->context;
	struct rmi_input_abs_pos queue_key;
	abs->time_stamp = jiffies;
	finger = &tp->fingers[abs->n_finger];
	if (state == RMI_INPUT_NO_FINGER) {
		if (finger->finger_state == RMI_INPUT_FINGER_PRESENT) {
			/* finger went away */
			pr_debug("%s: finger (%d) went away\n",__func__,abs->n_finger);
		}
		if(tp->has_dribble == false) // dribble should be always enabled when this file built in. this is just a workaround to make device workable when dribble has problem.
		{
			if(finger->reporting)
			{
				while(rmi_touchpad_queue_get(&finger->finger_fifo, &queue_abs))
				{
					_rmi_input_report_abs(rmi_input, &queue_abs);
					input_sync(rmi_input->input_dev);
				}
			}
		}
		if (rmi_touchpad_queue_get(&finger->finger_fifo, &queue_abs)) {
			_rmi_input_report_abs(rmi_input, &queue_abs);
			finger->finger_state = RMI_INPUT_FINGER_PRESENT;
			dev_info(dev, "draining queue: finger[%d] - x:%d y:%d z:%d"
				" w_max:%d w_min:%d\n", queue_abs.n_finger, queue_abs.x,
			queue_abs.y, queue_abs.z,
			queue_abs.w_max, queue_abs.w_min);
			finger->reporting = 1;
		} else {
			if(!list_empty(&tp->filter_list))
			{
				struct rmi_filter *entry;
				bool ret = true;
				list_for_each_entry(entry, &tp->filter_list, node) {
					if(entry->up_proc)
						ret &= entry->up_proc(entry, abs);
				}
				if(!ret) /*for now, we suppress this packet if any filter return false, but the packet still need go through every filters to prevent error */
				{
					return;
				}
			}
			rmi_touchpad_clear_finger(finger);
			_rmi_input_report_state(rmi_input, RMI_INPUT_NO_FINGER, abs);
		}
		rmi_touchpad_queue_add(&finger->finger_buffer, abs);
	} else {
		finger->finger_state = state;
	}
	if(rmi_touchpad_queue_count(&tp->key_fifo) && !rmi_touchpad_touched_count(tp))
	{
		rmi_touchpad_queue_get(&tp->key_fifo, &queue_key);
		_rmi_input_report_key(rmi_input, queue_key.key_code, queue_key.key_value);
	}
}
EXPORT_SYMBOL_GPL(rmi_touchpad_inject);

void rmi_touchpad_inject(struct rmi_input_dev *rmi_input,struct rmi_input_abs_pos *abs)
{
	struct rmi_touchpad * tp =
			(struct rmi_touchpad *)rmi_input->context;
	struct rmi_touchpad_finger * finger = &tp->fingers[abs->n_finger];
	rmi_touchpad_queue_clear(&finger->finger_fifo); // need to clear the fifo due to no dribble pkts
	rmi_touchpad_report_state(rmi_input,RMI_INPUT_NO_FINGER,abs);
}

EXPORT_SYMBOL_GPL(rmi_touchpad_reset_finger);

void rmi_touchpad_reset_finger(struct rmi_input_dev *rmi_input,struct rmi_input_abs_pos *abs)
{
	struct rmi_touchpad_finger * finger;
	struct rmi_touchpad * tp =
			(struct rmi_touchpad *)rmi_input->context;
	finger = &tp->fingers[abs->n_finger];
	rmi_touchpad_clear_finger(finger);
	rmi_touchpad_queue_clear(&finger->finger_fifo);
}
int rmi_touchpad_create(struct rmi_input_dev *rmi_input, int finger_count, int x_max, int y_max)
{

	struct rmi_touchpad *tp;
	struct device *dev = rmi_input->input_dev->dev.parent;
	int i;

	tp = devm_kzalloc(dev, sizeof(struct rmi_touchpad), GFP_KERNEL);
	if (!tp)
		return -ENOMEM;
	tp->key_fifo.pos_data= devm_kzalloc(dev,
			sizeof(struct rmi_input_abs_pos) * RMI_TOUCHPAD_FINGER_QUEUE_SIZE,
			GFP_KERNEL);
	if (!tp->key_fifo.pos_data)
		return -ENOMEM;
	tp->fingers = devm_kzalloc(dev, sizeof(struct rmi_touchpad_finger)*finger_count,
					GFP_KERNEL);
	if (!tp->fingers)
		return -ENOMEM;

	for (i = 0; i < finger_count; ++i) {
		tp->fingers[i].finger_fifo.pos_data= devm_kzalloc(dev,
			sizeof(struct rmi_input_abs_pos) * RMI_TOUCHPAD_FINGER_QUEUE_SIZE,
			GFP_KERNEL);
		pr_debug("%s: Create fifo (%p) for finger (%p) (%d)\n",__func__,
			tp->fingers[i].finger_fifo.pos_data, &tp->fingers[i], i);
		if (!tp->fingers[i].finger_fifo.pos_data)
			return -ENOMEM;
		tp->fingers[i].finger_buffer.pos_data= devm_kzalloc(dev,
			sizeof(struct rmi_input_abs_pos) * RMI_TOUCHPAD_FINGER_QUEUE_SIZE,
			GFP_KERNEL);
		pr_debug("%s: Create buffer (%p) for finger (%p) (%d)\n",__func__,
			tp->fingers[i].finger_buffer.pos_data, &tp->fingers[i], i);
		if (!tp->fingers[i].finger_buffer.pos_data)
			return -ENOMEM;
	}

	dev_info(dev, "%s: finger_count (%d)\n", __func__, finger_count);
	tp->finger_count = finger_count;

	/* hard code till I hook up sysfs */
	tp->highw_threshold = 11;
	rmi_input->context = tp;
	tp->x_max = x_max;
	tp->y_max = y_max;
	tp->packet_delay_count = 0;
	tp->has_dribble = true;
	tp->suppress_click = 0;
	tp->suppress_finger = 0;
	INIT_LIST_HEAD(&tp->filter_list);
	rmi_palm_filter_create(rmi_input);
	rmi_zone_filter_create(rmi_input);
	rmi_tap_filter_create(rmi_input);
	rmi_click_filter_create(rmi_input);
	rmi_input->report_key = rmi_touchpad_report_key;
	rmi_input->report_abs = rmi_touchpad_report_abs;
	rmi_input->report_state = rmi_touchpad_report_state;
	rmi_tp_control_init(rmi_input);
	return 0;
}
EXPORT_SYMBOL_GPL(rmi_touchpad_create);

u8 rmi_finger_state_to_count(u8 x)
{
  x = (0x55 & x) + (0x55 & (x >> 1));
  x = (0x33 & x) + (0x33 & (x >> 2));
  x = (0x0f & x) + (0x0f & (x >> 4));
  return x;
}

static ssize_t rmi_touchpad_delay_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct rmi_touchpad *tp;
	struct rmi_driver_data *drv_data = dev_get_drvdata(dev);
	struct rmi_input_dev *rmi_input = drv_data->rmi_input;
	if(!rmi_input)
		return ENODEV;
	tp = rmi_input->context;
	return snprintf(buf, PAGE_SIZE, "%u\n", tp->packet_delay_count);
}

static ssize_t rmi_touchpad_delay_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)

{
	struct rmi_touchpad *tp;
	struct rmi_driver_data *drv_data = dev_get_drvdata(dev);
	struct rmi_input_dev *rmi_input = drv_data->rmi_input;
	unsigned int delay_count;
	if(!rmi_input)
		return ENODEV;
	tp = rmi_input->context;
	if (sscanf(buf, "%u", &delay_count) != 1)
                return -EINVAL;
	if(delay_count > RMI_TOUCHPAD_FINGER_QUEUE_SIZE-1 )
		delay_count = RMI_TOUCHPAD_FINGER_QUEUE_SIZE-1 ;
	tp->packet_delay_count = delay_count;
	return count;
}


static ssize_t rmi_touchpad_suppress_finger_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct rmi_touchpad *tp;
	struct rmi_driver_data *drv_data = dev_get_drvdata(dev);
	struct rmi_input_dev *rmi_input = drv_data->rmi_input;
	if(!rmi_input)
		return ENODEV;
	tp = rmi_input->context;
	return snprintf(buf, PAGE_SIZE, "%u\n", tp->suppress_finger);
}

static ssize_t rmi_touchpad_suppress_finger_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)

{
	struct rmi_touchpad *tp;
	struct rmi_driver_data *drv_data = dev_get_drvdata(dev);
	struct rmi_input_dev *rmi_input = drv_data->rmi_input;
	unsigned int suppress;
	if(!rmi_input)
		return ENODEV;
	tp = rmi_input->context;
	if (sscanf(buf, "%u", &suppress) != 1)
                return -EINVAL;
	tp->suppress_finger = suppress;
	return count;
}
static ssize_t rmi_touchpad_suppress_click_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct rmi_touchpad *tp;
	struct rmi_driver_data *drv_data = dev_get_drvdata(dev);
	struct rmi_input_dev *rmi_input = drv_data->rmi_input;
	if(!rmi_input)
		return ENODEV;
	tp = rmi_input->context;
	return snprintf(buf, PAGE_SIZE, "%u\n", tp->suppress_click);
}

static ssize_t rmi_touchpad_suppress_click_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)

{
	struct rmi_touchpad *tp;
	struct rmi_driver_data *drv_data = dev_get_drvdata(dev);
	struct rmi_input_dev *rmi_input = drv_data->rmi_input;
	unsigned int suppress;
	if(!rmi_input)
		return ENODEV;
	tp = rmi_input->context;
	if (sscanf(buf, "%u", &suppress) != 1)
                return -EINVAL;
	tp->suppress_click = suppress;
	return count;
}

static ssize_t rmi_touchpad_max_w_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct rmi_touchpad *tp;
	struct rmi_driver_data *drv_data = dev_get_drvdata(dev);
	struct rmi_input_dev *rmi_input = drv_data->rmi_input;
	if(!rmi_input)
		return ENODEV;
	tp = rmi_input->context;
	return snprintf(buf, PAGE_SIZE, "%u\n", tp->highw_threshold);
}

static ssize_t rmi_touchpad_max_w_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)

{
	struct rmi_touchpad *tp;
	struct rmi_driver_data *drv_data = dev_get_drvdata(dev);
	struct rmi_input_dev *rmi_input = drv_data->rmi_input;
	unsigned int highw_thresshold;
	if(!rmi_input)
		return ENODEV;
	tp = rmi_input->context;
	if (sscanf(buf, "%u", &highw_thresshold) != 1)
                return -EINVAL;
	tp->highw_threshold = highw_thresshold;
	return count;
}


static struct device_attribute dev_attr_delay_count =
        __ATTR(delay_count, RMI_RW_ATTR, rmi_touchpad_delay_show, rmi_touchpad_delay_store);

static struct device_attribute dev_attr_suppress_finger =
        __ATTR(suppress_finger, RMI_RW_ATTR, rmi_touchpad_suppress_finger_show, rmi_touchpad_suppress_finger_store);

static struct device_attribute dev_attr_suppress_click =
        __ATTR(suppress_click, RMI_RW_ATTR, rmi_touchpad_suppress_click_show, rmi_touchpad_suppress_click_store);

static struct device_attribute dev_attr_maxw =
        __ATTR(max_w, RMI_RW_ATTR, rmi_touchpad_max_w_show, rmi_touchpad_max_w_store);


static struct attribute *rmi_tp_attrs[] = {
	&dev_attr_delay_count.attr,
	&dev_attr_suppress_finger.attr,
	&dev_attr_suppress_click.attr,
	&dev_attr_maxw.attr,
	NULL
};

static const struct attribute_group rmi_touchpad_attributes = {
	.name = "touchpad",
	.attrs = rmi_tp_attrs,
};

void rmi_tp_control_init(struct rmi_input_dev *rmi_input)
{
	int error;
	struct device *dev = rmi_input->input_dev->dev.parent;
	dev_dbg(dev, "%s called.\n", __func__);
	error = sysfs_create_group(&dev->kobj, &rmi_touchpad_attributes);
	if (error) {
		dev_warn(dev, "Failed to create rmi_tp_control sysfs attributes.\n");
		return;
	}

}
void rmi_tp_control_cleanup(struct rmi_input_dev *rmi_input)
{
	struct device *dev = rmi_input->input_dev->dev.parent;
	sysfs_remove_group(&dev->kobj, &rmi_touchpad_attributes);
}

