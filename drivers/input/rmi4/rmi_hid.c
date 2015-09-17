/*
 *  Copyright (c) 2013 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/hid.h>
#include <linux/interrupt.h>
#include <linux/kconfig.h>
#include <linux/lockdep.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/circ_buf.h>
#include <linux/version.h>
#include "rmi_driver.h"

#define uhid_workaround_test 1
#define samsung_kbd_mouse_map_key_clear(c) \
	hid_map_usage_clear(hi, usage, bit, max, EV_KEY, (c))

#define to_hid_device(d) container_of(d, struct hid_device, dev)

#define REPORT_ID_MOUSE                          0x02
/* Report Id's used for RMI data access and reflash */
#define RMI_WRITE_REPORT_ID		0x9 /* Output Report */
#define RMI_READ_ADDR_REPORT_ID		0xa /* Output Report */
#define RMI_READ_DATA_REPORT_ID		0xb /* Input Report */
#define RMI_ATTN_REPORT_ID		0xc /* Input Report */
#define RMI_SET_RMI_MODE_REPORT_ID	0xf /* Feature Report */

/* flags */
#define RMI_HID_READ_REQUEST_PENDING	(1 << 0)
#define RMI_HID_READ_DATA_PENDING	(1 << 1)
#define RMI_HID_STARTED			(1 << 2)

#define RMI_HID_INPUT_REPORT_QUEUE_LEN	8

#define TOUCHPAD_WAKE_SYSTEM

enum rmi_hid_mode_type {
	RMI_HID_MODE_OFF 			= 0,
	RMI_HID_MODE_ATTN_REPORTS		= 1,
	RMI_HID_MODE_NO_PACKED_ATTN_REPORTS	= 2,
};

#define RMI_HID_REPORT_ID		0
#define RMI_HID_READ_INPUT_COUNT	1
#define RMI_HID_READ_INPUT_DATA		2
#define RMI_HID_READ_OUTPUT_ADDR	2
#define RMI_HID_READ_OUTPUT_COUNT	4
#define RMI_HID_WRITE_OUTPUT_COUNT	1
#define RMI_HID_WRITE_OUTPUT_ADDR	2
#define RMI_HID_WRITE_OUTPUT_DATA	4
#define RMI_HID_FEATURE_MODE		1
#define RMI_HID_ATTN_INTERUPT_SOURCES	1
#define RMI_HID_ATTN_DATA		2

static u16 tp_button_codes[] = {BTN_LEFT, BTN_RIGHT, BTN_MIDDLE};

static struct rmi_f30_gpioled_map tp_gpioled_map = {
	.ngpioleds = ARRAY_SIZE(tp_button_codes),
	.map = tp_button_codes,
};

static struct rmi_f11_sensor_data tp_f11_sensor_data[] = { {
	.axis_align = {
		.flip_y = 1,
	},
	.sensor_type = rmi_sensor_touchpad,
	.suppress_highw = 0,
	.x_mm = 102,
	.y_mm = 68,
} };

static struct rmi_f12_sensor_data tp_f12_sensor_data = {
	.axis_align = {
		.flip_y = 1,
	},
	.sensor_type = rmi_sensor_touchpad,
	.suppress_highw = 0,
	.x_mm = 102,
	.y_mm = 68,
};

static int rmi_hid_set_mode(struct hid_device *hdev, u8 mode);

int rmi_hid_post_resume(const void *pm_data)
{
	struct hid_device *hdev = (struct hid_device *)pm_data;
	return rmi_hid_set_mode(hdev, RMI_HID_MODE_ATTN_REPORTS);
}

static struct rmi_device_platform_data tp_platformdata = {
	.sensor_name = "TouchPad",
	.gpioled_map = &tp_gpioled_map,
	.f11_sensor_data = tp_f11_sensor_data,
	.f11_sensor_count = ARRAY_SIZE(tp_f11_sensor_data),
	.f12_sensor_data = &tp_f12_sensor_data,
	.post_resume = rmi_hid_post_resume,
	.unified_input_device = 1,
};

#define BUFFER_SIZE_INCREMENT 32
/**
 * struct rmi_hid_data - stores information for hid communication
 *
 * @page_mutex: Locks current page to avoid changing pages in unexpected ways.
 * @page: Keeps track of the current virtual page
 * @xport: Pointer to the transport interface
 *
 * @debug_buf: Buffer used for exposing buffer contents using dev_dbg
 * @debug_buf_size: Size of the debug buffer.
 *
 * @comms_debug: Latest data read/written for debugging HID communications
 * @debugfs_comms: Debugfs file for debugging HID communications
 *
 */
struct rmi_hid_data {
        struct mutex page_mutex;
        int page;
        struct rmi_transport_device *xport;

        u8 *debug_buf;
        int debug_buf_size;

	wait_queue_head_t wait;		/* For waiting for read data */
	u8 * writeReport;
	u8 * readReport;
	u8 * attnReport;

	int input_report_size;
	int output_report_size;
	int feature_report_size;

	spinlock_t input_queue_producer_lock;
	spinlock_t input_queue_consumer_lock;
	u8 * input_queue;
	int input_queue_head;
	int input_queue_tail;
	
	struct work_struct attn_report_work;

	struct work_struct reset_work;

	struct work_struct init_work;
	unsigned long last_reset_time;

	unsigned long flags;

        bool comms_debug;
#ifdef CONFIG_RMI4_DEBUG
        struct dentry *debugfs_comms;
#endif
};

#ifdef CONFIG_RMI4_DEBUG

/**
 * struct hid_debugfs_data - stores information for debugfs
 *
 * @done: Indicates that we are done reading debug data. Subsequent reads
 * will return EOF.
 * @hid_data: Pointer to the hid data
 *
 */
struct hid_debugfs_data {
        bool done;
        struct rmi_hid_data *hid_data;
};

static int debug_open(struct inode *inodep, struct file *filp)
{
        struct hid_debugfs_data *data;

        data = kzalloc(sizeof(struct hid_debugfs_data), GFP_KERNEL);
        if (!data)
                return -ENOMEM;

        data->hid_data = inodep->i_private;
        filp->private_data = data;
        return 0;
}

static int debug_release(struct inode *inodep, struct file *filp)
{
        kfree(filp->private_data);
        return 0;
}

static ssize_t comms_debug_read(struct file *filp, char __user *buffer,
                size_t size, loff_t *offset) {
        int retval;
        char *local_buf;
        struct hid_debugfs_data *dfs = filp->private_data;
        struct rmi_hid_data *data = dfs->hid_data;

        if (dfs->done)
                return 0;

        local_buf = kcalloc(size, sizeof(u8), GFP_KERNEL);
        if (!local_buf)
                return -ENOMEM;

        dfs->done = 1;

        retval = snprintf(local_buf, PAGE_SIZE, "%u\n", data->comms_debug);

        if (retval <= 0 || copy_to_user(buffer, local_buf, retval))
                retval = -EFAULT;
        kfree(local_buf);

        return retval;
}

static ssize_t comms_debug_write(struct file *filp, const char __user *buffer,
                           size_t size, loff_t *offset) {
        int retval;
        char *local_buf;
        unsigned int new_value;
        struct hid_debugfs_data *dfs = filp->private_data;
        struct rmi_hid_data *data = dfs->hid_data;

        local_buf = kcalloc(size, sizeof(u8), GFP_KERNEL);
        if (!local_buf)
                return -ENOMEM;
        retval = copy_from_user(local_buf, buffer, size);
        if (retval) {
                kfree(local_buf);
                return -EFAULT;
        }

        retval = sscanf(local_buf, "%u", &new_value);
        kfree(local_buf);
        if (retval != 1 || new_value > 1)
                return -EINVAL;

        data->comms_debug = new_value;

        return size;
}


static const struct file_operations comms_debug_fops = {
        .owner = THIS_MODULE,
        .open = debug_open,
        .release = debug_release,
        .read = comms_debug_read,
        .write = comms_debug_write,
};

static int setup_debugfs(struct rmi_device *rmi_dev, struct rmi_hid_data *data)
{
        if (!rmi_dev->debugfs_root)
                return -ENODEV;

        data->debugfs_comms = debugfs_create_file("comms_debug", RMI_RW_ATTR,
                        rmi_dev->debugfs_root, data, &comms_debug_fops);
        if (!data->debugfs_comms || IS_ERR(data->debugfs_comms)) {
                dev_warn(&rmi_dev->dev, "Failed to create debugfs comms_debug.\n");
                data->debugfs_comms = NULL;
        }

        return 0;
}

static void teardown_debugfs(struct rmi_hid_data *data)
{
        if (data->debugfs_comms)
                debugfs_remove(data->debugfs_comms);
}
#else
#define setup_debugfs(rmi_dev, data) 0
#define teardown_debugfs(data)
#endif

#define COMMS_DEBUG(data) (IS_ENABLED(CONFIG_RMI4_DEBUG) && data->comms_debug)

#define RMI_PAGE_SELECT_REGISTER 0xff
#define RMI_HID_PAGE(addr) (((addr) >> 8) & 0xff)

static char *transport_proto_name = "hid";

static int rmi_hid_write_report(struct hid_device *hdev,
				u8 * report, int len);
/*
 * rmi_set_page - Set RMI page
 * @xport: The pointer to the rmi_transport_device struct
 * @page: The new page address.
 *
 * RMI devices have 16-bit addressing, but some of the physical
 * implementations (like SMBus) only have 8-bit addressing. So RMI implements
 * a page address at 0xff of every page so we can reliable page addresses
 * every 256 registers.
 *
 * The page_mutex lock must be held when this function is entered.
 *
 * Returns zero on success, non-zero on failure.
 */
static int rmi_set_page(struct rmi_transport_device *xport, u8 page)
{
        struct hid_device *hdev = to_hid_device(xport->dev);
        struct rmi_hid_data *data = xport->data;
        int retval;

        if (COMMS_DEBUG(data))
                dev_dbg(&hdev->dev, "writes output report: %*ph\n",
			data->output_report_size, data->writeReport);

        xport->info.tx_count++;
        xport->info.tx_bytes += data->output_report_size;

	data->writeReport[RMI_HID_REPORT_ID] = RMI_WRITE_REPORT_ID;
	data->writeReport[RMI_HID_WRITE_OUTPUT_COUNT] = 1;
	data->writeReport[RMI_HID_WRITE_OUTPUT_ADDR] = 0xFF;
	data->writeReport[RMI_HID_WRITE_OUTPUT_DATA] = page;

        retval = rmi_hid_write_report(hdev, data->writeReport,
			data->output_report_size);
        if (retval != data->output_report_size) {
                xport->info.tx_errs++;
                dev_err(&hdev->dev,
                        "%s: set page failed: %d.", __func__, retval);
                return retval;
        }

        data->page = page;
        return 0;
}

static int copy_to_debug_buf(struct device *dev, struct rmi_hid_data *data,
                             const u8 *buf, const int len)
{
        int i;
        int n = 0;
        char *temp;
        int dbg_size = 3 * len + 1;

        if (!data->debug_buf || data->debug_buf_size < dbg_size) {
                if (data->debug_buf)
                        devm_kfree(dev, data->debug_buf);
                data->debug_buf_size = dbg_size + BUFFER_SIZE_INCREMENT;
                data->debug_buf = devm_kzalloc(dev, data->debug_buf_size,
                                               GFP_KERNEL);
                if (!data->debug_buf) {
                        data->debug_buf_size = 0;
                        return -ENOMEM;
                }
        }
        temp = data->debug_buf;

        for (i = 0; i < len; i++) {
                n = sprintf(temp, " %02x", buf[i]);
                temp += n;
        }

        return 0;
}

static int rmi_hid_set_mode(struct hid_device *hdev, u8 mode)
{
	int ret;
	u8 txbuf[2] = {RMI_SET_RMI_MODE_REPORT_ID, mode};

	ret = hdev->hid_output_raw_report(hdev, txbuf, sizeof(txbuf), 
			HID_FEATURE_REPORT);
	if (ret < 0) {
		dev_err(&hdev->dev, "unable to set rmi mode to %d (%d)\n", mode,
			ret);
		return ret;
	}

	return 0;
}

static int rmi_hid_write_report(struct hid_device *hdev, u8 * report, int len)
{
	int ret;

	ret = hdev->hid_output_raw_report(hdev, (void *)report, len,
		HID_OUTPUT_REPORT);
	if (ret < 0) {
		dev_err(&hdev->dev, "failed to write hid report (%d)\n", ret);
		return ret;
	}

	return ret;
}

static int rmi_hid_write_block(struct rmi_transport_device *xport, u16 addr,
			       const void *buf, const int len)
{
        struct hid_device *hdev = to_hid_device(xport->dev);
	struct rmi_hid_data *data = xport->data;
	int ret;

	mutex_lock(&data->page_mutex);

	if (RMI_HID_PAGE(addr) != data->page) {
		ret = rmi_set_page(xport, RMI_HID_PAGE(addr));
		if (ret < 0)
			goto exit;
	}

	if (COMMS_DEBUG(data)) {
		ret = copy_to_debug_buf(&hdev->dev, data, (u8 *) buf, len);
		if (!ret)
			dev_dbg(&hdev->dev, "writes %d bytes at %#06x:%s\n",
				len, addr, data->debug_buf);
	}

	xport->info.tx_count++;
	xport->info.tx_bytes += len;

	data->writeReport[RMI_HID_REPORT_ID] = RMI_WRITE_REPORT_ID;
	data->writeReport[RMI_HID_WRITE_OUTPUT_COUNT] = len;
	data->writeReport[RMI_HID_WRITE_OUTPUT_ADDR] = addr & 0xFF;
	data->writeReport[RMI_HID_WRITE_OUTPUT_ADDR + 1] = (addr >> 8) & 0xFF;
	memcpy(&data->writeReport[RMI_HID_WRITE_OUTPUT_DATA], buf, len);

	ret = rmi_hid_write_report(hdev, data->writeReport,
					data->output_report_size);
	if (ret != data->output_report_size) {
		dev_err(&hdev->dev, "failed to send output report (%d)\n", ret);
		goto exit;
	}

exit:
	mutex_unlock(&data->page_mutex);
	return ret;
}
const u16 Register_buf[64][32]=
{
	{0xef,0x00},
	{0xe9,0x20,0x00,0xa,0x00,0x21,0x34},
	{0xe3,0x24,0x1f,0x0e,0x4,0x01,0x01},
	{0x04,0x01},
	{0xdd,0x41,0x0,0x14,0x07,0x02,0x12},
	{0xd7,0,0,0,0,0,0},
	{0x1e9,0x35,0x34,0xf,0x0,0x1,0x54},
	{0x1e3,0,0,0,0x0,0,0},
	{0x2e9,0x07,0x00,0x01,0x0,0x1,0x30},
	{0x2e3,0,0,0,0,0,0},
	{0x3e9,0x08,0x07,0x0,0x0,0x1,0x55},
	{0x3e3,0,0,0,0,0,0},
	{0x4e9,0,0,0,0,0,0},
	{0x0e,0x40},
	{0x0f,0x7f},
	{0x05,0x00},
	{0x24,0x01,0xe0,0x01,0x04,0xcc,0xeb,0x86,0xe6,0x28,0x31,0x7d,0x54,0x4d,
	 0x33,0x31,0x31,0x31,0x2d,0x30,0x30,0x31},
	{0x2f,0x54,0x4d,0x33,0x31,0x31,0x31,0x2d,0x30,0x30,0x31},
	{0x39,0x09},
	{0x3a,0x03},
	{0x3b,0x2f},
	{0x3c,0x10},
	{0x3d,0x03},
	{0x35,0x8e,0x23,0x01,0x0},
	{0x36,0xf0,0x7d,0x1c},
	{0x3e,0x00},
	{0x201,0x0},
	{0x202,0x0},
	{0x203,0x4},  //index 30
	{0x204,0x0},
	{0x205,0x0},
	{0x207,0x28,0x3,0,0},
	{0x206,0x0c,0x28},
	{0x208,0x3,0x0,0x0},
	{0x20b,0x0},
	{0x20c,0x0,0x0},
	{0x20e,0x0},
	{0x3f,0x44,0x57,0x20,0x52,0x35,0x2e,0x30,0x2e,0x30,0x0,0,0,0,0,0,0},
	{0x40,0x53,0x59,0x4e,0x41,0x50,0x54,0x49,0x43,0x53,0x20,0x52,0x35,0x2e,
	0x30,0x2e,0x30},
	{0x41,0x9},
	{0x42,0x3},
	{0x43,0x1d,0xff,0x7},
	{0x44,0x1,0x1,0x1,0x1,0x3,0x3,0x1d,0xff,0xf,0x1,0x1,0x5,0x3,
	0x17,0x80,0x9e,0xc2,0x97,0x1,0x1,0x1,0x3,0x3,0x6,0x82,0x84,0x2,0x1,0x1},
	{0x45,0x5},
	{0x46,0x17,0x0,0x8f,0xf0,0x12},
	{0x47,0xe,0xf,0xf,0x7,0x6,0x7,0xf,0xf,0x6,0x83,0x0,0x3,0x3,
	0x2,0x1,0x1,0x1,0x2,0x3,0x1,0x1,0x1,0x1},
	{0x48,0x3},
	{0x49,0x6,0x2,0x82},
	{0x4a,0x18,0x7,0x6,0x7,0x1,0x1},
	{0x14,0xa8,0x2,0x45,0x1,0x85,0x4b,0x52,0x48,0,0,0,0,0xc,0x6},
	{0x19,0x0,0x0,0x4},
	{0x10,0x3},
	{0x11,0x1e},
	{0x12,0x30}
};

static int rmi_hid_read_from_array(struct rmi_transport_device *xport, u16 addr,
			      void *buf, const int len)
{
	int i=0,j=0;
	//printk("rmi_hid_read_from_array addr=0x%x, len=%d \n",addr,len);
	for(i=0;i<64;i++)
	{
		if(addr==Register_buf[i][0])
		{
			for(j=0;j<len;j++)
			{
				*((char*)buf+j)=Register_buf[i][1+j]&0xff;
				//printk("array[%d]=0x%x",j,*((char*)buf+j));
			}
			goto ARRAY_Find;
		}
	}
	return -1;
ARRAY_Find:
	return 0;
}

static int rmi_hid_read_block(struct rmi_transport_device *xport, u16 addr,
			      void *buf, const int len)
{
	struct hid_device *hdev = to_hid_device(xport->dev);
	struct rmi_hid_data *data = xport->data;
	int ret;
	int bytes_read;
	int bytes_needed;
	int retries;
	int read_input_count;

	mutex_lock(&data->page_mutex);
	if(!rmi_hid_read_from_array(xport,addr,buf,len))
	{
		xport->info.rx_count++;
		xport->info.rx_bytes += len;
		ret = len;
		goto exit;
	}
	if (RMI_HID_PAGE(addr) != data->page) {
		ret = rmi_set_page(xport, RMI_HID_PAGE(addr));
		if (ret < 0)
			goto exit;
	}

	for (retries = 5; retries > 0; retries--) {
		data->writeReport[RMI_HID_REPORT_ID] = RMI_READ_ADDR_REPORT_ID;
		data->writeReport[1] = 0; /* old 1 byte read count */
		data->writeReport[RMI_HID_READ_OUTPUT_ADDR] = addr & 0xFF;
		data->writeReport[RMI_HID_READ_OUTPUT_ADDR + 1] = (addr >> 8) & 0xFF;
		data->writeReport[RMI_HID_READ_OUTPUT_COUNT] = len  & 0xFF;
		data->writeReport[RMI_HID_READ_OUTPUT_COUNT + 1] = (len >> 8) & 0xFF;

		if (COMMS_DEBUG(data)) {
			ret = copy_to_debug_buf(&hdev->dev, data, data->writeReport, len);
			if (!ret)
				dev_dbg(&hdev->dev, "wrote %d bytes at %#06x:%s\n",
					len, addr, data->debug_buf);
		}

		set_bit(RMI_HID_READ_REQUEST_PENDING, &data->flags);

		ret = rmi_hid_write_report(hdev, data->writeReport,
						data->output_report_size);
		if (ret != data->output_report_size) {
			clear_bit(RMI_HID_READ_REQUEST_PENDING, &data->flags);
			dev_err(&hdev->dev,
				"failed to write request output report (%d)\n", ret);
			goto exit;
		}

		bytes_read = 0;
		bytes_needed = len;
		while (bytes_read < len) {
			if (!wait_event_interruptible_timeout(data->wait, 
					test_bit(RMI_HID_READ_DATA_PENDING, &data->flags),
					msecs_to_jiffies(10000)))
			{
					dev_info(&hdev->dev, "%s: timeout elapsed\n", __func__);
					ret = -ENODATA;
					break;
			} else {
				if (data->readReport[RMI_HID_REPORT_ID]
						!= RMI_READ_DATA_REPORT_ID)
				{
					ret = -ENODATA;
					dev_err(&hdev->dev,
						"%s: Expected data report, but got"
						" report id %d instead", __func__,
						data->readReport[RMI_HID_REPORT_ID]);
					goto exit;
				}

				read_input_count = data->readReport[RMI_HID_READ_INPUT_COUNT];
				memcpy(buf + bytes_read,
					&data->readReport[RMI_HID_READ_INPUT_DATA],
					read_input_count < bytes_needed 
					? read_input_count : bytes_needed);

				if (COMMS_DEBUG(data)) {
					ret = copy_to_debug_buf(&hdev->dev, data,
							(u8 *) buf + bytes_read,
							read_input_count);
					if (!ret)
						dev_dbg(&hdev->dev, "read %d bytes at %#06x:%s\n",
							read_input_count, addr,
							data->debug_buf);
				}
				bytes_read += read_input_count;
				bytes_needed -= read_input_count;
				clear_bit(RMI_HID_READ_DATA_PENDING, &data->flags);
			}
		}

		if (bytes_read == len)
			break;
	}

	if (bytes_read == len) {
		xport->info.rx_count++;
		xport->info.rx_bytes += len;
		ret = len;
	}
	printk("warning.. reading from touchpad \n");
	//for( i=0;i<ret;i++)
	//	printk("data[%d]=0x%x\n",i,*((char*)buf+i));
exit:
	clear_bit(RMI_HID_READ_REQUEST_PENDING, &data->flags);
	mutex_unlock(&data->page_mutex);
	return ret;
}

static void rmi_hid_reset_work(struct work_struct *work)
{
	struct rmi_hid_data * hdata = container_of(work, struct rmi_hid_data,
						reset_work);
	struct rmi_transport_device * xport = hdata->xport;

        struct hid_device *hdev = to_hid_device(xport->dev);
	rmi_hid_set_mode(hdev, RMI_HID_MODE_ATTN_REPORTS);  
}

static void rmi_hid_init_work(struct work_struct *work)
{
	struct rmi_hid_data * hdata = container_of(work, struct rmi_hid_data,
						init_work);
	struct rmi_transport_device * xport = hdata->xport;

    struct hid_device *hdev = to_hid_device(xport->dev);

	int ret=0;
	dev_err(&hdev->dev, "rmi_hid_set_mode! \n");
	ret = rmi_hid_set_mode(hdev, RMI_HID_MODE_ATTN_REPORTS);
	if (ret < 0) {
		dev_err(&hdev->dev, "failed to set rmi mode\n");
	}
	dev_err(&hdev->dev, "rmi_hid_set_mode finished \n");
	ret = rmi_set_page(xport, 0);
	if (ret < 0) {
	}
		ret = rmi_register_transport_device(xport);
	if (ret) {
		dev_err(&hdev->dev, "failed to register transport device at %s\n",
		hdev->phys);
	}
	if (!xport->probe_succeeded) {
		dev_err(&hdev->dev, "Probe failed in rmi_driver\n");
		ret = -ENODEV;
		goto rmi_driver_probe_failed;
	}

	set_bit(RMI_HID_STARTED, &hdata->flags);
	dev_err(&hdev->dev, "init work done.\n");
	return;
	rmi_driver_probe_failed:
	rmi_unregister_transport_device(xport);
}


static void rmi_hid_attn_report_work(struct work_struct *work)
{
	struct rmi_hid_data * hdata = container_of(work, struct rmi_hid_data,
						attn_report_work);
	struct rmi_transport_device * xport = hdata->xport;
	u8 * queue_report;
	struct rmi_driver_data * drv_data;
	unsigned long lock_flags;
	int head;
	int tail;

	spin_lock_irqsave(&hdata->input_queue_consumer_lock, lock_flags);

	head = ACCESS_ONCE(hdata->input_queue_head);
	tail = hdata->input_queue_tail;

	while (CIRC_CNT(head, tail, RMI_HID_INPUT_REPORT_QUEUE_LEN)) {
		queue_report = hdata->input_queue
				+ hdata->input_report_size * tail;
		memcpy(hdata->attnReport, queue_report, hdata->input_report_size);

		smp_mb();
		hdata->input_queue_tail = (tail + 1) & (RMI_HID_INPUT_REPORT_QUEUE_LEN - 1);

		//dev_dbg(&xport->rmi_dev->dev, "attn = %*ph\n", hdata->input_report_size,
		//	hdata->attnReport);

		if (test_bit(RMI_HID_STARTED, &hdata->flags)) {
			/* process it! */
			drv_data = dev_get_drvdata(&xport->rmi_dev->dev);
			*(drv_data->irq_status) =
				hdata->attnReport[RMI_HID_ATTN_INTERUPT_SOURCES];
			xport->attn_data = &hdata->attnReport[RMI_HID_ATTN_DATA];
			xport->attn_size = hdata->input_report_size -
						1 /* report id */ -
						1 /* interrupt sources */;
			xport->rmi_dev->driver->process(xport->rmi_dev);
		}

		head = ACCESS_ONCE(hdata->input_queue_head);
		tail = hdata->input_queue_tail;
	}
	spin_unlock_irqrestore(&hdata->input_queue_consumer_lock, lock_flags);
}

static int rmi_hid_raw_event(struct hid_device *hdev,
		struct hid_report *report, u8 *data, int size)
{
	struct rmi_transport_device *xport = hid_get_drvdata(hdev);
	struct rmi_hid_data * hdata = xport->data;
	u8 * queue_report;
	int sched_work = 0;
	unsigned long lock_flags;
	int head;
	int tail;
	unsigned long last_timeout;
	unsigned long current_time;
	//printk("rmi_hid_raw_event report id=%d \n",data[RMI_HID_REPORT_ID]);
	if (data[RMI_HID_REPORT_ID] == RMI_READ_DATA_REPORT_ID) {
		if (test_bit(RMI_HID_READ_REQUEST_PENDING, &hdata->flags)) {
			memcpy(hdata->readReport, data,
				size < hdata->input_report_size ? size
				: hdata->input_report_size);
			set_bit(RMI_HID_READ_DATA_PENDING, &hdata->flags);
			wake_up(&hdata->wait);
		} else
			dev_info(&hdev->dev, "NO READ REQUEST PENDING\n");

	} else if (data[RMI_HID_REPORT_ID] == RMI_ATTN_REPORT_ID
			&& test_bit(RMI_HID_STARTED, &hdata->flags))
	{
		spin_lock_irqsave(&hdata->input_queue_producer_lock, lock_flags);
		
		head = hdata->input_queue_head;
		tail = ACCESS_ONCE(hdata->input_queue_tail);


		if (!CIRC_CNT(head, tail, RMI_HID_INPUT_REPORT_QUEUE_LEN))
			sched_work = 1;

		queue_report = hdata->input_queue
				+ hdata->input_report_size * head;
		memcpy(queue_report, data, size < hdata->input_report_size ? size
			: hdata->input_report_size);

		smp_wmb();
		hdata->input_queue_head = (head + 1) & (RMI_HID_INPUT_REPORT_QUEUE_LEN - 1);

		spin_unlock_irqrestore(&hdata->input_queue_producer_lock, lock_flags);

		if (sched_work)
			schedule_work(&hdata->attn_report_work);
	}
	else if (data[RMI_HID_REPORT_ID] == REPORT_ID_MOUSE) {
		last_timeout = hdata->last_reset_time + 5*HZ;
		current_time = jiffies;
		if (current_time > last_timeout) {
			hdata->last_reset_time = current_time;
			schedule_work(&hdata->reset_work);
		}
	}
	return 0;
}

static void rmi_hid_post_reset(struct rmi_transport_device *xport)
{
        struct hid_device *hdev = to_hid_device(xport->dev);
	rmi_hid_set_mode(hdev, RMI_HID_MODE_ATTN_REPORTS);
}

static int rmi_hid_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct rmi_transport_device *xport = NULL;
	struct rmi_hid_data *data = NULL;
	unsigned int connect_mask = HID_CONNECT_DEFAULT;
	int ret;
	struct hid_report *input_report;
	struct hid_report *output_report;
	struct hid_report *feature_report;

	dev_dbg(&hdev->dev, "%s\n", __func__);

	xport = devm_kzalloc(&hdev->dev, sizeof(struct rmi_transport_device),
				GFP_KERNEL);
	if (!xport) {
		ret = -ENOMEM;
		goto err;
	}

	data = devm_kzalloc(&hdev->dev, sizeof(struct rmi_hid_data),
				GFP_KERNEL);
	if (!data) {
		ret =-ENOMEM;
		goto err;
	}

	data->xport = xport;

	xport->data = data;
	xport->dev = &hdev->dev;

	xport->write_block = rmi_hid_write_block;
	xport->read_block = rmi_hid_read_block;
	xport->info.proto_type = RMI_PROTOCOL_HID;
	xport->info.proto = transport_proto_name;
	xport->post_reset = rmi_hid_post_reset;
	hid_set_drvdata(hdev, xport);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed\n");
		goto err;
	}
#if 0
	data->input_report_size =
		(hdev->report_enum[HID_INPUT_REPORT]
		.report_id_hash[RMI_ATTN_REPORT_ID]->size >> 3)
		+ 1 /* report id */;
	data->output_report_size =
		(hdev->report_enum[HID_OUTPUT_REPORT]
		.report_id_hash[RMI_WRITE_REPORT_ID]->size >> 3)
		+ 1 /* report id */;
	data->feature_report_size =
		(hdev->report_enum[HID_FEATURE_REPORT]
		.report_id_hash[RMI_SET_RMI_MODE_REPORT_ID]->size >> 3)
		+ 1 /* report id */;
#endif
	input_report = hdev->report_enum[HID_INPUT_REPORT].report_id_hash[RMI_ATTN_REPORT_ID];
	if (!input_report) {
		hid_err(hdev, "device does not have expected input report\n");
		ret = -ENODEV;
		return ret;
	}
	data->input_report_size = (input_report->size >> 3) + 1 /* report id */;

	output_report = hdev->report_enum[HID_OUTPUT_REPORT].report_id_hash[RMI_WRITE_REPORT_ID];
	if (!output_report) {
		hid_err(hdev, "device does not have expected output report\n");
		ret = -ENODEV;
		return ret;
	}
	data->output_report_size = (output_report->size >> 3)+ 1 /* report id */;

	feature_report = hdev->report_enum[HID_FEATURE_REPORT].report_id_hash[RMI_SET_RMI_MODE_REPORT_ID];
	if (!feature_report) {
		hid_err(hdev, "device does not have expected feature report\n");
		ret = -ENODEV;
		return ret;
	}
	data->feature_report_size = (feature_report->size >> 3)+ 1 /* report id */;

	dev_dbg(&hdev->dev, "input report size %d\n", data->input_report_size);
	dev_dbg(&hdev->dev, "output report size %d\n",
		data->output_report_size);
	dev_dbg(&hdev->dev, "feature report size %d\n",
		data->feature_report_size);

	data->input_queue = devm_kzalloc(&hdev->dev, data->input_report_size
				* RMI_HID_INPUT_REPORT_QUEUE_LEN, GFP_KERNEL);
	if (!data->input_queue) {
		ret = -ENOMEM;
		goto err;
	}

	data->writeReport = devm_kzalloc(&hdev->dev, data->output_report_size,
				GFP_KERNEL);
	if (!data->writeReport) {
		ret = -ENOMEM;
		goto err;
	}

	data->readReport = devm_kzalloc(&hdev->dev, data->input_report_size,
				GFP_KERNEL);
	if (!data->readReport) {
		ret = -ENOMEM;
		goto err;
	}

	data->attnReport = devm_kzalloc(&hdev->dev, data->input_report_size,
				GFP_KERNEL);
	if (!data->attnReport) {
		ret = -ENOMEM;
		goto err;
	}

	tp_platformdata.pm_data = hdev;
	xport->dev->platform_data = &tp_platformdata;

#ifdef TOUCHPAD_WAKE_SYSTEM
	if (tp_platformdata.f11_sensor_data[0].sensor_type == rmi_sensor_touchpad) {
		device_init_wakeup(hdev->dev.parent, 1);
	}
#endif

	spin_lock_init(&data->input_queue_consumer_lock);
	spin_lock_init(&data->input_queue_producer_lock);
	data->input_queue_head = 0;
	data->input_queue_tail = 0;
	INIT_WORK(&data->attn_report_work, rmi_hid_attn_report_work);
	INIT_WORK(&data->reset_work, rmi_hid_reset_work);
	INIT_WORK(&data->init_work, rmi_hid_init_work);
	init_waitqueue_head(&data->wait);

	mutex_init(&data->page_mutex);

	ret = hid_hw_start(hdev, connect_mask);
	if (ret) {
		hid_err(hdev, "hw start failed\n");
		goto err;
	}

	dev_dbg(&hdev->dev, "Opening low level driver\n");
	hdev->ll_driver->open(hdev);

	/* Allow incoming hid reports */
	hid_device_io_start(hdev);
#ifndef uhid_workaround_test
	dev_err(&hdev->dev, "rmi_hid_set_mode! \n");
	ret = rmi_hid_set_mode(hdev, RMI_HID_MODE_ATTN_REPORTS);
	if (ret < 0) {
		dev_err(&hdev->dev, "failed to set rmi mode\n");
		goto rmi_read_failed;
	}
	dev_err(&hdev->dev, "rmi_hid_set_mode finished \n");
	ret = rmi_set_page(xport, 0);
	if (ret < 0) {
		dev_err(&hdev->dev, "failed to set page select to 0.\n");
		goto rmi_read_failed;
	}

	ret = rmi_register_transport_device(xport);
	if (ret) {
		dev_err(&hdev->dev, "failed to register transport device at %s\n",
			hdev->phys);
		goto rmi_read_failed;
	}

	if (!xport->probe_succeeded) {
		dev_err(&hdev->dev, "Probe failed in rmi_driver\n");
		ret = -ENODEV;
		goto rmi_driver_probe_failed;
	}

	set_bit(RMI_HID_STARTED, &data->flags);
	if (IS_ENABLED(CONFIG_RMI4_DEBUG))
		ret = setup_debugfs(xport->rmi_dev, data);
#else
	schedule_work(&data->init_work);
#endif

	dev_info(&hdev->dev, "registered rmi hid driver at %s\n", hdev->phys);
	return 0;

#ifndef uhid_workaround_test
rmi_driver_probe_failed:
	rmi_unregister_transport_device(xport);

rmi_read_failed:
#endif
	hdev->ll_driver->close(hdev);
	hid_hw_stop(hdev);

err:
	return ret;
}

static void rmi_hid_remove(struct hid_device *hdev)
{
	struct rmi_transport_device *xport = hid_get_drvdata(hdev);
	struct rmi_hid_data * hdata = xport->data;

	//clear_bit(RMI_HID_STARTED, &hdata->flags);
	cancel_work_sync(&hdata->attn_report_work);
	cancel_work_sync(&hdata->reset_work);
	cancel_work_sync(&hdata->init_work);
	if (IS_ENABLED(CONFIG_RMI4_DEBUG))
		teardown_debugfs(xport->data);
	if (test_bit(RMI_HID_STARTED, &hdata->flags)) {
		rmi_unregister_transport_device(xport);
		clear_bit(RMI_HID_STARTED, &hdata->flags);
	}

	hdev->ll_driver->close(hdev);
	hid_hw_stop(hdev);
}

static int samsung_bookcover_input_mapping(struct hid_device *hdev,
	struct hid_input *hi, struct hid_field *field, struct hid_usage *usage,
	unsigned long **bit, int *max)
{
	if (!(HID_UP_CONSUMER == (usage->hid & HID_USAGE_PAGE) ||
			HID_UP_KEYBOARD == (usage->hid & HID_USAGE_PAGE)))
		return 0;

	dbg_hid("samsung wireless keyboard input mapping event [0x%x]\n",
		usage->hid & HID_USAGE);

	if (HID_UP_KEYBOARD == (usage->hid & HID_USAGE_PAGE)) {
		switch (usage->hid & HID_USAGE) {
		set_bit(EV_REP, hi->input->evbit);
		/* Only for UK keyboard */
		/* key found */
#ifdef CONFIG_HID_KK_UPGRADE
        case 0x32: samsung_kbd_mouse_map_key_clear(KEY_KBDILLUMTOGGLE); break;
        case 0x64: samsung_kbd_mouse_map_key_clear(KEY_BACKSLASH); break;
#else
        case 0x32: samsung_kbd_mouse_map_key_clear(KEY_BACKSLASH); break;
        case 0x64: samsung_kbd_mouse_map_key_clear(KEY_102ND); break;
#endif
        /* Only for BR keyboard */
        case 0x87: samsung_kbd_mouse_map_key_clear(KEY_RO); break;
        default:
            return 0;
		}
	}

	if (HID_UP_CONSUMER == (usage->hid & HID_USAGE_PAGE)) {
        printk(KERN_DEBUG "switch value : 0x%x\n", usage->hid & HID_USAGE);  
		switch (usage->hid & HID_USAGE) {
		/* report 2 */
		/* MENU */
		case 0x040: samsung_kbd_mouse_map_key_clear(KEY_MENU); break;
		case 0x18a: samsung_kbd_mouse_map_key_clear(KEY_MAIL); break;
		case 0x196: samsung_kbd_mouse_map_key_clear(KEY_WWW); break;
		case 0x19e: samsung_kbd_mouse_map_key_clear(KEY_SCREENLOCK); break;
		case 0x221: samsung_kbd_mouse_map_key_clear(KEY_SEARCH); break;
		case 0x223: samsung_kbd_mouse_map_key_clear(KEY_HOMEPAGE); break;
		/* RECENTAPPS */
		case 0x301: samsung_kbd_mouse_map_key_clear(BTN_TRIGGER_HAPPY1); break;
		/* APPLICATION */
		case 0x302: samsung_kbd_mouse_map_key_clear(BTN_TRIGGER_HAPPY2); break;
		/* Voice search */
		case 0x305: samsung_kbd_mouse_map_key_clear(BTN_TRIGGER_HAPPY4); break;
		/* QPANEL on/off */
		case 0x306: samsung_kbd_mouse_map_key_clear(BTN_TRIGGER_HAPPY5); break;
		/* SIP on/off */
		case 0x307: samsung_kbd_mouse_map_key_clear(BTN_TRIGGER_HAPPY3); break;
		/* LANG */
		case 0x308: samsung_kbd_mouse_map_key_clear(KEY_LANGUAGE); break;
		case 0x30a: samsung_kbd_mouse_map_key_clear(KEY_BRIGHTNESSDOWN); break;
		case 0x070: samsung_kbd_mouse_map_key_clear(KEY_BRIGHTNESSDOWN); break;
		case 0x30b: samsung_kbd_mouse_map_key_clear(KEY_BRIGHTNESSUP); break;
		case 0x06f: samsung_kbd_mouse_map_key_clear(KEY_BRIGHTNESSUP); break;
		/* S-Finder */
		case 0x304: samsung_kbd_mouse_map_key_clear(BTN_TRIGGER_HAPPY7); break;
		/* Screen Capture */
		case 0x303: samsung_kbd_mouse_map_key_clear(KEY_SYSRQ); break;
		/* Multi Window */
		case 0x309: samsung_kbd_mouse_map_key_clear(BTN_TRIGGER_HAPPY9); break;
		/* TouchPad ON for TabS2*/
		case 0x310: samsung_kbd_mouse_map_key_clear(KEY_TOUCHPAD_ON); break;
		/* TouchPad OFF for TabS2*/
		case 0x311: samsung_kbd_mouse_map_key_clear(KEY_TOUCHPAD_OFF); break;
		default:
			return 0;
		}
	}

	return 1;
}

static int rmi_hid_mapping(struct hid_device *hdev, struct hid_input *hi,
	struct hid_field *field, struct hid_usage *usage,
	unsigned long **bit, int *max)
{
	int ret = 0;

	if (0xa008/*USB_DEVICE_ID_SAMSUNG_NEW_WIRELESS_BOOKCOVER*/ == hdev->product)
		ret = samsung_bookcover_input_mapping(hdev,
			hi, field, usage, bit, max);

	return ret;
}

static const struct hid_device_id rmi_hid_id[] = {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0))
	{ HID_DEVICE(BUS_I2C, HID_GROUP_ANY, 0x06cb, HID_ANY_ID),
#else
	{ HID_DEVICE(BUS_I2C, 0x06cb, HID_ANY_ID),
#endif
		.driver_data = 0 },
	{ HID_USB_DEVICE(0x06cb, HID_ANY_ID),
		.driver_data = 0 },
    { HID_BLUETOOTH_DEVICE(0x04e8/*USB_VENDOR_ID_SAMSUNG_ELECTRONICS*/, 0xa008/*USB_DEVICE_ID_SAMSUNG_WIRELESS_BOOKCOVER_TABS2*/),
		.driver_data = 0},
	{ }
};
MODULE_DEVICE_TABLE(hid, rmi_hid_id);

static struct hid_driver rmi_hid_driver = {
	.name = "rmi_hid",
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "rmi_hid",
	},
	.id_table	= rmi_hid_id,
	.probe		= rmi_hid_probe,
	.remove		= rmi_hid_remove,
	.raw_event	= rmi_hid_raw_event,
	.input_mapping = rmi_hid_mapping,
};

static int __init rmi_hid_init(void)
{
	int ret;

	ret = hid_register_driver(&rmi_hid_driver);
	if (ret)
		pr_err("can't register rmi_hid driver (%d)\n", ret);
	else
		pr_info("Successfully registered rmi_hid driver\n");

	return ret;
}

static void __exit rmi_hid_exit(void)
{
	hid_unregister_driver(&rmi_hid_driver);
}

late_initcall(rmi_hid_init);
module_exit(rmi_hid_exit);

MODULE_AUTHOR("Andrew Duggan <aduggan@synaptics.com>");
MODULE_DESCRIPTION("RMI HID driver");
MODULE_LICENSE("GPL");
