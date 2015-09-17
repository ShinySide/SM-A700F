/*
 * Copyright (c) 2013 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include "rmi_driver.h"
#include "rmi_input_dev.h"


struct rmi_input_dev *rmi_input_dev_alloc(struct device *dev, void * data,
                        const char * phys_fmt, void *input_drvdata)
{
	struct rmi_driver_data * drvdata = (struct rmi_driver_data *)data;
	struct rmi_device *rmi_dev = drvdata->rmi_dev;
	struct rmi_driver *driver = rmi_dev->driver;
	struct rmi_input_dev *rmi_input;
	int rc = 0;

	rmi_input = devm_kzalloc(dev, sizeof(struct rmi_input_dev), GFP_KERNEL);
	if (!rmi_input)
		return NULL;

	/* initialize to default values */
	rmi_input->report_key = _rmi_input_report_key;
	rmi_input->report_rel = _rmi_input_report_rel;
	rmi_input->report_abs = _rmi_input_report_abs;
	rmi_input->report_state= _rmi_input_report_state;

	rmi_input->input_dev = input_allocate_device();
	if (!rmi_input->input_dev)
		return NULL;

	if (driver->set_input_params) {
		rc = driver->set_input_params(rmi_dev, rmi_input->input_dev);
		if (rc < 0) {
			dev_err(&rmi_dev->dev, "%s: Error in setting input device.\n",
				__func__);
			return NULL;
		}

		sprintf(rmi_input->input_phys, phys_fmt, dev_name(&rmi_dev->dev), 0);
		rmi_input->input_dev->phys = rmi_input->input_phys;
		rmi_input->input_dev->dev.parent = &rmi_dev->dev;
		if (data)
			input_set_drvdata(rmi_input->input_dev, data);
	}

	return rmi_input;
}
EXPORT_SYMBOL_GPL(rmi_input_dev_alloc);


int rmi_input_dev_register(struct rmi_input_dev *rmi_input)
{
	int rc;

	rc = input_register_device(rmi_input->input_dev);
	if (rc < 0) {
		input_free_device(rmi_input->input_dev);
		rmi_input->input_dev = NULL;
		return rc;
	}

	return rc;
}
EXPORT_SYMBOL_GPL(rmi_input_dev_register);

