// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2020 Fabian Wüthrich

#include <linux/acpi.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/gpio-regulator.h>
#include <linux/regulator/machine.h>

#define CRL_MAX_CUSTOM_GPIO_AMOUNT 3

struct crl_custom_gpio {
	char name[16];
	int number;
	unsigned int val;
	unsigned int undo_val;
};

struct crlmodule_platform_data {
	unsigned short i2c_addr;
	unsigned short i2c_adapter;

	unsigned int ext_clk; /* sensor external clk */

	unsigned int lanes; /* Number of CSI-2 lanes */
	const s64 *op_sys_clock;

	int xshutdown; /* gpio */
	struct crl_custom_gpio custom_gpio[CRL_MAX_CUSTOM_GPIO_AMOUNT];
	char module_name[16]; /* module name from ACPI */
	int crl_irq_pin;
	unsigned int irq_pin_flags;
	char irq_pin_name[16];
	const char *id_string;
};

/* Data representation as it is in ACPI SSDB buffer */
struct sensor_bios_data_packed {
	u8 version;
	u8 sku;
	u8 guid_csi2[16];
	u8 devfunction;
	u8 bus;
	u32 dphylinkenfuses;
	u32 clockdiv;
	u8 link;
	u8 lanes;
	u32 csiparams[10];
	u32 maxlanespeed;
	u8 sensorcalibfileidx;
	u8 sensorcalibfileidxInMBZ[3];
	u8 romtype;
	u8 vcmtype;
	u8 platforminfo;
	u8 platformsubinfo;
	u8 flash;
	u8 privacyled;
	u8 degree;
	u8 mipilinkdefined;
	u32 mclkspeed;
	u8 controllogicid;
	u8 reserved1[3];
	u8 mclkport;
	u8 reserved2[13];
} __attribute__((__packed__));

/* Fields needed by ipu4 driver */
struct sensor_bios_data {
	struct device *dev;
	u8 link;
	u8 lanes;
	u8 vcmtype;
	u8 flash;
	u8 degree;
	u8 mclkport;
	u16 xshutdown;
};

static int read_acpi_block(struct device *dev, char *id, void *data, u32 size)
{
	union acpi_object *obj;
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	struct acpi_handle *dev_handle = ACPI_HANDLE(dev);
	int status;
	u32 buffer_length;

	status = acpi_evaluate_object(dev_handle, id, NULL, &buffer);
	if (!ACPI_SUCCESS(status))
		return -ENODEV;

	obj = (union acpi_object *)buffer.pointer;
	if (!obj || obj->type != ACPI_TYPE_BUFFER) {
		dev_err(dev, "Could't read acpi buffer\n");
		status = -ENODEV;
		goto err;
	}

	if (obj->buffer.length > size) {
		dev_err(dev, "Given buffer is too small\n");
		status = -ENODEV;
		goto err;
	}

	memcpy(data, obj->buffer.pointer, min(size, obj->buffer.length));
	buffer_length = obj->buffer.length;
	kfree(buffer.pointer);

	return buffer_length;
err:
	kfree(buffer.pointer);
	return status;
}

static int get_sensor_gpio(struct device *dev, int index)
{
	struct gpio_desc *gpiod_gpio;
	int gpio;

	gpiod_gpio = gpiod_get_index(dev, NULL, index, GPIOD_ASIS);
	if (IS_ERR(gpiod_gpio)) {
		dev_err(dev, "No gpio from index %d\n", index);
		return -ENODEV;
	}
	gpio = desc_to_gpio(gpiod_gpio);
	gpiod_put(gpiod_gpio);
	return gpio;
}

static int match_depend(struct device *dev, const void *data)
{
	return (dev && dev->fwnode == data) ? 1 : 0;
}

static int get_acpi_dep_data(struct device *dev,
			     struct sensor_bios_data *sensor)
{
	struct acpi_handle *dev_handle = ACPI_HANDLE(dev);
	struct acpi_handle_list dep_devices;
	acpi_status status;
	int i;

	if (!acpi_has_method(dev_handle, "_DEP"))
		return 0;

	status =
		acpi_evaluate_reference(dev_handle, "_DEP", NULL, &dep_devices);
	if (ACPI_FAILURE(status)) {
		dev_dbg(dev, "Failed to evaluate _DEP.\n");
		return -ENODEV;
	}

	for (i = 0; i < dep_devices.count; i++) {
		struct acpi_device *device;
		struct acpi_device_info *info;
		struct device *p_dev;
		int match;

		status = acpi_get_object_info(dep_devices.handles[i], &info);
		if (ACPI_FAILURE(status)) {
			dev_dbg(dev, "Error reading _DEP device info\n");
			continue;
		}

		match = info->valid & ACPI_VALID_HID &&
			!strcmp(info->hardware_id.string, "INT3472");

		kfree(info);

		if (!match)
			continue;

		/* Process device IN3472 created by acpi */
		if (acpi_bus_get_device(dep_devices.handles[i], &device))
			return -ENODEV;

		dev_dbg(dev, "Depend ACPI device found: %s\n",
			dev_name(&device->dev));

		p_dev = bus_find_device(&platform_bus_type, NULL,
					&device->fwnode, match_depend);
		if (p_dev) {
			dev_dbg(dev, "Dependent platform device found %s\n",
				dev_name(p_dev));
			sensor->dev = p_dev;
			/* GPIO in index 1 is fixed regulator */
			//TODO How to add regulator device?
			//create_gpio_regulator(p_dev, 1, dev_name(dev));
		}
	}
	return 0;
}

static int get_acpi_ssdb_sensor_data(struct device *dev,
				     struct sensor_bios_data *sensor)
{
	struct sensor_bios_data_packed sensor_data;
	int ret =
		read_acpi_block(dev, "SSDB", &sensor_data, sizeof(sensor_data));
	if (ret < 0)
		return ret;

	get_acpi_dep_data(dev, sensor);

	/* Xshutdown is not part of the ssdb data */
	sensor->link = sensor_data.link;
	sensor->lanes = sensor_data.lanes;
	sensor->mclkport = sensor_data.mclkport;
	sensor->flash = sensor_data.flash;
	dev_dbg(dev, "sensor acpi data: link %d, lanes %d, mclk %d, flash %d\n",
		sensor->link, sensor->lanes, sensor->mclkport, sensor->flash);
	return 0;
}

static int get_custom_gpios(struct device *dev,
			    struct crlmodule_platform_data *pdata)
{
	int i, ret, c = gpiod_count(dev, NULL) - 1;

	for (i = 0; i < c; i++) {
		ret = snprintf(pdata->custom_gpio[i].name,
			       sizeof(pdata->custom_gpio[i].name),
			       "custom_gpio%d", i);
		if (ret < 0 || ret >= sizeof(pdata->custom_gpio[i].name)) {
			dev_err(dev, "Failed to set custom gpio name\n");
			return -EINVAL;
		}
		/* First GPIO is xshutdown */
		pdata->custom_gpio[i].number = get_sensor_gpio(dev, i + 1);
		if (pdata->custom_gpio[i].number < 0) {
			dev_err(dev, "unable to get custom gpio number\n");
			return -ENODEV;
		}
		pdata->custom_gpio[i].val = 1;
		pdata->custom_gpio[i].undo_val = 0;
	}

	return 0;
}

static int ov5693_probe(struct i2c_client *client)
{
	struct sensor_bios_data sensor;
	struct crlmodule_platform_data *pdata;
	struct gpio_desc *gpio;
	int rval;

	printk("ov5693 probe\n");

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	sensor.dev = &client->dev;

	rval = get_acpi_ssdb_sensor_data(&client->dev, &sensor);

	pdata->xshutdown = get_sensor_gpio(sensor.dev, 0);
	if (pdata->xshutdown < 0) {
		rval = pdata->xshutdown;
		goto err_free_pdata;
	}

	rval = get_custom_gpios(sensor.dev, pdata);
	if (rval)
		goto err_free_pdata;

	pdata->lanes = sensor.lanes;
	pdata->ext_clk = 24000000;
	client->dev.platform_data = pdata;

	// turn on status LED
	gpio = gpio_to_desc(pdata->custom_gpio[1].number);
	gpiod_set_value(gpio, 1);

err_free_pdata:
	kfree(pdata);
	return rval;
}

static int ov5693_remove(struct i2c_client *client)
{
	printk("ov5693 removed\n");
	return 0;
}

#ifdef CONFIG_ACPI
static const struct acpi_device_id ov5693_acpi_ids[] = { { "INT33BE" }, {} };

MODULE_DEVICE_TABLE(acpi, ov5693_acpi_ids);
#endif

static struct i2c_driver ov5693_i2c_driver = {
	.driver = {
		.name = "ov5693",
		.acpi_match_table = ACPI_PTR(ov5693_acpi_ids),
	},
	.probe_new = ov5693_probe,
	.remove = ov5693_remove,
};

module_i2c_driver(ov5693_i2c_driver);

MODULE_AUTHOR("Fabian Wüthrich <me@fabwu.ch>");
MODULE_DESCRIPTION("OmniVision OV5693 sensor driver");
MODULE_LICENSE("GPL v2");
