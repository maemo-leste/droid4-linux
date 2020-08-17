// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * Touchscreen Virutal Button Input Driver
 *
 * Copyright (C) 2020 Carl Klemm <carl@uvos.xyz>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/input/mt.h>
#include <linux/device/bus.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#define EVENT_QUEUE_SIZE 32

struct touchscreen_button {
	u32 x;
	u32 y;
	u32 width;
	u32 height;
	u32 keycode;
	u8 depressed;
};

struct touchscreen_button_map {
	struct touchscreen_button *buttons;
	u32 count;
	struct device_node *ts_node;
};

struct event {
	unsigned int type;
	unsigned int code;
	int value;
};

struct event_queue {
	struct event events[EVENT_QUEUE_SIZE];
	unsigned int lastindex;
};

struct touchscreen_buttons {
	struct device *dev;
	struct input_dev *buttons_idev;
	struct input_dev *filtered_ts_idev;
	struct touchscreen_button_map *map;
	struct input_handler *handler;
	struct input_handle *ts_handle;
	struct event_queue queue;
	struct workqueue_struct *workqueue;
	struct work_struct open_task;
	struct work_struct close_task;
	struct work_struct merge_task;
	struct mutex mutex;
};

static const struct input_device_id touchscreen_buttons_ids[] = {
	{
	 .flags = INPUT_DEVICE_ID_MATCH_EVBIT,
	 .evbit = {BIT_MASK(EV_ABS)},
	 },
	{
	 .flags = INPUT_DEVICE_ID_MATCH_EVBIT,
	 .evbit = {BIT_MASK(EV_KEY)},
	 },
	{
	 .flags = INPUT_DEVICE_ID_MATCH_EVBIT,
	 .evbit = {BIT_MASK(EV_SYN)},
	 },
	{},
};

static int touchscreen_buttons_process_syn(const struct event_queue *queue,
					   const struct touchscreen_button_map
					   *map, struct input_dev *idev)
{
	u32 i;
	int x, y, ret, pressed;

	x = INT_MIN;
	y = INT_MIN;
	pressed = -1;
	ret = 0;

	for (i = 0; i < queue->lastindex; ++i) {
		const struct event *ev;

		ev = &queue->events[i];
		if (ev->type == EV_ABS && ev->code == ABS_X)
			x = ev->value;
		else if (ev->type == EV_ABS && ev->code == ABS_Y)
			y = ev->value;
		else if (ev->type == EV_KEY && ev->code == BTN_TOUCH)
			pressed = ev->value;
	}

	for (i = 0; i < map->count; ++i) {
		struct touchscreen_button *button = &map->buttons[i];

		if (pressed == 1 &&
		    button->x <= x &&
		    button->y <= y &&
		    button->width + button->x >= x &&
		    button->height + button->y >= y && button->depressed == 0) {
			input_report_key(idev, button->keycode, 1);
			button->depressed = 1;
			ret = 1;
		} else if (button->depressed == 1) {
			if (pressed == 0) {
				input_report_key(idev, button->keycode, 0);
				button->depressed = 0;
			}
			ret = 2;
		}
	}

	if (ret != 0) {
		input_event(idev, EV_SYN, SYN_REPORT, 0);
	} else if (ret == 0) {
		bool buttonpressed = false;

		for (i = 0; i < map->count; ++i)
			buttonpressed = buttonpressed
			    || map->buttons[i].depressed;
		if (buttonpressed)
			ret = 3;
	}

	return ret;
}

static void touchscreen_buttons_resend_events(const struct event_queue *queue,
					      struct input_dev *idev)
{
	u32 i;

	for (i = 0; i < queue->lastindex; ++i)
		input_event(idev, queue->events[i].type, queue->events[i].code,
			    queue->events[i].value);
	input_event(idev, EV_SYN, SYN_REPORT, 0);
}

static void touchscreen_buttons_copy_mt_slots(struct input_dev *target,
					      struct input_dev *source)
{
	if (source->mt && target->mt
	    && source->mt->num_slots == target->mt->num_slots) {
		memcpy(target->mt->slots, source->mt->slots,
		       sizeof(struct input_mt_slot) * source->mt->num_slots);
	}
}

static void touchscreen_buttons_input_event(struct input_handle *handle,
					    unsigned int type,
					    unsigned int code, int value)
{
	struct touchscreen_buttons *buttons;

	buttons = handle->private;

	if (type == EV_SYN && code == SYN_REPORT) {
		if (touchscreen_buttons_process_syn(&buttons->queue,
						    buttons->map,
						    buttons->buttons_idev) == 0)
			touchscreen_buttons_resend_events(&buttons->queue,
							  buttons->filtered_ts_idev);
		buttons->queue.lastindex = 0;
	} else if (buttons->queue.lastindex < EVENT_QUEUE_SIZE) {
		buttons->queue.events[buttons->queue.lastindex].type = type;
		buttons->queue.events[buttons->queue.lastindex].code = code;
		buttons->queue.events[buttons->queue.lastindex].value = value;
		++buttons->queue.lastindex;
	} else {
		dev_warn(buttons->dev,
			 "event_qeue overrun, will not capture events until next SYN_REPORT\n");
	}
}

static void touchscreen_buttons_merge_capabilitys(struct input_dev *target,
						  struct input_dev *source)
{
	unsigned int i;

	mutex_lock(&target->mutex);
	mutex_lock(&source->mutex);
	for (i = 0; i < BITS_TO_LONGS(INPUT_PROP_CNT); ++i)
		target->propbit[i] = target->propbit[i] | source->propbit[i];
	for (i = 0; i < BITS_TO_LONGS(EV_CNT); ++i)
		target->evbit[i] = target->evbit[i] | source->evbit[i];
	for (i = 0; i < BITS_TO_LONGS(KEY_CNT); ++i)
		target->keybit[i] = target->keybit[i] | source->keybit[i];
	for (i = 0; i < BITS_TO_LONGS(REL_CNT); ++i)
		target->relbit[i] = target->relbit[i] | source->relbit[i];
	for (i = 0; i < BITS_TO_LONGS(ABS_CNT); ++i)
		target->absbit[i] = target->absbit[i] | source->absbit[i];
	for (i = 0; i < BITS_TO_LONGS(MSC_CNT); ++i)
		target->mscbit[i] = target->mscbit[i] | source->mscbit[i];
	for (i = 0; i < BITS_TO_LONGS(LED_CNT); ++i)
		target->ledbit[i] = target->ledbit[i] | source->ledbit[i];
	for (i = 0; i < BITS_TO_LONGS(SND_CNT); ++i)
		target->sndbit[i] = target->sndbit[i] | source->sndbit[i];
	for (i = 0; i < BITS_TO_LONGS(FF_CNT); ++i)
		target->ffbit[i] = target->ffbit[i] | source->ffbit[i];
	for (i = 0; i < BITS_TO_LONGS(SW_CNT); ++i)
		target->swbit[i] = target->swbit[i] | source->swbit[i];
	mutex_unlock(&source->mutex);
	mutex_unlock(&target->mutex);

	if (*source->evbit & (1 << EV_ABS)) {
		input_alloc_absinfo(target);
		mutex_lock(&target->mutex);
		mutex_lock(&source->mutex);
		for (i = 0; i < ABS_CNT; ++i)
			target->absinfo[i] = source->absinfo[i];
		mutex_unlock(&source->mutex);
		mutex_unlock(&target->mutex);
		if (source->mt) {
			input_mt_init_slots(target, source->mt->num_slots,
					    source->mt->flags);
			mutex_lock(&target->mutex);
			mutex_lock(&source->mutex);
			touchscreen_buttons_copy_mt_slots(target, source);
			mutex_unlock(&source->mutex);
			mutex_unlock(&target->mutex);
		}
	}
}

static void merge_task_handler(struct work_struct *work)
{
	struct touchscreen_buttons *buttons =
	    container_of(work, struct touchscreen_buttons, merge_task);


	mutex_lock(&buttons->mutex);
	if (buttons->ts_handle && buttons->ts_handle->dev)
		touchscreen_buttons_merge_capabilitys(buttons->filtered_ts_idev,
						      buttons->ts_handle->dev);
	mutex_unlock(&buttons->mutex);
}

static void close_task_handler(struct work_struct *work)
{
	struct touchscreen_buttons *buttons =
	    container_of(work, struct touchscreen_buttons, close_task);

	mutex_lock(&buttons->mutex);
	if (buttons && buttons->ts_handle && buttons->ts_handle->open != 0)
		input_close_device(buttons->ts_handle);
	mutex_unlock(&buttons->mutex);
}

static void open_task_handler(struct work_struct *work)
{
	struct touchscreen_buttons *buttons =
	    container_of(work, struct touchscreen_buttons, open_task);
	int error;

	mutex_lock(&buttons->mutex);
	if (buttons && buttons->ts_handle) {
		error = input_open_device(buttons->ts_handle);
		if (error) {
			dev_err(buttons->dev,
				"Failed to open input device, error %d\n",
				error);
			input_unregister_handle(buttons->ts_handle);
			kfree(buttons->ts_handle);
			buttons->ts_handle = NULL;
		}
	}
	mutex_unlock(&buttons->mutex);
}

static int touchscreen_buttons_input_connect(struct input_handler *handler,
					     struct input_dev *dev,
					     const struct input_device_id *id)
{
	struct touchscreen_buttons *buttons;

	buttons = handler->private;

	mutex_lock(&buttons->mutex);

	if ((!buttons->ts_handle
	     && device_match_of_node(&dev->dev, buttons->map->ts_node))
	    || (dev->dev.parent
		&& device_match_of_node(dev->dev.parent,
					buttons->map->ts_node))) {
		int error;

		dev_info(buttons->dev, "Binding to device: %s\n",
			 dev_name(&dev->dev));

		buttons->ts_handle =
		    kzalloc(sizeof(*buttons->ts_handle), GFP_KERNEL);
		if (!buttons->ts_handle) {
			mutex_unlock(&buttons->mutex);
			return -ENOMEM;
		}

		buttons->ts_handle->dev = dev;
		buttons->ts_handle->handler = handler;
		buttons->ts_handle->name = "touchscreen-buttons";
		buttons->ts_handle->private = handler->private;
		buttons->queue.lastindex = 0;

		error = input_register_handle(buttons->ts_handle);
		if (error) {
			dev_err(buttons->dev,
				"Failed to register input handler, error %d\n",
				error);
			kfree(buttons->ts_handle);
			buttons->ts_handle = NULL;
			mutex_unlock(&buttons->mutex);
			return error;
		}

		queue_work(buttons->workqueue, &buttons->merge_task);

		if (buttons->filtered_ts_idev->users > 0
		    && buttons->ts_handle->open == 0)
			queue_work(buttons->workqueue, &buttons->open_task);
	}

	mutex_unlock(&buttons->mutex);
	return 0;
}

static void touchscreen_buttons_input_disconnect(struct input_handle *handle)
{
	struct touchscreen_buttons *buttons;

	buttons = handle->private;

	mutex_lock(&buttons->mutex);
	if (handle == buttons->ts_handle) {
		input_close_device(handle);
		input_unregister_handle(handle);
		kfree(handle);
		buttons->ts_handle = NULL;
		dev_info(buttons->dev,
			 "Touchscreen device disconnected buttons disabled\n");
	} else {
		dev_err(buttons->dev,
			"Unknown device disconnected, %p should be %p", handle,
			buttons->ts_handle);
	}
	mutex_unlock(&buttons->mutex);
}

static struct touchscreen_button_map
*touchscreen_buttons_get_devtree_pdata(struct device *dev)
{
	struct touchscreen_button_map *map;
	struct fwnode_handle *child_node;
	struct device_node *node;
	int i;

	map = devm_kzalloc(dev, sizeof(*map), GFP_KERNEL);
	if (!map)
		return ERR_PTR(-ENOMEM);

	map->count = device_get_child_node_count(dev);
	if (map->count == 0)
		return ERR_PTR(-ENODEV);

	map->buttons = devm_kcalloc(dev, map->count, sizeof(*map->buttons), GFP_KERNEL);
	if (!map->buttons)
		return ERR_PTR(-ENOMEM);

	node = dev->of_node;
	map->ts_node = of_parse_phandle(node, "touchscreen_phandle", 0);
	if (!map->ts_node) {
		dev_err(dev, "touchscreen_phandle node missing\n");
		return ERR_PTR(-ENODEV);
	}

	dev_info(dev, "Device_node name: %s\n", map->ts_node->name);

	i = 0;
	device_for_each_child_node(dev, child_node) {
		struct touchscreen_button *button;

		button = &map->buttons[i];

		fwnode_property_read_u32(child_node, "x-position", &button->x);
		fwnode_property_read_u32(child_node, "y-position", &button->y);
		fwnode_property_read_u32(child_node, "x-size", &button->width);
		fwnode_property_read_u32(child_node, "y-size", &button->height);
		fwnode_property_read_u32(child_node, "keycode",
					 &button->keycode);
		dev_info(dev,
			 "Adding button at x=%u y=%u size %u x %u keycode=%u\n",
			 button->x, button->y, button->width, button->height,
			 button->keycode);
		++i;
	}
	return map;
}

static int touchscreen_buttons_idev_opened(struct input_dev *idev)
{
	struct touchscreen_buttons *buttons;

	buttons = dev_get_drvdata(idev->dev.parent);

	mutex_lock(&buttons->mutex);
	if (buttons && buttons->ts_handle) {
		if (buttons->ts_handle->open == 0) {
			queue_work(buttons->workqueue, &buttons->open_task);
			dev_dbg(idev->dev.parent, "idev opened\n");
		} else {
			dev_info(idev->dev.parent, "idev allready opened\n");
		}
	} else {
		dev_warn(idev->dev.parent,
			 "Input device opend but touchscreen not opened. %p %p\n",
			 buttons, buttons->ts_handle);
	}
	mutex_unlock(&buttons->mutex);
	return 0;
}

static void touchscreen_buttons_idev_closed(struct input_dev *idev)
{
	struct touchscreen_buttons *buttons;

	buttons = dev_get_drvdata(idev->dev.parent);

	mutex_lock(&buttons->mutex);
	if (buttons && buttons->ts_handle && buttons->ts_handle->open != 0) {
		queue_work(buttons->workqueue, &buttons->close_task);
		dev_dbg(idev->dev.parent, "idev closed\n");
	}
	mutex_unlock(&buttons->mutex);
}

static int touchscreen_buttons_probe(struct platform_device *pdev)
{
	struct touchscreen_buttons *buttons;
	int error, i;

	buttons = devm_kzalloc(&pdev->dev, sizeof(*buttons), GFP_KERNEL);
	if (!buttons)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, buttons);

	buttons->workqueue =
	    create_singlethread_workqueue("touchscreen-buttons-workqueue");
	INIT_WORK(&buttons->merge_task, merge_task_handler);
	INIT_WORK(&buttons->open_task, open_task_handler);
	INIT_WORK(&buttons->close_task, close_task_handler);

	mutex_init(&buttons->mutex);

	buttons->queue.lastindex = 0;
	buttons->dev = &pdev->dev;

	buttons->map = touchscreen_buttons_get_devtree_pdata(&pdev->dev);
	if (IS_ERR(buttons->map))
		return PTR_ERR(buttons->map);

	/*filtered touchscreen device */
	buttons->filtered_ts_idev = input_allocate_device();
	if (!buttons->filtered_ts_idev)
		return -ENOMEM;
	buttons->filtered_ts_idev->name = "Filtered Touchscreen";
	buttons->filtered_ts_idev->phys = "touchscreen-buttons/input1";
	buttons->filtered_ts_idev->dev.parent = buttons->dev;
	buttons->filtered_ts_idev->open = touchscreen_buttons_idev_opened;
	buttons->filtered_ts_idev->close = touchscreen_buttons_idev_closed;

	/*buttons input device */
	buttons->buttons_idev = input_allocate_device();
	if (!buttons->buttons_idev)
		return -ENOMEM;
	buttons->buttons_idev->name = "Touchscreen Buttons";
	buttons->buttons_idev->phys = "touchscreen-buttons/input0";
	buttons->buttons_idev->dev.parent = buttons->dev;
	for (i = 0; i < buttons->map->count; ++i)
		input_set_capability(buttons->buttons_idev, EV_KEY,
				     buttons->map->buttons[i].keycode);

	/*handler for touchscreen input device */
	buttons->handler = devm_kzalloc(&pdev->dev, sizeof(*buttons->handler), GFP_KERNEL);

	buttons->handler->event = touchscreen_buttons_input_event;
	buttons->handler->connect = touchscreen_buttons_input_connect;
	buttons->handler->disconnect = touchscreen_buttons_input_disconnect;
	buttons->handler->name = "touchscreen-buttons";
	buttons->handler->id_table = touchscreen_buttons_ids;
	buttons->handler->private = buttons;

	error = input_register_handler(buttons->handler);
	if (error) {
		dev_err(&pdev->dev, "Input handler register failed: %d\n",
			error);
		return error;
	}

	error = input_register_device(buttons->buttons_idev);
	if (error) {
		dev_err(&pdev->dev, "Input device register failed: %d\n",
			error);
		return error;
	}

	error = input_register_device(buttons->filtered_ts_idev);
	if (error) {
		dev_err(&pdev->dev, "Input device register failed: %d\n",
			error);
		return error;
	}

	return 0;
}

static int touchscreen_buttons_remove(struct platform_device *pdev)
{
	struct touchscreen_buttons *buttons = dev_get_drvdata(&pdev->dev);
	struct input_handle *ts_handle = buttons->ts_handle;

	input_unregister_device(buttons->buttons_idev);
	input_unregister_device(buttons->filtered_ts_idev);

	mutex_lock(&buttons->mutex);
	if (buttons->ts_handle) {
		if (buttons->ts_handle->open != 0)
			input_close_device(buttons->ts_handle);
		input_unregister_handle(buttons->ts_handle);
		buttons->ts_handle = NULL;
	}
	mutex_unlock(&buttons->mutex);
	input_unregister_handler(buttons->handler);

	flush_workqueue(buttons->workqueue);
	destroy_workqueue(buttons->workqueue);

	kfree(ts_handle);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id touchscreen_buttons_dt_match_table[] = {
	{.compatible = "touchscreen-buttons"},
	{},
};

MODULE_DEVICE_TABLE(of, touchscreen_buttons_dt_match_table);
#endif

static struct platform_driver touchscreen_buttons_driver = {
	.probe = touchscreen_buttons_probe,
	.remove = touchscreen_buttons_remove,
	.driver = {
		   .name = "touchscreen-buttons",
		   .of_match_table =
		   of_match_ptr(touchscreen_buttons_dt_match_table),
		   },
};

module_platform_driver(touchscreen_buttons_driver);

MODULE_ALIAS("platform:touchscreen-buttons");
MODULE_DESCRIPTION("touchscreen-buttons");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Carl Klemm <carl@uvos.xyz>");
