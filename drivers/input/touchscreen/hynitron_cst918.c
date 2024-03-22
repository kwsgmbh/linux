// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Driver for Hynitron cst918 Touchscreen
 *
 *  Copyright (c) 2024 Marcel Guenter <mguenter@kws-computersysteme.de>
 *
 *  This code is based on hynitron_core.c authored by Hynitron.
 *  Note that no datasheet was available, so much of these registers
 *  are undocumented. This is essentially a cleaned-up version of the
 *  vendor driver with support removed for hardware I cannot test and
 *  device-specific functions replated with generic functions wherever
 *  possible.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/kthread.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <asm/unaligned.h>

static struct i2c_client *i2c_client;
struct i2c_adapter *adapter;
static struct task_struct *i2c_thread;
static DECLARE_WAIT_QUEUE_HEAD(i2c_waitqueue);
static int interrupt_occurred = 0;
struct hynitron_ts_data *ts_data;
int iAnzIRQ = 0;

static int TSP_INT_GPIO = 517;
static unsigned int I2C_BUS_NUM = 0;
static unsigned int TSP_ADDR = 0x1A;


/* Per chip data */
struct hynitron_ts_chip_data {
    unsigned int max_touch_num;
    u32 ic_chkcode;
    int (*firmware_info)(struct i2c_client *client);
    int (*bootloader_enter)(struct i2c_client *client);
    int (*init_input)(struct i2c_client *client);
    void (*report_touch)(struct i2c_client *client);
};

/* Data generic to all (supported and non-supported) controllers. */
struct hynitron_ts_data {
    struct hynitron_ts_chip_data *chip;
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct touchscreen_properties prop;
    struct gpio_desc *reset_gpio;
};

/*
 * Since I have no datasheet, these values are guessed and/or assumed
 * based on observation and testing.
 */
#define CSTXXX_TOUCH_DATA_PART_REG		0x00d0
#define CSTXXX_TOUCH_DATA_FULL_REG		0x07d0
#define CSTXXX_TOUCH_DATA_CHK_VAL		0xab
#define CSTXXX_TOUCH_DATA_TOUCH_VAL		0x03
#define CSTXXX_TOUCH_DATA_STOP_CMD		0xab00d0
#define CSTXXX_TOUCH_COUNT_MASK			GENMASK(6, 0)

/*
 * The vendor driver would retry twice before failing to read or write
 * to the i2c device.
 */
static int cstxxx_i2c_write(unsigned char *buf, int len)
{
    int ret;
    int retries = 0;

    while (retries < 2) {
        ret = i2c_master_send(i2c_client, buf, len);
        if (ret == len)
            return 0;
        if (ret <= 0)
            retries++;
        else
            break;
    }

    return ret < 0 ? ret : -EIO;
}

static int cstxxx_i2c_read_register( u16 reg, u8 *val, u16 len)
{
    __le16 buf = cpu_to_le16(reg);
    struct i2c_msg msgs[] = {
        {
            .addr = i2c_client->addr,
            .flags = 0,
            .len = 2,
            .buf = (u8 *)&buf,
        },
        {
            .addr = i2c_client->addr,
            .flags = I2C_M_RD,
            .len = len,
            .buf = val,
        }
    };
    int err;
    int ret;

    ret = i2c_transfer(i2c_client->adapter, msgs, ARRAY_SIZE(msgs));
    if (ret == ARRAY_SIZE(msgs))
        return 0;

    err = ret < 0 ? ret : -EIO;
    dev_err(&i2c_client->dev, "Error reading %d bytes from 0x%04x: %d (%d)\n",
        len, reg, err, ret);

    return err;
    }

    static void cstxxx_report_contact(struct hynitron_ts_data *ts_data,
                    u8 id, unsigned int x, unsigned int y, u8 w)
    {
    input_mt_slot(ts_data->input_dev, id);
    input_mt_report_slot_state(ts_data->input_dev, MT_TOOL_FINGER, 1);
    touchscreen_report_pos(ts_data->input_dev, &ts_data->prop, x, y, true);
    input_report_abs(ts_data->input_dev, ABS_MT_TOUCH_MAJOR, w);
    }

    static int cstxxx_finish_touch_read(void)
    {
    unsigned char buf[3];
    int err;


    put_unaligned_le24(CSTXXX_TOUCH_DATA_STOP_CMD, buf);
    err = cstxxx_i2c_write(buf, 3);
    if (err) {
        dev_err(&i2c_client->dev,
            "send read touch info ending failed: %d\n", err);
        return err;
    }

    return 0;
}

static void cst9xx_touch_report(void)
{
    u8 buf[28];
    u8 finger_id, sw, w;
    unsigned int x, y;
    unsigned int touch_cnt, end_byte;
    unsigned int idx = 0;
    unsigned int i;
    int err;

    /* Read and validate the first bits of input data. */
    err = cstxxx_i2c_read_register(CSTXXX_TOUCH_DATA_PART_REG,
                        buf, 28);
    if (err ||
        buf[6] != CSTXXX_TOUCH_DATA_CHK_VAL ||
        buf[0] == CSTXXX_TOUCH_DATA_CHK_VAL) {
        dev_err(&i2c_client->dev, "cstxxx touch read failure\n");
        return;
    }

    /* Report to the device we're done reading the touch data. */
    err = cstxxx_finish_touch_read();
    if (err)
        return;

    touch_cnt = buf[5] & CSTXXX_TOUCH_COUNT_MASK;
    /*
        * Check the check bit of the last touch slot. The check bit is
        * always present after touch point 1 for valid data, and then
        * appears as the last byte after all other touch data.
        */
    if (touch_cnt > 1) {
        end_byte = touch_cnt * 5 + 2;
        if (buf[end_byte] != CSTXXX_TOUCH_DATA_CHK_VAL) {
            dev_err(&i2c_client->dev, "cstxxx touch read failure\n");
            return;
        }
    }

    /* Parse through the buffer to capture touch data. */
    for (i = 0; i < touch_cnt; i++) {
        x = ((buf[idx + 1] << 4) | ((buf[idx + 3] >> 4) & 0x0f));
        y = ((buf[idx + 2] << 4) | (buf[idx + 3] & 0x0f));
        w = (buf[idx + 4] >> 3);
        sw = (buf[idx] & 0x0f) >> 1;
        finger_id = (buf[idx] >> 4) & 0x0f;

        /* Sanity check we don't have more fingers than we expect */
        if (5 < finger_id) {
            dev_err(&i2c_client->dev, "cstxxx touch read failure\n");
            break;
        }

        /* sw value of 0 means no touch, 0x03 means touch */
        if (sw == CSTXXX_TOUCH_DATA_TOUCH_VAL)
            cstxxx_report_contact(ts_data, finger_id, x, y, w);

        idx += 5;

        /* Skip the 2 bytes between point 1 and point 2 */
        if (i == 0)
            idx += 2;
    }

    input_mt_sync_frame(ts_data->input_dev);
    input_sync(ts_data->input_dev);
}

static int cst918_input_dev_init(void)
{
    int err;

    ts_data = kzalloc(sizeof(struct hynitron_ts_data), GFP_KERNEL);

    ts_data->input_dev = input_allocate_device();
    if (!ts_data->input_dev) {
        pr_err("Failed to allocate input device\n");
        return -ENOMEM;
    }

    ts_data->input_dev->name = "Hynitron cst918 Touchscreen";
    ts_data->input_dev->phys = "input/ts";
    ts_data->input_dev->id.bustype = BUS_I2C;


    input_set_drvdata(ts_data->input_dev, ts_data);

    input_set_capability(ts_data->input_dev, EV_ABS, ABS_MT_POSITION_X);
    input_set_capability(ts_data->input_dev, EV_ABS, ABS_MT_POSITION_Y);
    input_set_abs_params(ts_data->input_dev, ABS_MT_TOUCH_MAJOR,
                    0, 255, 0, 0);

    ts_data->prop.max_x = 368;
    ts_data->prop.max_y = 448;
    input_abs_set_max(ts_data->input_dev,
                ABS_MT_POSITION_X, ts_data->prop.max_x);
    input_abs_set_max(ts_data->input_dev,
                ABS_MT_POSITION_Y, ts_data->prop.max_y);

    err = input_mt_init_slots(ts_data->input_dev,
                    5,
                    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
    if (err) {
        dev_err(&i2c_client->dev,
            "Failed to initialize input slots: %d\n", err);
        return err;
    }

    err = input_register_device(ts_data->input_dev);
    if (err) {
        pr_err("Input device registration failed: %d\n", err);
        return err;
    }

    return 0;
}

// Interrupt-Handler-Funktion
static irqreturn_t my_interrupt_handler(int irq, void *dev_id) {
    if (iAnzIRQ == 0)
    {
        iAnzIRQ++;
    }
    else
    {
        interrupt_occurred = 1;
        wake_up_interruptible(&i2c_waitqueue);
    }

    return IRQ_HANDLED;
}

static int i2c_worker_thread(void *data) {
    while (!kthread_should_stop()) {
        set_current_state(TASK_INTERRUPTIBLE);

        if (interrupt_occurred) {
            if (iAnzIRQ == 1)
            {
                unsigned short addr_list[] = { 0x1A, -1 };
                adapter = i2c_get_adapter(0);
                // Initialisierung des I2C-Client
                if (!adapter) {
                    pr_err("Could not find I2C adapter\n");
                    gpio_free(TSP_INT_GPIO);
                    return -1;
                }
                i2c_client = i2c_new_scanned_device(adapter, &(struct i2c_board_info) {
                    I2C_BOARD_INFO("cst918_touch", 0x1A)
                }, addr_list, NULL);
                i2c_put_adapter(adapter);

                if (!i2c_client) {
                    pr_err("Could not initialize I2C device\n");
                    gpio_free(TSP_INT_GPIO);
                    return -1;
                }

                cst918_input_dev_init();
                iAnzIRQ++;
            }
            else if (iAnzIRQ > 1)
            {
                cst9xx_touch_report();
            }
            interrupt_occurred = 0;
        }

        wait_event_interruptible(i2c_waitqueue, interrupt_occurred);
    }

    return 0;
}



static int __init gpio_interrupt_init(void) {
    int result;
    printk(KERN_INFO "CST918 Touchscreen Driver is loaded. TSP_INT_GPIO=%d, I2C_BUS_NUM=%d, TSP_ADDR=%d\n", TSP_INT_GPIO, I2C_BUS_NUM, TSP_ADDR);
    if (gpio_is_valid(TSP_INT_GPIO))
    {
        // Richtige Konfiguration des GPIO-Pins
        if (gpio_request(TSP_INT_GPIO, "TSP_Interrupt") < 0) {
            pr_err("Error requesting GPIO_%d\n", TSP_INT_GPIO);
            return -EIO;
        }

        if (gpio_direction_input(TSP_INT_GPIO) < 0) {
            pr_err("Error setting GPIO pin as input\n");
            gpio_free(TSP_INT_GPIO);
            return -EIO;
        }

    }
    else
    {
        pr_err("Error GPIO_%d is not valid\n", TSP_INT_GPIO);
        return -EIO;
    }

    // Erstellen des kThreads
    i2c_thread = kthread_run(i2c_worker_thread, NULL, "i2c_thread");
    if (IS_ERR(i2c_thread)) {
        pr_err("Error creating kThreads\n");
        free_irq(gpio_to_irq(TSP_INT_GPIO), NULL);
        gpio_free(TSP_INT_GPIO);
        i2c_unregister_device(i2c_client);
        return PTR_ERR(i2c_thread);
    }

    // Anfordern des IRQs und Registrieren des Interrupt-Handlers
    result = request_irq(gpio_to_irq(TSP_INT_GPIO), my_interrupt_handler, IRQF_TRIGGER_RISING, "My GPIO Interrupt", NULL);
    if (result < 0) {
        pr_err("Error requesting IRQs\n");
        gpio_free(TSP_INT_GPIO);
        return result;
    }

    pr_info("CST918 Touch Modul loaded\n");
    return 0;
}

static void __exit gpio_interrupt_exit(void) {
    kthread_stop(i2c_thread);
    free_irq(gpio_to_irq(TSP_INT_GPIO), NULL);
    gpio_free(TSP_INT_GPIO);
    i2c_unregister_device(i2c_client);

    pr_info("CST918 Touch Modul removed\n");
}

module_init(gpio_interrupt_init);
module_exit(gpio_interrupt_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marcel Guenter");
MODULE_DESCRIPTION("CST918 Touchscreen Driver must be load with parameter in the filesystem");
MODULE_VERSION("1.0");

module_param(TSP_INT_GPIO, int, 0);
MODULE_PARM_DESC(TSP_INT_GPIO, "TSP_INT_GPIO Number of GPIO for the TSP_INT");
module_param(I2C_BUS_NUM, uint, 0);
MODULE_PARM_DESC(I2C_BUS_NUM, "I2C_BUS_NUM from the system /sys/class/i2c-adapter-..");
module_param(TSP_ADDR, uint, 0);
MODULE_PARM_DESC(TSP_ADDR, "TSP_ADDR adress of the chip on the i2c bus");
