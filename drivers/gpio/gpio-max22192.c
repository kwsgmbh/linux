// SPDX-License-Identifier: GPL-2.0
//
// Copyright (C) 2021 KWS Computersysteme Gmbh
// Author: Frank Bintakies <fbintakies@kws-computer.de>
//
// GPIO driver for MAXIM 22192

#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/crc4.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/of_gpio.h>
#include <linux/mfd/max22192.h>
#include <linux/interrupt.h>



static char getCRC(char data2, char data1, char data0)
{
    int length = 19; //19-bit data
    int i;
    char crc_init = 0x07; //5-bit init word, constant, 00111
    char crc_poly = 0x35; //6-bit polynomial, constan, 110101
    char crc_step;
    char crc_result;
    char tmp;
    unsigned int datainput;

    //construct 24-bit data frame
    datainput = (unsigned int)((data2 << (int)16) + (data1 << (int)8) + data0);
    //append 5-bit init word to first 19-bit data
    datainput = (datainput & 0xffffe0) + crc_init;

    //first step, get crc_step 0
    tmp = (char)((datainput & 0xfc0000) >> (int)18); //crc_step 0 = data[18:13]
    //next crc_step = crc_step[5] = 0 ? (crc_step[5:0] ^ crc_poly) : crc_step[5:0]
    if ((tmp & 0x20) == 0x20)
    {
        crc_step = (char)(tmp ^ crc_poly);
    }
    else
    {
        crc_step = tmp;
    }

    //step 1 - 18
    for (i = 0; i < length - 1; i++)
    {
        //append next data bit to previous crc_step[4:0], {crc_step[4:0], next data bit}
        tmp = (char)(((crc_step & 0x1f) << (int)1) + ((datainput >> (int)(length - 2 - i)) & 0x01));
        //next crc_step = crc_step[5] = 0 ? (crc_step[5:0] ^ crc_poly) : crc_step[5:0]
        if ((tmp & 0x20) == 0x20)
        {
            crc_step = (char)(tmp ^ crc_poly);
        }
        else
        {
            crc_step = tmp;
        }
    }

    crc_result = (char)(crc_step & 0x1f); //crc_result = crc_Step[4:0]
    return crc_result;
}

static int max22192_spi_write(struct max22192_chip* max, unsigned int reg, unsigned int val)
{
    int res;
    char buffer[3];

    buffer[0] = reg | MAX22192_WRITE_REGISTER;
    buffer[1] = val;
    buffer[2] = getCRC(buffer[0], buffer[1], 0x00);

    max->xfer.tx_buf = &buffer;
    max->xfer.len = sizeof(buffer);

    spi_message_init(&max->spi_msg);
    spi_message_add_tail(&max->xfer, &max->spi_msg);
    res = spi_sync(max->spi, &max->spi_msg);
    if (res)
    {
        printk("SPI transmit error\n");
        return res;
    }
    return res;
}

static int max22192_read_spi(struct max22192_chip* max, unsigned int reg)
{
    int ret;
    char buffer[3];

    buffer[0] = reg;
    buffer[1] = 0xff;
    buffer[2] = getCRC(buffer[0], buffer[1], 0x00);

    max->xfer.tx_buf = &buffer;
    max->xfer.len = sizeof(buffer);

    spi_message_init(&max->spi_msg);
    spi_message_add_tail(&max->xfer, &max->spi_msg);
    ret = spi_sync(max->spi, &max->spi_msg);
    if (ret)
    {
        printk("SPI receive error\n");
        return ret;
    }
    return ret;
}

static bool max22192_fault_detection(struct max22192_chip* max, int chipnum, int wordlen)
{
    char status, wbg_status;
    char crc;
    char in;
    int fault_gpio;
    int ret;

    fault_gpio = gpio_get_value(max->fault_gpio);

    if (!fault_gpio)
    {

        in = ((char*)max->xfer.rx_buf)[chipnum * wordlen];
        status = ((char*)max->xfer.rx_buf)[chipnum * wordlen + 1];
        crc = ((char*)max->xfer.rx_buf)[chipnum * wordlen + 2];

        if (status & MAX22192_BIT_WBG)
        {
            dev_err(&max->spi->dev, "chip %d: Wire Break Fault\n", chipnum);
            ret = max22192_read_spi(max, MAX22192_REG_WIREBREAK);
            wbg_status = ((char*)max->xfer.rx_buf)[chipnum * wordlen + 1];
            dev_err(&max->spi->dev, "chip %d: Wirebreak at %x \n", chipnum, wbg_status);
        }

        if (status & MAX22192_BIT_24M)
        {
            dev_err(&max->spi->dev, "chip %d: 24V M Fault\n", chipnum);
            max->undervolt1 = 1;
        }

        if (status & MAX22192_BIT_24L)
        {
            dev_err(&max->spi->dev, "chip %d: 24V L Fault\n", chipnum);
            max->undervolt2 = 1;
        }

        if (status & MAX22192_BIT_ALRMT1)
        {
            dev_err(&max->spi->dev, "chip %d: overtemperature T1 (115 C) Fault\n", chipnum);
            max->overtemp1 = 1;
        }

        if (status & MAX22192_BIT_ALRMT2)
        {
            dev_err(&max->spi->dev, "chip %d: overtemperatur T2 (144 C)Fault\n", chipnum);
            max->overtemp2 = 1;
        }

        if (status & MAX22192_BIT_FAULT2)
        {
            dev_err(&max->spi->dev, "chip %d: Fault 2\n", chipnum);
            max->fault = 1;
        }

        if (status & MAX22192_BIT_POR)
        {
            dev_err(&max->spi->dev, "chip %d: POR Fault\n", chipnum);
            max->por = 1;
        }

        if (status & MAX22192_BIT_CRC)
        {
            dev_err(&max->spi->dev, "chip %d: CRC Fault\n", chipnum);
            max->crc_error = 1;
        }

        if (crc != getCRC(in, status, crc))
        {
            dev_err(&max->spi->dev, "chip %d: CRC receive fault\n", chipnum);
            max->crc_error = 1;
        }

        return 1;
    }
    else
    {
        in = ((char*)max->xfer.rx_buf)[chipnum * wordlen];
        status = ((char*)max->xfer.rx_buf)[chipnum * wordlen + 1];
        crc = ((char*)max->xfer.rx_buf)[chipnum * wordlen + 2];
        if (crc != getCRC(in, status, crc))
        {
            dev_err(&max->spi->dev, "chip %d: CRC receive fault\n", chipnum);
            max->crc_error = 1;
            return 1;
        }
        else
        {
            return 0;
        }
    }
}

static unsigned int max22192_wordlen(struct max22192_chip* max22192)
{
    return max22192->mode == STATUS_BYTE_ENABLED ? 2 : 1;
}

static int max22192_get_direction(struct gpio_chip* gpio, unsigned int offset)
{
    return 1; /* always in */
}

static int max22192_direction_input(struct gpio_chip* gpio, unsigned int offset)
{
    return 0;
}

static int max22192_direction_output(struct gpio_chip* gpio,
    unsigned int offset, int value)
{
    return -EINVAL;
}

static void max22192_set(struct gpio_chip* gpio, unsigned int offset, int value)
{

}

static void max22192_set_multiple(struct gpio_chip* gpio, unsigned long* mask, unsigned long* bits)
{

}

static int max22192_get(struct gpio_chip* gpio, unsigned int offset)
{
    struct max22192_chip* max22192 = gpiochip_get_data(gpio);
    int ret, chipnum, wordlen = max22192_wordlen(max22192);
    char in;

    mutex_lock(&max22192->lock);

    if (max22192->latch_gpio > 0)
        gpio_set_value(max22192->latch_gpio, 0);

    ret = max22192_read_spi(max22192, MAX22192_REG_FAULT1);
    if (ret)
        goto out_unlock;

    chipnum = offset / DEFAULT_NGPIO;

    if (max22192_fault_detection(max22192, chipnum, wordlen))
    {
        max22192->crc_error = 0;
        max22192->por = 0;
        max22192->fault = 0;
        max22192->overtemp2 = 0;
        max22192->overtemp1 = 0;
        max22192->undervolt2 = 0;
        max22192->undervolt1 = 0;
        max22192->wbg = 0;
        ret = -EIO;
        goto out_unlock;
    }

    in = ((char*)max22192->xfer.rx_buf)[chipnum * wordlen];
    ret = (in >> (offset % DEFAULT_NGPIO)) & 1;


out_unlock:
    if (max22192->latch_gpio > 0)
        gpio_set_value(max22192->latch_gpio, 1);

    mutex_unlock(&max22192->lock);
    return ret;
}

static int max22192_get_multiple(struct gpio_chip* gpio, unsigned long* mask, unsigned long* bits)
{
    struct max22192_chip* max22192 = gpiochip_get_data(gpio);
    int ret, bit = 0, wordlen = max22192_wordlen(max22192);

    mutex_lock(&max22192->lock);

    if (max22192->latch_gpio > 0)
        gpio_set_value(max22192->latch_gpio, 0);

    ret = max22192_read_spi(max22192, MAX22192_REG_FAULT1);
    if (ret)
        goto out_unlock;

    while ((bit = find_next_bit(mask, gpio->ngpio, bit)) != gpio->ngpio)
    {
        unsigned int chipnum = bit / DEFAULT_NGPIO;
        unsigned long in, shift, index;

        if (max22192_fault_detection(max22192, chipnum, wordlen))
        {
            max22192->crc_error = 0;
            max22192->por = 0;
            max22192->fault = 0;
            max22192->overtemp2 = 0;
            max22192->overtemp1 = 0;
            max22192->undervolt2 = 0;
            max22192->undervolt1 = 0;
            max22192->wbg = 0;
            ret = -EIO;
            goto out_unlock;
        }
        in = ((char*)max22192->xfer.rx_buf)[chipnum * wordlen];
        shift = round_down(bit % BITS_PER_LONG, DEFAULT_NGPIO);
        index = bit / BITS_PER_LONG;

        bits[index] &= ~(mask[index] & (0xff << shift));
        bits[index] |= mask[index] & (in << shift); /* copy bits */

        bit = (chipnum + 1) * DEFAULT_NGPIO;
    }

out_unlock:
    if (max22192->latch_gpio > 0)
        gpio_set_value(max22192->latch_gpio, 1);
    mutex_unlock(&max22192->lock);
    return ret;
}

static int max22192_write_config(struct max22192_chip* max)
{
    int ret, i, anz_config;
    int n_wirebreaks;
    unsigned int* wires;
    unsigned int wires_activ;

    n_wirebreaks = device_property_count_u32(&max->spi->dev, "wirebreak-inputs");
    mutex_lock(&max->lock);
    if (n_wirebreaks <= 0)
    {
        anz_config = sizeof(default_config) / sizeof(cfg_data_t);
        for (i = 0; i < anz_config; i++)
        {
            ret = max22192_spi_write(max, default_config[i].reg, default_config[i].data);
            if (ret)
            {
                dev_err(&max->spi->dev, "unable to write config\n");
                return ret;
            }
        }
    }
    else
    {
        wires = kcalloc(n_wirebreaks, sizeof(unsigned int), GFP_KERNEL);
        if (!wires)
        {
            dev_err(&max->spi->dev, "Error to get memory for wirebreaks\n");
            return -ENOMEM;
        }
        ret = device_property_read_u32_array(&max->spi->dev, "wirebreak-inputs", wires, n_wirebreaks);
        if (ret)
        {
            kfree(wires);
            dev_err(&max->spi->dev, "Error to get property from dts.\n");
            return -ENOMEM;
        }

        wires_activ = *wires;

        if (n_wirebreaks > 0)
        {
            for (i = 0; i < n_wirebreaks; i++)
            {
                wires_activ = wires[i];
                switch (wires_activ)
                {
                case 1:
                    ret = max22192_spi_write(max, MAX22192_FILTER_IN1, 0x18);
                    if (ret)
                    {
                        dev_err(&max->spi->dev, "unable to write Wire 1 config\n");
                        return ret;
                    }
                    break;

                case 2:
                    ret = max22192_spi_write(max, MAX22192_FILTER_IN2, 0x18);
                    if (ret)
                    {
                        dev_err(&max->spi->dev, "unable to write Wire 2 config\n");
                        return ret;
                    }
                    break;

                case 3:
                    ret = max22192_spi_write(max, MAX22192_FILTER_IN3, 0x18);
                    if (ret)
                    {
                        dev_err(&max->spi->dev, "unable to write Wire 3 config\n");
                        return ret;
                    }
                    break;

                case 4:
                    ret = max22192_spi_write(max, MAX22192_FILTER_IN4, 0x18);
                    if (ret)
                    {
                        dev_err(&max->spi->dev, "unable to write Wire 4 config\n");
                        return ret;
                    }
                    break;

                case 5:
                    ret = max22192_spi_write(max, MAX22192_FILTER_IN5, 0x18);
                    if (ret)
                    {
                        dev_err(&max->spi->dev, "unable to write Wire 5 config\n");
                        return ret;
                    }
                    break;

                case 6:
                    ret = max22192_spi_write(max, MAX22192_FILTER_IN6, 0x18);
                    if (ret)
                    {
                        dev_err(&max->spi->dev, "unable to write Wire 6 config\n");
                        return ret;
                    }
                    break;

                case 7:
                    ret = max22192_spi_write(max, MAX22192_FILTER_IN7, 0x18);
                    if (ret)
                    {
                        dev_err(&max->spi->dev, "unable to write Wire 7 config\n");
                        return ret;
                    }
                    break;

                case 8:
                    ret = max22192_spi_write(max, MAX22192_FILTER_IN8, 0x18);
                    if (ret)
                    {
                        dev_err(&max->spi->dev, "unable to write Wire 8 config\n");
                        return ret;
                    }
                    break;

                default:
                    dev_err(&max->spi->dev, "Invalid property\n");
                    break;
                }
            }

            ret = max22192_spi_write(max, MAX22192_FAULT2_EN, 0x10);
            if (ret)
            {
                dev_err(&max->spi->dev, "unable to write config\n");
                return ret;
            }

            ret = max22192_spi_write(max, MAX22192_FAULT1_EN, 0x27);
            if (ret)
            {
                dev_err(&max->spi->dev, "unable to write config\n");
                return ret;
            }

            ret = max22192_spi_write(max, MAX22192_REG_FAULT1, 0x00);
            if (ret)
            {
                dev_err(&max->spi->dev, "unable to write config\n");
                return ret;
            }
        }
    }

    mutex_unlock(&max->lock);
    max->crc_error = 0;
    max->por = 0;
    max->fault = 0;
    max->overtemp2 = 0;
    max->overtemp1 = 0;
    max->undervolt2 = 0;
    max->undervolt1 = 0;
    max->wbg = 0;

    return ret;
}

static struct max22192_platform_data* of_gpio_max22192(struct device* dev)
{
    struct max22192_platform_data* pdata;

    pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
    if (!pdata)
    {
        dev_err(dev, "flatform_data failed");
        return NULL;
    }

    pdata->base = -1;

    return pdata;

};

static int max22192_gpio_probe(struct spi_device* spi)
{
    struct max22192_platform_data* pdata;
    struct max22192_chip* max22192;
    struct device_node* pp;
    int ret;
    int n;

    pdata = dev_get_platdata(&spi->dev);
    pp = spi->dev.of_node;

    if (!pdata || !pdata->base)
    {
        pdata = of_gpio_max22192(&spi->dev);
    }

    /*
     * bits_per_word cannot be configured in platform data
     */
    spi->bits_per_word = 8;

    ret = spi_setup(spi);
    if (ret < 0)
        return ret;

    max22192 = devm_kzalloc(&spi->dev, sizeof(*max22192), GFP_KERNEL);
    if (!max22192)
        return -ENOMEM;


    max22192->nchips = 1;

    max22192->latch_gpio = of_get_named_gpio(pp, "latch-gpio", 0);
    if (max22192->latch_gpio < 0)
    {
        dev_warn(&spi->dev, "latch-gpio not defined in dts.\n");
    }
    else
    {
        ret = devm_gpio_request_one(&spi->dev, max22192->latch_gpio, GPIOF_OUT_INIT_HIGH, "MAX221912 Latch");
        if (ret)
        {
            dev_err(&spi->dev, "unable to get latch gpio\n");
            return ret;
        }
    }

    max22192->fault_gpio = of_get_named_gpio(pp, "fault-gpio", 0);
    if (max22192->fault_gpio < 0)
    {
        dev_err(&spi->dev, "Failed to retrieve fault-gpio from dts.\n");
        goto exit_destroy;
    }
    else
    {
        ret = devm_gpio_request_one(&spi->dev, max22192->fault_gpio, GPIOF_IN, "MAX221912 Fault");
        if (ret)
        {
            dev_err(&spi->dev, "unable to get fault gpio\n");
            return ret;
        }
    }

    n = BITS_TO_LONGS(max22192->nchips);
    max22192->xfer.rx_buf = devm_kcalloc(&spi->dev, 1, 2, GFP_KERNEL);

    if (!max22192->xfer.rx_buf)
        return -ENOMEM;

    mutex_init(&max22192->lock);

    spi_set_drvdata(spi, max22192);

    max22192->spi = spi;
    max22192->gpio.label = spi->modalias;
    max22192->gpio.base = pdata->base;
    max22192->gpio.ngpio = PIN_NUMER;
    max22192->gpio.owner = THIS_MODULE;
    max22192->gpio.parent = &spi->dev;

    max22192->gpio.get_direction = max22192_get_direction;
    max22192->gpio.direction_input = max22192_direction_input;
    max22192->gpio.direction_output = max22192_direction_output;
    max22192->gpio.set = max22192_set;
    max22192->gpio.set_multiple = max22192_set_multiple;
    max22192->gpio.get = max22192_get;
    max22192->gpio.get_multiple = max22192_get_multiple;

    max22192->msg = 0x00;

    ret = max22192_write_config(max22192);
    if (ret)
    {
        dev_err(&spi->dev, "Failed writing to " DRIVER_NAME ": %d\n", ret);
        goto exit_destroy;
    }

    ret = gpiochip_add_data(&max22192->gpio, max22192);
    if (ret)
    {
        dev_err(&spi->dev, "Unable to register gpiochip\n");
        goto exit_destroy;
    }

    return ret;

exit_destroy:
    mutex_destroy(&max22192->lock);
    return ret;
}

static void max22192_remove(struct spi_device* spi)
{
    struct max22192_chip* max22192 = spi_get_drvdata(spi);

    gpiochip_remove(&max22192->gpio);

    mutex_destroy(&max22192->lock);

    //return 0;
}

static const struct of_device_id MAXxx_of_match[] = {
    {.compatible = "max22192", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, MAXxx_of_match);

static const struct spi_device_id max22192_ids[] = {
    { "max22192", 0},
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, max22192_ids);

static struct spi_driver max22192_gpio_driver = {
    .driver = {
        .name = "max22192",
        .of_match_table = MAXxx_of_match,
    },
    .probe = max22192_gpio_probe,
    .remove = max22192_remove,
    .id_table = max22192_ids,
};
module_spi_driver(max22192_gpio_driver);

MODULE_DESCRIPTION("MAXIM 22192 GPIO driver");
MODULE_AUTHOR("Frank Bintakies <fbintakies@kws-computer.de");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:max22192-gpio");
