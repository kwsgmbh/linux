// SPDX-License-Identifier: GPL-2.0
//
// Copyright (C) 2021 KWS Computersysteme Gmbh
// Author: Frank Bintakies <fbintakies@kws-computer.de>
//
// GPIO driver for MAXIM 14915

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
#include <linux/mfd/max14915.h>
#include <linux/interrupt.h>

static unsigned int max14915_wordlen(struct max14915_chip *max14915)
{
	return max14915->mode == STATUS_BYTE_ENABLED ? 2 : 1;
}

static char crc5encode(char data1, char data2)
{
	int length = 19; //19-bit data
	int i;
	char data3 = 0x00;
	char crc_init = 0x07; //5-bit init word, constant, 00111
	char crc_poly = 0x35; //6-bit polynomial, constan, 110101
	char crc_step;
	char crc_result;
	char tmp;
	unsigned int datainput;

	//construct 24-bit data frame
	datainput =
		(unsigned int)((data1 << (int)16) + (data2 << (int)8) + data3);
	//append 5-bit init word to first 19-bit data
	datainput = (datainput & 0xffffe0) + crc_init;

	//first step, get crc_step 0
	tmp = (char)((datainput & 0xfc0000) >>
		     (int)18); //crc_step 0 = data[18:13]
	//next crc_step = crc_step[5] = 0 ? (crc_step[5:0] ^ crc_poly) : crc_step[5:0]
	if ((tmp & 0x20) == 0x20) {
		crc_step = (char)(tmp ^ crc_poly);
	} else {
		crc_step = tmp;
	}

	//step 1 - 18
	for (i = 0; i < length - 1; i++) {
		//append next data bit to previous crc_step[4:0], {crc_step[4:0], next data bit}
		tmp = (char)(((crc_step & 0x1f) << (int)1) +
			     ((datainput >> (int)(length - 2 - i)) & 0x01));
		//next crc_step = crc_step[5] = 0 ? (crc_step[5:0] ^ crc_poly) : crc_step[5:0]
		if ((tmp & 0x20) == 0x20) {
			crc_step = (char)(tmp ^ crc_poly);
		} else {
			crc_step = tmp;
		}
	}

	crc_result = (char)(crc_step & 0x1f); //crc_result = crc_Step[4:0]
	return crc_result;
}

static char crc5decode(char data1, char data2, struct max14915_chip *max)
{
	char crc5_start = 0x1f;
	char crc5_poly = 0x15;
	char data3 = (max->device_addr <<  6);
	char crc_result = crc5_start;
	int i;

	// DATA1
	for (i = 2; i < 8; i++) {
		if ((((data1 >> ((int)7 - i)) & 0x01) ^
		     ((crc_result & 0x10) >> 4)) >
		    0) //IF(XOR(C6,BITAND(D5;2^4)/(2^4)))
		{
			crc_result =
				(char)(crc5_poly ^
				       ((crc_result << 1) &
					0x1f)); //BITXOR($D$1; BITAND((D5*2);31))
		} else {
			crc_result =
				(char)((crc_result << 1) &
				       0x1f); // shift left, keep only lower 6 bits
		}
	}

	// DATA 2
	for (i = 0; i < 8; i++) {
		if ((((data2 >> ((int)7 - i)) & 0x01) ^
		     ((crc_result & 0x10) >> 4)) >
		    0) //IF(XOR(C6,BITAND(D5;2^4)/(2^4)))
		{
			crc_result =
				(char)(crc5_poly ^
				       ((crc_result << 1) &
					0x1f)); //BITXOR($D$1; BITAND((D5*2);31))
		} else {
			crc_result =
				(char)((crc_result << 1) &
				       0x1f); // shift left, keep only lower 6 bits
		}
	}

	// 3 extra bits set to zero
	for (i = 0; i < 3; i++) {
		if ((((data3 >> ((int)7 - i)) & 0x01) ^
		     ((crc_result & 0x10) >> 4)) >
		    0) //IF(XOR(C6,BITAND(D5;2^4)/(2^4)))
		{
			crc_result =
				(char)(crc5_poly ^
				       ((crc_result << 1) &
					0x1f)); //BITXOR($D$1; BITAND((D5*2);31))
		} else {
			crc_result =
				(char)((crc_result << 1) &
				       0x1f); // shift left, keep only lower 6 bits
		}
	}

	return crc_result;
}

static int max14915_spi_write(struct max14915_chip *max, unsigned int reg,
			      unsigned int val)
{
	int res;
	char buffer[3];

	buffer[0] =
		(char)(((max->device_addr << (int)6) + (max->burst << (int)5)) +
		       (reg << (int)1)) +
		MAX14915_WRITE_REGISTER;
	buffer[1] = val;
	buffer[2] = crc5encode(buffer[0], buffer[1]);

	max->xfer.tx_buf = &buffer;
	max->xfer.len = sizeof(buffer);

	if (0 < &max->spi_cs_gpio)
	{
	    gpio_set_value(max->spi_cs_gpio, 1);
	}
	spi_message_init(&max->spi_msg);
	spi_message_add_tail(&max->xfer, &max->spi_msg);
	res = spi_sync(max->spi, &max->spi_msg);
	if (res) {
		printk("SPI transmit error\n");
		if (0 < &max->spi_cs_gpio)
		{
		    gpio_set_value(max->spi_cs_gpio, 0);
		}
		return res;
	}
	if (0 < &max->spi_cs_gpio)
	{
	    gpio_set_value(max->spi_cs_gpio, 0);
	}
	return res;
}

static int max14915_read_spi(struct max14915_chip *max, unsigned int reg)
{
	int ret;
	char buffer[3];

	buffer[0] =
		(char)(((max->device_addr << (int)6) + (max->burst << (int)5)) +
		       (reg << (int)1)) +
		MAX14915_READ_REGISTER;
	buffer[1] = 0xff;
	buffer[2] = crc5encode(buffer[0], buffer[1]);

	max->xfer.tx_buf = &buffer;
	max->xfer.len = sizeof(buffer);

	if (0 < &max->spi_cs_gpio)
	{
	    gpio_set_value(max->spi_cs_gpio, 1);
	}
	spi_message_init(&max->spi_msg);
	spi_message_add_tail(&max->xfer, &max->spi_msg);
	ret = spi_sync(max->spi, &max->spi_msg);
	if (ret) {
		printk("SPI receive error\n");
		if (0 < &max->spi_cs_gpio)
		{
		    gpio_set_value(max->spi_cs_gpio, 0);
		}
		return ret;
	}
	if (0 < &max->spi_cs_gpio)
	{
	    gpio_set_value(max->spi_cs_gpio, 0);
	}
	return ret;
}

static bool max14915_fault_detection(struct max14915_chip *max)
{
	char status;
	char crc;
	char data;
	int ret;
	int errcnt = 0;
	int chipnum = 1;

	ret = max14915_read_spi(max, MAX14915_INTERRUPT_REGISTER);
	if (ret) {
		dev_err(&max->spi->dev, "Cannot read SPI reg\n");
		return -EIO;
	}

	status = ((char *)max->xfer.rx_buf)[0];
	data = ((char *)max->xfer.rx_buf)[1];
	crc = ((char *)max->xfer.rx_buf)[2];

	if (data & MAX14915_COMERR) {
		dev_err(&max->spi->dev, "chip %d: ComErr has occured\n",
			chipnum);
		errcnt++;
	}

	if (data & MAX14915_SUPPLYERR) {
		dev_err(&max->spi->dev, "chip %d: Chip had a Supply Error\n",
			chipnum);
		errcnt++;
	}

	if (data & MAX14915_THERR) {
		dev_err(&max->spi->dev,
			"chip %d: Chip had a thermal shutdown\n", chipnum);
		errcnt++;
	}

	if (data & MAX14915_SHTVDDFAULT) {
		dev_err(&max->spi->dev, "chip %d: Chip had a VDD error\n",
			chipnum);
		ret = max14915_read_spi(max, MAX14915_SHTVDDCHF_REGISTER);
		if (ret) {
			dev_err(&max->spi->dev, "Cannot read SPI reg\n");
			return -EIO;
		}
		dev_err(&max->spi->dev, "chip %d: VDD error at %x \n", chipnum,
			((char *)max->xfer.rx_buf)[1]);
		errcnt++;
	}

	if (data & MAX14915_OWONFAULT) {
		dev_err(&max->spi->dev,
			"chip %d: Chip had a Wire Break in ON state\n",
			chipnum);
		ret = max14915_read_spi(max, MAX14915_OWONCHF_REGISTER);
		if (ret) {
			dev_err(&max->spi->dev, "Cannot read SPI reg\n");
			return -EIO;
		}
		dev_err(&max->spi->dev, "chip %d: Wire Break at %x \n", chipnum,
			((char *)max->xfer.rx_buf)[1]);
		errcnt++;
	}

	if (data & MAX14915_OWOFFFAULT) {
		dev_err(&max->spi->dev,
			"chip %d: Chip had a Wire Break in Off state\n",
			chipnum);
		ret = max14915_read_spi(max, MAX14915_OWOFFCHF_REGISTER);
		if (ret) {
			dev_err(&max->spi->dev, "Cannot read SPI reg\n");
			return -EIO;
		}
		dev_err(&max->spi->dev, "chip %d: Wire Break at %x \n", chipnum,
			((char *)max->xfer.rx_buf)[1]);
		errcnt++;
	}

	if (data & MAX14915_CURRLIMERR) {
		dev_err(&max->spi->dev,
			"chip %d: Chip had a curr limit error\n", chipnum);
		ret = max14915_read_spi(max, MAX14915_CURRLIM_REGISTER);
		if (ret) {
			dev_err(&max->spi->dev, "Cannot read SPI reg\n");
			return -EIO;
		}
		dev_err(&max->spi->dev, "chip %d: curr limit at %x \n", chipnum,
			((char *)max->xfer.rx_buf)[1]);
		errcnt++;
	}

	if (data & MAX14915_OVERLDFAULT) {
		dev_err(&max->spi->dev, "chip %d: Chip had an overload error\n",
			chipnum);
		ret = max14915_read_spi(max, MAX14915_OVLCHF_REGISTER);
		if (ret) {
			dev_err(&max->spi->dev, "Cannot read SPI reg\n");
			return -EIO;
		}
		dev_err(&max->spi->dev, "chip %d: overload at at %x \n",
			chipnum, ((char *)max->xfer.rx_buf)[1]);
		errcnt++;
	}

	ret = max14915_read_spi(max, MAX14915_GLOBALERR_REGISTER);
	if (ret) {
		dev_err(&max->spi->dev, "Cannot read SPI reg\n");
		return -EIO;
	}

	return errcnt;
}

static int max14915_gpio_set_out_val(struct max14915_chip *max,
				     unsigned int pin, int value)
{
	int ret;
	int value_reg;
	int fault_gpio;
	char status;
	char data;
	char crc;

	ret = max14915_read_spi(max, MAX14915_SETOUT_REGISTER);
	if (ret) {
		dev_err(&max->spi->dev, "Cannot read SPI reg\n");
		return -EIO;
	}

	status = ((char *)max->xfer.rx_buf)[0];
	data = ((char *)max->xfer.rx_buf)[1];
	crc = ((char *)max->xfer.rx_buf)[2];

	fault_gpio = gpio_get_value(max->fault_gpio);
	if (fault_gpio) {
		if ((crc & 0x1F) == crc5decode(status, data, max)) {
			switch (pin) {
			case 0:
				value_reg = 0x01;
				break;
			case 1:
				value_reg = 0x02;
				break;
			case 2:
				value_reg = 0x04;
				break;
			case 3:
				value_reg = 0x08;
				break;
			case 4:
				value_reg = 0x10;
				break;
			case 5:
				value_reg = 0x20;
				break;
			case 6:
				value_reg = 0x40;
				break;
			case 7:
				value_reg = 0x80;
				break;
			default:
				dev_err(&max->spi->dev, "Invalid Pin\n");
				break;
			}

			if (value) {
				ret = max14915_spi_write(
					max, MAX14915_SETOUT_REGISTER,
					(data | value_reg));
				fault_gpio = gpio_get_value(max->fault_gpio);
				if (ret) {
					dev_err(&max->spi->dev,
						"Cannot set Output\n");
					return -EIO;
				} else if (fault_gpio) {
					dev_err(&max->spi->dev,
						"Cannot set Output (fault gpio)\n");
                    max14915_fault_detection(max);
					return -EIO;
				} else {
					return 0;
				}

			} else {
				ret = max14915_spi_write(
					max, MAX14915_SETOUT_REGISTER,
					(data & (~value_reg)));
				if (ret) {
					dev_err(&max->spi->dev,
						"Cannot set Output\n");
					return -EIO;
				} else if (fault_gpio) {
					dev_err(&max->spi->dev,
						"Cannot set Output (fault gpio)\n");
                    max14915_fault_detection(max);
					return -EIO;
				} else {
					return 0;
				}
			}
		} else {
			dev_err(&max->spi->dev, "CRC error\n");
			return -EIO;
		}
	} else {
		if ((crc & 0x1F) != crc5decode(status, data, max)) {
			dev_err(&max->spi->dev, "CRC error\n");
		}

		ret = max14915_fault_detection(max);
		if (ret) {
			return -EIO;
		}

		/*WORKAROUND*/
		switch (pin) {
		case 0:
			value_reg = 0x01;
			break;
		case 1:
			value_reg = 0x02;
			break;
		case 2:
			value_reg = 0x04;
			break;
		case 3:
			value_reg = 0x08;
			break;
		case 4:
			value_reg = 0x10;
			break;
		case 5:
			value_reg = 0x20;
			break;
		case 6:
			value_reg = 0x40;
			break;
		case 7:
			value_reg = 0x80;
			break;
		default:
			dev_err(&max->spi->dev, "Invalid Pin\n");
			break;
		}

		if (value) {
			ret = max14915_spi_write(max, MAX14915_SETOUT_REGISTER,
						 (data | value_reg));
			fault_gpio = gpio_get_value(max->fault_gpio);
			if (ret) {
				dev_err(&max->spi->dev, "Cannot set Output\n");
				return -EIO;
			} else if (fault_gpio) {
				dev_err(&max->spi->dev, "Cannot set Output (fault gpio)\n");
				return -EIO;
			} else {
				return 0;
			}

		} else {
			ret = max14915_spi_write(max, MAX14915_SETOUT_REGISTER,
						 (data & (~value_reg)));
			if (ret) {
				dev_err(&max->spi->dev, "Cannot set Output\n");
				return -EIO;
			} else if (fault_gpio) {
				dev_err(&max->spi->dev, "Cannot set Output (fault gpio)\n");
				return -EIO;
			} else {
				return 0;
			}
		}
		/***********/
	}
	return 0;
}

static int max14915_get_direction(struct gpio_chip *gpio, unsigned int offset)
{
	return 0; /*always output */
}

static int max14915_direction_input(struct gpio_chip *gpio, unsigned int offset)
{
	return -EINVAL;
}

static int max14915_direction_output(struct gpio_chip *gpio,
				     unsigned int offset, int value)
{
	return 0;
}

static int max14915_get(struct gpio_chip *gpio, unsigned int offset)
{
	struct max14915_chip *max14915 = gpiochip_get_data(gpio);
	int ret, chipnum, wordlen = max14915_wordlen(max14915);
	char in, crc, status;
	int fault_gpio;

	mutex_lock(&max14915->lock);

	ret = max14915_read_spi(max14915, MAX14915_SETOUT_REGISTER);
	if (ret)
		goto out_unlock;

	fault_gpio = gpio_get_value(max14915->fault_gpio);
	dev_err(&max14915->spi->dev, "FAULT PIN: %d \n", fault_gpio);
	if (fault_gpio) {
		chipnum = offset / DEFAULT_NGPIO;

		status = ((char *)max14915->xfer.rx_buf)[chipnum * wordlen];
		in = ((char *)max14915->xfer.rx_buf)[chipnum * wordlen + 1];
		crc = ((char *)max14915->xfer.rx_buf)[chipnum * wordlen + 2];
		if ((crc & 0x1F) == crc5decode(status, in, max14915)) {
			ret = (in >> (offset % DEFAULT_NGPIO)) & 1;
		} else {
			dev_err(&max14915->spi->dev, "CRC error\n");
			ret = -EIO;
		}
	} else {
		chipnum = offset / DEFAULT_NGPIO;

		status = ((char *)max14915->xfer.rx_buf)[chipnum * wordlen];
		in = ((char *)max14915->xfer.rx_buf)[chipnum * wordlen + 1];
		crc = ((char *)max14915->xfer.rx_buf)[chipnum * wordlen + 2];

		if ((crc & 0x1F) != crc5decode(status, in, max14915)) {
			dev_err(&max14915->spi->dev, "CRC error\n");
			ret = -EIO;
			goto out_unlock;
		}

		ret = max14915_fault_detection(max14915);
		if (ret) {
			ret = -EIO;
			goto out_unlock;
		}

		ret = (in >> (offset % DEFAULT_NGPIO)) & 1;
	}

out_unlock:
	mutex_unlock(&max14915->lock);
	return ret;
}

static int max14915_get_multiple(struct gpio_chip *gpio, unsigned long *mask,
				 unsigned long *bits)
{
	struct max14915_chip *max14915 = gpiochip_get_data(gpio);
	int ret, bit = 0, wordlen = max14915_wordlen(max14915);
	char in, crc, status;
	unsigned int chipnum = bit / DEFAULT_NGPIO;
	int fault_gpio;

	mutex_lock(&max14915->lock);

	fault_gpio = gpio_get_value(max14915->fault_gpio);
	if (!fault_gpio) {
		ret = max14915_fault_detection(max14915);
		if (ret) {
			ret = -EIO;
			goto out_unlock;
		}

		ret = max14915_read_spi(max14915, MAX14915_SETOUT_REGISTER);
		if (ret)
			goto out_unlock;

		status = ((char *)max14915->xfer.rx_buf)[chipnum * wordlen];
		in = ((char *)max14915->xfer.rx_buf)[chipnum * wordlen + 1];
		crc = ((char *)max14915->xfer.rx_buf)[chipnum * wordlen + 2];

		if ((crc & 0x1F) == crc5decode(status, in, max14915)) {
			while ((bit = find_next_bit(mask, gpio->ngpio, bit)) !=
			       gpio->ngpio) {
				unsigned long in, shift, index;
				chipnum = bit / DEFAULT_NGPIO;

				in = ((char *)max14915->xfer
					      .rx_buf)[chipnum * wordlen + 1];
				shift = round_down(bit % BITS_PER_LONG,
						   DEFAULT_NGPIO);
				index = bit / BITS_PER_LONG;

				bits[index] &= ~(mask[index] & (0xff << shift));
				bits[index] |= mask[index] &
					       (in << shift); /* copy bits */

				bit = (chipnum + 1) * DEFAULT_NGPIO;
			}
		} else {
			dev_err(&max14915->spi->dev, "CRC error\n");
			ret = -EIO;
		}
	} else {
		status = ((char *)max14915->xfer.rx_buf)[chipnum * wordlen];
		in = ((char *)max14915->xfer.rx_buf)[chipnum * wordlen + 1];
		crc = ((char *)max14915->xfer.rx_buf)[chipnum * wordlen + 2];

		if ((crc & 0x1F) != crc5decode(status, in, max14915)) {
			dev_err(&max14915->spi->dev, "CRC error\n");
			ret = -EIO;
			goto out_unlock;
		}

		ret = max14915_fault_detection(max14915);
		if (ret) {
			ret = -EIO;
			goto out_unlock;
		}
	}

out_unlock:
	mutex_unlock(&max14915->lock);
	return ret;
}

static void max14915_set(struct gpio_chip *gpio, unsigned int offset, int value)
{
	struct max14915_chip *max14915 = gpiochip_get_data(gpio);

	max14915_gpio_set_out_val(max14915, offset, value);
}

static int max14915_write_config(struct max14915_chip *max)
{
	int n_wirebreaks, n_vdd;
	int value_wire = 0x00;
	int value_vdd = 0x00;
	int value_limit_vdd = 0x00;
	int value_wires_pullup = 0x00;
	char config2;
	unsigned int *wires;
	unsigned int *vdd;
	unsigned int wires_active;
	unsigned int vdd_active;
	unsigned int vdd_limit;
	unsigned int wires_pullup;
	int ret, i;

	n_wirebreaks =
		device_property_count_u32(&max->spi->dev, "wirebreak-outputs");
	mutex_lock(&max->lock);
	if (n_wirebreaks <= 0) {
		//return 0;
	} else {
		wires = kcalloc(n_wirebreaks, sizeof(unsigned int), GFP_KERNEL);
		if (!wires) {
			dev_err(&max->spi->dev,
				"Error to get memory for wirebreak\ns");
			return -ENOMEM;
		}
		ret = device_property_read_u32_array(&max->spi->dev,
						     "wirebreak-outputs", wires,
						     n_wirebreaks);
		if (ret) {
			kfree(wires);
			dev_err(&max->spi->dev,
				"Error to get wirebreaks-outputs from dts.\n");
			return -ENOMEM;
		}

		wires_active = *wires;

		for (i = 0; i < n_wirebreaks; i++) {
			wires_active = wires[i];
			switch (wires_active) {
			case 1:
				value_wire = value_wire | 0x01;
				break;

			case 2:
				value_wire = value_wire | 0x02;
				break;

			case 3:
				value_wire = value_wire | 0x04;
				break;

			case 4:
				value_wire = value_wire | 0x08;
				break;

			case 5:
				value_wire = value_wire | 0x10;
				break;

			case 6:
				value_wire = value_wire | 0x20;
				break;

			case 7:
				value_wire = value_wire | 0x40;
				break;

			case 8:
				value_wire = value_wire | 0x80;
				break;

			default:
				dev_err(&max->spi->dev, "Invalid property\n");
				break;
			}
		}
		ret = max14915_spi_write(max, MAX14915_OWOFFEN_REGISTER,
					 value_wire);
		if (ret) {
			dev_err(&max->spi->dev,
				"unable to write OwOffEn Register\n");
			return ret;
		}

		ret = max14915_spi_write(max, MAX14915_OWONEN_REGISTER,
					 value_wire);
		if (ret) {
			dev_err(&max->spi->dev,
				"unable to write OwOnEn Register\n");
			return ret;
		}
	}

	mutex_unlock(&max->lock);
	n_vdd = device_property_count_u32(&max->spi->dev, "vdd-outputs");
	mutex_lock(&max->lock);
	if (n_vdd <= 0) {
		//return 0;
	} else {
		vdd = kcalloc(n_vdd, sizeof(unsigned int), GFP_KERNEL);
		if (!vdd) {
			dev_err(&max->spi->dev,
				"Error to get memory for VDD\n");
			return -ENOMEM;
		}

		ret = device_property_read_u32_array(&max->spi->dev,
						     "vdd-outputs", vdd, n_vdd);
		if (ret) {
			kfree(vdd);
			dev_err(&max->spi->dev,
				"Error to get property from vdd in dts\n.");
			return -ENOMEM;
		}

		vdd_active = *vdd;
		if (n_vdd > 0) {
			for (i = 0; i < n_vdd; i++) {
				vdd_active = vdd[i];
				switch (vdd_active) {
				case 1:
					value_vdd = value_vdd | 0x01;
					break;

				case 2:
					value_vdd = value_vdd | 0x02;
					break;

				case 3:
					value_vdd = value_vdd | 0x04;
					break;

				case 4:
					value_vdd = value_vdd | 0x08;
					break;

				case 5:
					value_vdd = value_vdd | 0x10;
					break;

				case 6:
					value_vdd = value_vdd | 0x20;
					break;

				case 7:
					value_vdd = value_vdd | 0x40;
					break;

				case 8:
					value_vdd = value_vdd | 0x80;
					break;

				default:
					dev_err(&max->spi->dev,
						"Invalid property\n");
					break;
				}
			}

			ret = max14915_spi_write(
				max, MAX14915_SHTVDDEN_REGISTER, value_wire);
			if (ret) {
				dev_err(&max->spi->dev,
					"unable to write OwOffEn Register\n");
				return ret;
			}
		}
	}

	mutex_unlock(&max->lock);
	device_property_read_u32(&max->spi->dev, "vdd-limit", &vdd_limit);
	device_property_read_u32(&max->spi->dev, "wires_pullup", &wires_pullup);

	switch (vdd_limit) {
	case 9:
		value_limit_vdd = 0x00;
		break;

	case 10:
		value_limit_vdd = 0x01;
		break;

	case 12:
		value_limit_vdd = 0x02;
		break;

	case 14:
		value_limit_vdd = 0x03;
		break;

	default:
		dev_err(&max->spi->dev,
			"Wrong VDD Limit, setting to default 9V");
		value_limit_vdd = 0x00;
		break;
	}

	switch (wires_pullup) {
	case 20:
		value_wires_pullup = 0x00;
		break;

	case 100:
		value_wires_pullup = 0x01;
		break;

	case 300:
		value_wires_pullup = 0x02;
		break;

	case 600:
		value_wires_pullup = 0x03;
		break;

	default:
		dev_err(&max->spi->dev,
			"Wrong Wire Pullup Limit, setting to default 20uA");
		value_wires_pullup = 0x00;
		break;
	}

	config2 = config2 | (value_wires_pullup << (int)4);
	config2 = config2 | (value_limit_vdd << (int)2);

	ret = max14915_spi_write(max, MAX14915_CONFIG2_REGISTER, config2);
	if (ret) {
		dev_err(&max->spi->dev, "unable to write Config2 Register\n");
		return ret;
	}
	// return 0;

	ret = max14915_spi_write(max, MAX14915_MASK_REGISTER, 0x83);
	if (ret) {
		dev_err(&max->spi->dev,
			"unable to write MAX14915_MASK_REGISTER Register\n");
		return ret;
	}
	//return 0;

	ret = max14915_read_spi(max, MAX14915_GLOBALERR_REGISTER);
	if (ret) {
		dev_err(&max->spi->dev, "Cannot read SPI reg\n");
		return -EIO;
	}
	return 0;
}

static struct max14915_platform_data *of_gpio_max14915(struct device *dev)
{
	struct max14915_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "flatform_data failed");
		return NULL;
	}

	pdata->base = -1;

	return pdata;
};

static int max14915_gpio_probe(struct spi_device *spi)
{
	struct max14915_platform_data *pdata;
	struct max14915_chip *max14915;
	struct device_node *pp;
	int ret, n;

	pdata = dev_get_platdata(&spi->dev);
	pp = spi->dev.of_node;

	if (!pdata || !pdata->base) {
		pdata = of_gpio_max14915(&spi->dev);
		dev_err(&spi->dev, "incorrect or missing platform data\n");
	}

	max14915 = devm_kzalloc(&spi->dev, sizeof(*max14915), GFP_KERNEL);
	if (!max14915)
		return -ENOMEM;

    ret = device_property_read_u32(&spi->dev, "spi_cs_num", &max14915->spi_cs_num);

	if (0 > ret) {
		dev_err(&spi->dev, "spi_cs_num not defined in dts.\n");
	} else {
		spi->chip_select = max14915->spi_cs_num;
	}

	max14915->spi_cs_gpio = of_get_named_gpio(pp, "spi_cs_gpio", 0);

	if (0 > max14915->spi_cs_gpio) {
		dev_err(&spi->dev, "spi_cs_gpio not defined in dts.\n");
	} else {
		ret = devm_gpio_request_one(&spi->dev, max14915->spi_cs_gpio,
					    GPIOF_OUT_INIT_LOW,
					    "MAX14915 SPI_CS");
		if (ret) {
			dev_err(&spi->dev, "unable to get spi_cs gpio\n");
			return ret;
		}
	}

	/*
     * bits_per_word cannot be configured in platform data
     */
	spi->bits_per_word = 8;

	if (0 < max14915->spi_cs_gpio) {
		gpio_set_value(max14915->spi_cs_gpio, 1);
	}
	ret = spi_setup(spi);
	if (ret < 0) {
		if (0 < max14915->spi_cs_gpio)
		{
		    gpio_set_value(max14915->spi_cs_gpio, 0);
		}
		return ret;
	}

	if (0 < max14915->spi_cs_gpio)
	{
	    gpio_set_value(max14915->spi_cs_gpio, 0);
	}

	max14915->nchips = 1;
	device_property_read_u32(&spi->dev, "#device-addr",
				 &max14915->device_addr);
	device_property_read_u32(&spi->dev, "#burst", &max14915->burst);

	if (max14915->burst < 0) {
		max14915->burst = 0;
	}

	max14915->fault_gpio = of_get_named_gpio(pp, "fault-gpio", 0);
	if (max14915->fault_gpio < 0) {
		dev_err(&spi->dev, "fault-gpio not defined in dts.\n");
	} else {
		ret = devm_gpio_request_one(&spi->dev, max14915->fault_gpio,
					    GPIOF_IN, "MAX14915 Fault");
		if (ret) {
			dev_err(&spi->dev, "unable to get fault gpio\n");
			return ret;
		}
	}

	n = BITS_TO_LONGS(max14915->nchips);
	max14915->xfer.rx_buf = devm_kcalloc(&spi->dev, 1, 2, GFP_KERNEL);

	if (!max14915->xfer.rx_buf)
		return -ENOMEM;

	mutex_init(&max14915->lock);

	spi_set_drvdata(spi, max14915);

	max14915->spi = spi;
	max14915->gpio.label = spi->modalias;
	max14915->gpio.base = pdata->base;
	max14915->gpio.ngpio = PIN_NUMER;
	max14915->gpio.owner = THIS_MODULE;
	max14915->gpio.parent = &spi->dev;

	max14915->gpio.get_direction = max14915_get_direction;
	max14915->gpio.direction_input = max14915_direction_input;
	max14915->gpio.direction_output = max14915_direction_output;
	max14915->gpio.set = max14915_set;
	max14915->gpio.get = max14915_get;
	max14915->gpio.get_multiple = max14915_get_multiple;

	max14915->msg = 0x00;

	ret = max14915_write_config(max14915);
	if (ret) {
		dev_err(&spi->dev, "Failed writing to MAX14915: %d\n", ret);
		goto exit_destroy;
	}

	ret = gpiochip_add_data(&max14915->gpio, max14915);
	if (ret) {
		dev_err(&spi->dev, "Unable to register gpiochip\n");
		goto exit_destroy;
	}
	dev_err(&spi->dev, "Registered gpiochip\n");

exit_destroy:
	mutex_destroy(&max14915->lock);
	return ret;
}

static void max14915_remove(struct spi_device *spi)
{
	struct max14915_chip *max14915 = spi_get_drvdata(spi);

	gpiochip_remove(&max14915->gpio);

	mutex_destroy(&max14915->lock);

	//return 0;
}

static const struct of_device_id MAXxx_of_match[] = {
	{
		.compatible = "max14915",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, MAXxx_of_match);

static const struct spi_device_id max14915_ids[] = { { "max14915", 0 },
						     { /* sentinel */ } };
MODULE_DEVICE_TABLE(spi, max14915_ids);

static struct spi_driver max14915_gpio_driver = {
    .driver = {
        .name = "max14915-gpio",
        .of_match_table = MAXxx_of_match,
    },
    .probe = max14915_gpio_probe,
    .remove = max14915_remove,
    .id_table = max14915_ids,
};
module_spi_driver(max14915_gpio_driver);

MODULE_DESCRIPTION("MAXIM 14915 GPIO driver");
MODULE_AUTHOR("Frank Bintakies <fbintakies@kws-computer.de");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:max14915-gpio");
