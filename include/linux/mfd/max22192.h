/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Defining registers address and its bit definitions of MAX22192 and MAX14915
 *
 * Copyright (C) 2021 KWS Computersysteme Gmbh. All rights reserved.
 */

#ifndef _MFD_MAX22192_H_
#define _MFD_MAX22192_H_

#include <linux/types.h>

#define DRIVER_NAME "max22192"

#define MAX22192_READ_REGISTER      0x00
#define MAX22192_WRITE_REGISTER     0x80

#define MAX22192_REG_WIREBREAK      0x00
#define MAX22192_REG_FAULT1         0x04
#define MAX22192_REG_FILTER1        0x06
#define MAX22192_REG_CFG            0x18
#define MAX22192_REG_FAULT2         0x1c
#define MAX22192_FAULT2_EN          0x1e
#define MAX22192_FAULT1_EN          0x24

 /* Error Bits for fault reg */
#define MAX22192_BIT_WBG            0x01
#define MAX22192_BIT_24M            0x02
#define MAX22192_BIT_24L            0x04
#define MAX22192_BIT_ALRMT1         0x08
#define MAX22192_BIT_ALRMT2         0x10
#define MAX22192_BIT_FAULT2         0x20
#define MAX22192_BIT_POR            0x40
#define MAX22192_BIT_CRC            0x80

#define MAX22192_CFG                0x18
#define MAX22192_CRC_INIT           0x07
#define MAX22192_CRC_POLY           0x35

#define MAX22192_FILTER_IN1         0x06
#define MAX22192_FILTER_IN2         0x08
#define MAX22192_FILTER_IN3         0x0A
#define MAX22192_FILTER_IN4         0x0C
#define MAX22192_FILTER_IN5         0x0E
#define MAX22192_FILTER_IN6         0x10
#define MAX22192_FILTER_IN7         0x12
#define MAX22192_FILTER_IN8         0x14

#define MAX22192_LED                0x20
#define MAX22192_GPO                0x22
#define MAX22192_INEN               0x1A

#define MAX22192_DIGI_INPUT         0x02

#define PIN_NUMER 8
#define DEFAULT_NGPIO 8

struct max22192_platform_data {
    /* number assigned to the first GPIO */
    unsigned	base;
};

typedef struct {
    unsigned char reg;
    unsigned char data;
}cfg_data_t;


/*default config for the MAX11292 */
cfg_data_t default_config[] =
{
    {MAX22192_FAULT2_EN, 0x10}, //enable fault detection for overtemperature
    {MAX22192_FAULT1_EN, 0x26}, //enable fault detection for wire break, voltage fault, FAULT2 cond
    {MAX22192_REG_FAULT1, 0x00}, //clear POR state
};

enum max22192_mode {
    STATUS_BYTE_ENABLED,
    STATUS_BYTE_DISABLED,
};

/**
 * struct max22192_chip - GPIO driver data
 * @gpio: GPIO controller struct
 * @lock: Protects read sequences
 * @latch_gpio: pin to latch all inputs of the chip
 * @fault_gpio: pin to show if a fault is occured
 * @nchips: number of chips in the daisy-chain
 * @mode: current mode, 0 for 16-bit, 1 for 8 bit;
 *        for simplicity, all chips in the daisy chain are assumed
 *        to use the same mode
 * @fault: bitmap signaling assertion of @fault_pins for each chip
 */
struct max22192_chip {
    struct gpio_chip gpio;
    struct mutex lock;
    int     latch_gpio;
    int     fault_gpio;
    u32 nchips;
    enum max22192_mode mode;
    struct gpio_descs* gpiod;
    unsigned long msg;
    struct spi_device* spi;
    struct spi_message spi_msg;
    struct spi_transfer xfer;
    /* faults */
    int wbg;
    int undervolt1;
    int undervolt2;
    int overtemp1;
    int overtemp2;
    int fault;
    int por;
    int crc_error;
};

#endif /* _MFD_MAX22192_H_ */
