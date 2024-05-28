/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Defining registers address and its bit definitions of max14915 and MAX14915
 *
 * Copyright (C) 2021 KWS Computersysteme Gmbh. All rights reserved.
 */

#ifndef _MFD_MAX14915_H_
#define _MFD_MAX14915_H_

#include <linux/types.h>

#define DRIVER_NAME "max14915"

/* max14915 Register map */
#define MAX14915_READ_REGISTER       0x00
#define MAX14915_WRITE_REGISTER      0x01

#define MAX14915_SETOUT_REGISTER     0x00
#define MAX14915_INTERRUPT_REGISTER  0x03
#define MAX14915_OVLCHF_REGISTER     0x04
#define MAX14915_CURRLIM_REGISTER    0x05
#define MAX14915_OWOFFCHF_REGISTER   0x06
#define MAX14915_OWONCHF_REGISTER    0x07
#define MAX14915_SHTVDDCHF_REGISTER  0x08
#define MAX14915_GLOBALERR_REGISTER  0x09
#define MAX14915_OWOFFEN_REGISTER    0x0A
#define MAX14915_OWONEN_REGISTER     0x0B
#define MAX14915_SHTVDDEN_REGISTER   0x0C
#define MAX14915_CONFIG1_REGISTER    0x0D
#define MAX14915_CONFIG2_REGISTER    0x0E
#define MAX14915_MASK_REGISTER       0x0F

#define MAX14915_COMERR              0x80
#define MAX14915_SUPPLYERR           0x40
#define MAX14915_THERR               0x20
#define MAX14915_SHTVDDFAULT         0x10
#define MAX14915_OWONFAULT           0x08
#define MAX14915_OWOFFFAULT          0x04
#define MAX14915_CURRLIMERR          0x02
#define MAX14915_OVERLDFAULT         0x01




#define PIN_NUMER 8
#define DEFAULT_NGPIO 8

struct max14915_platform_data {
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
    //{max14915_FAULT2_EN, 0x10}, //enable fault detection for overtemperature
    //{max14915_FAULT1_EN, 0x26}, //enable fault detection for wire break, voltage fault, FAULT2 cond
    //{max14915_REG_FAULT1, 0x00}, //clear POR state
};

enum max14915_mode {
    STATUS_BYTE_ENABLED,
    STATUS_BYTE_DISABLED,
};

/**
 * struct max14915_chip - GPIO driver data
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
struct max14915_chip {
    struct gpio_chip gpio;
    struct mutex lock;
    int fault_gpio;
    int spi_cs_gpio;
    int spi_cs_num;
    u32 nchips;
    u32 device_addr;
    u32 burst;
    enum max14915_mode mode;
    struct gpio_descs* gpiod;
    unsigned long msg;
    struct spi_device* spi;
    struct spi_message spi_msg;
    struct spi_transfer xfer;
    /* faults */
    int comerr;
    int supplyerr;
    int therr;
    int shtvddfault;
    int owonfault;
    int currlim;
    int overldfault;
};

#endif /* _MFD_MAX14915_H_ */
