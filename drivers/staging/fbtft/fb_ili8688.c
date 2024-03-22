// SPDX-License-Identifier: GPL-2.0+fb_ili8688
/*
 * FB driver for the ILI8688 LCD Controller
 *
 * Copyright (C) 2023 KWS Computersysteme Gmbh
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <video/mipi_display.h>

#include "fbtft.h"

#define DRVNAME     "fb_ili8688"
#define WIDTH       368
#define HEIGHT      448
#define BPP         24

/* this init sequence matches ili868 */
static const s16 default_init_sequence[] = {
    -1, 0x01,
    -2, 200,
    -1, 0xFF, 0x86, 0x88, 0x00,
    -1, 0x2A, 0x00, 0x00, 0x01, 0x6F,
    -1, 0x2B, 0x00, 0x00, 0x01, 0xBF,
    -1, 0x35, 0x02,
    -1, 0x36, 0x80,
    -1, 0x51, 0xFF,
    -1, 0x53, 0x24,
    -1, 0xC4, 0x80,
    -1, 0x3A, 0x77,
    -1, 0xFF, 0x86, 0x88, 0x00,
    -1, 0x11,
    -2, 200,
    -1, 0x29,
    -3
};

static void set_addr_win(struct fbtft_par *par, int xs, int ys, int xe, int ye)
{
    write_reg(par, MIPI_DCS_SET_COLUMN_ADDRESS,
          xs >> 8, xs & 0xFF, xe >> 8, xe & 0xFF);

    write_reg(par, MIPI_DCS_SET_PAGE_ADDRESS,
          ys >> 8, ys & 0xFF, ye >> 8, ye & 0xFF);

    write_reg(par, MIPI_DCS_WRITE_MEMORY_START);
}

static int set_var(struct fbtft_par *par)
{
    switch (par->info->var.rotate) {
    case 0:
        write_reg(par, MIPI_DCS_SET_ADDRESS_MODE,
              0x80 | (par->bgr << 3));
        break;
    case 90:
        write_reg(par, MIPI_DCS_SET_ADDRESS_MODE,
              0x20 | (par->bgr << 3));
        break;
    case 180:
        write_reg(par, MIPI_DCS_SET_ADDRESS_MODE,
              0x40 | (par->bgr << 3));
        break;
    case 270:
        write_reg(par, MIPI_DCS_SET_ADDRESS_MODE,
              0xE0 | (par->bgr << 3));
        break;
    default:
        break;
    }

    par->info->var.red.offset    = 16;
    par->info->var.red.length    = 8;
    par->info->var.green.offset  = 8;
    par->info->var.green.length  = 8;
    par->info->var.blue.offset   = 0;
    par->info->var.blue.length   = 8;

    return 0;
}

/* 18/24 bit pixel over 8-bit databus */
int fbtft_write_vmem24_bus8(struct fbtft_par *par, size_t offset, size_t len)
{
        u8 *vmem8;
        u8 *txbuf8 = par->txbuf.buf;
        size_t remain;
        size_t to_copy;
        size_t tx_array_size;
        int i;
        int ret = 0;
        size_t startbyte_size = 0;
        fbtft_par_dbg(DEBUG_WRITE_VMEM, par, "%s(offset=%zu, len=%zu)\n",
                __func__, offset, len);

        remain = len / 3;
        vmem8 = (u8 *)(par->info->screen_buffer + offset);

        gpiod_set_value(par->gpio.dc, 1);

        /* non buffered write */
        if (!par->txbuf.buf)
                return par->fbtftops.write(par, vmem8, len);

        /* buffered write, /4*4 to faster */
        tx_array_size = par->txbuf.len / 3;

        if (par->startbyte) {
                txbuf8 = par->txbuf.buf + 1;
                tx_array_size -= 1;
                *(u8 *)(par->txbuf.buf) = par->startbyte | 0x2;
                startbyte_size = 1;
        }

        while (remain) {
                to_copy = min(tx_array_size, remain);
                dev_dbg(par->info->device, "    to_copy=%zu, remain=%zu\n",
                                                to_copy, remain - to_copy);

               for (i = 0; i < to_copy/4; i++)
               {       //faster copy
                       *(u32*)(txbuf8+i*12) = *(u32*)(vmem8+i*12);
                       *(u32*)(txbuf8+4+i*12) = *(u32*)(vmem8+4+i*12);
                       *(u32*)(txbuf8+8+i*12) = *(u32*)(vmem8+8+i*12);
               }
                for(i = 0; i < to_copy; i++)
                {
                        txbuf8[i*3] = vmem8[i*3+2];
                        txbuf8[i*3+1] = vmem8[i*3+1];
                        txbuf8[i*3+2] = vmem8[i*3];
                }
                vmem8 = vmem8 + to_copy*3;
                ret = par->fbtftops.write(par, par->txbuf.buf,
                                                startbyte_size + to_copy * 3);
                if (ret < 0)
                        return ret;
                remain -= to_copy;
        }

        return ret;
}

static void fbtft_reset_im(struct fbtft_par *par)
{
    if (par->gpio.im)
    {
        fbtft_par_dbg(DEBUG_RESET, par, "%s()\n", __func__);
        gpiod_set_value(par->gpio.im, 1);
    }

    if (!par->gpio.reset)
        return;

    fbtft_par_dbg(DEBUG_RESET, par, "%s()\n", __func__);

    gpiod_set_value_cansleep(par->gpio.reset, 1);
    usleep_range(20, 40);
    gpiod_set_value_cansleep(par->gpio.reset, 0);
    msleep(120);

    gpiod_set_value_cansleep(par->gpio.cs, 1);  /* Activate chip */
}

static struct fbtft_display display = {
    .regwidth = 8,
    .width = WIDTH,
    .height = HEIGHT,
    .bpp = BPP,
    .init_sequence = default_init_sequence,
    .fbtftops = {
        .set_addr_win = set_addr_win,
        .set_var = set_var,
        .write_vmem = fbtft_write_vmem24_bus8,
        .reset = fbtft_reset_im,
    },
};

FBTFT_REGISTER_DRIVER(DRVNAME, "ilitek,ili8688", &display);

MODULE_ALIAS("spi:" DRVNAME);
MODULE_ALIAS("platform:" DRVNAME);
MODULE_ALIAS("spi:ili8688");
MODULE_ALIAS("platform:ili8688");

MODULE_DESCRIPTION("FB driver for the ILI8688 LCD Controller");
MODULE_AUTHOR("Marcel Günter");
MODULE_LICENSE("GPL");
