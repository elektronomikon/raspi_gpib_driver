/***************************************************************************
 *                        raspi_gpib.h  -  description                     *
 *                           -------------------                           *
 *  This code has been developed at the Institute of Sensor and Actuator   *
 *  Systems (Technical University of Vienna, Austria) to enable the GPIO   *
 *  lines (e.g. of a raspberry pi) to function as a GPIO master device     *
 *                                                                         *
 *  begin                : March 2016                                      *
 *  copyright            : (C) 2016 Thomas Klima                           *
 *  email                : elektronomikon@gmail.com                        *
 *                                                                         *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _RASPI_GPIB_H
#define _RASPI_GPIB_H

#define DEBUG 1
#define TIMEOUT_US 200000
#define IRQ_DEBOUNCE_US 1000
#define DELAY 10


#include "gpibP.h"
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <linux/timer.h>

//  GPIB signal to GPIO pin-number mappings
// names for rpi from /arch/arm/boot/dts/bcm2835-rpi-b-plus.dts
typedef enum {
	D01_gpio_nr = 20,
	D02_gpio_nr = 26,
	D03_gpio_nr = 16,
	D04_gpio_nr = 19,
	D05_gpio_nr = 13,
	D06_gpio_nr = 12,
	D07_gpio_nr = 6,
	D08_gpio_nr = 5,
	EOI_gpio_nr = 9,
	NRFD_gpio_nr = 24,
	IFC_gpio_nr = 22,
	ATN_gpio_nr = 25,
	REN_gpio_nr = 27,
	DAV_gpio_nr = 10,
	NDAC_gpio_nr = 23,
	SRQ_gpio_nr = 11,
	PE_gpio_nr = 7,
	DC_gpio_nr = 8,
	TE_gpio_nr = 18,
	ACT_LED_gpio_nr = 4,
} gpio_nr_t;


// struct which defines private_data for gpio driver
typedef struct
{
	int irq;
	uint8_t eos;	// eos character
	short eos_flags; // eos mode
} raspi_gpib_private_t;

// interfaces
extern gpib_interface_t raspi_gpib_interface;

#define dbg_printk(...) {if (DEBUG) printk(KERN_INFO __VA_ARGS__);}

int raspi_gpib_attach(gpib_board_t *board, const gpib_board_config_t *config);
void raspi_gpib_detach(gpib_board_t *board);
int raspi_gpib_line_status(const gpib_board_t *board );
inline long int usec_diff(struct timespec *a, struct timespec *b);
inline long int msec_diff(struct timespec *a, struct timespec *b);
inline int sec_diff(struct timespec *a, struct timespec * b);
void set_data_lines(uint8_t byte);
uint8_t get_data_lines(void);
void set_data_lines_input(void);
int check_for_eos(raspi_gpib_private_t *priv, uint8_t byte);
irqreturn_t raspi_gpib_interrupt(int irq, void *arg PT_REGS_ARG);

void _delay(uint16_t delay);
inline void SET_DIR_WRITE(void);
inline void SET_DIR_READ(void);


#endif	// _RASPI_GPIB_H
