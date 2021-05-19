/**************************************************************************
 *                      raspi_gpib.c  -  description                      *
 *                           -------------------                          *
 *  This code has been developed at the Institute of Sensor and Actuator  *
 *  Systems (Technical University of Vienna, Austria) to enable the GPIO  *
 *  lines (e.g. of a raspberry pi) to function as a GPIO master device    *
 *                                                                        *
 *  begin                : March 2016                                     *
 *  copyright            : (C) 2016 Thomas Klima                          *
 *  email                : elektronomikon@gmail.com                       *
 *                                                                        *
 *  Special Thanks go to: Marcello Carla'  -  carla@fi.infn.it		  *
 *                        University of Florence - Dept. of Physics       *
 *		          for adding the interrupt-driven handshake	  *
 *                                                                        *
 *************************************************************************/

/**************************************************************************
 *                                                                        *
 *   This program is free software; you can redistribute it and/or modify *
 *   it under the terms of the GNU General Public License as published by *
 *   the Free Software Foundation; either version 2 of the License, or    *
 *   (at your option) any later version.                                  *
 *                                                                        *
 *************************************************************************/


#include "raspi_gpib.h"

MODULE_LICENSE("GPL");

struct gpio_desc *D01, *D02, *D03, *D04, *D05, *D06, *D07, *D08, *EOI, *NRFD, *IFC, *_ATN, *REN, *DAV, *NDAC, *SRQ, *ACT_LED, *PE, *DC, *TE;

/***************************************************************************
 *                                                                         *
 * READ                                                                    *
 *                                                                         *
 ***************************************************************************/


uint8_t raspi_gpib_read_byte(int *end)
{
	uint8_t data;
	struct timespec64 before, after;

	gpiod_direction_output(ACT_LED, 1);

	/* Raise NRFD, informing the talker we are ready for the byte */
	gpiod_direction_input(NRFD);

	/* Wait for DAV to go low, informing us the byte is read to be read */
	ktime_get_ts64(&before);
	while (gpiod_get_value(DAV) == 1) {
		_delay(DELAY);
		ktime_get_ts64(&after);
		if (usec_diff(&after, &before) > TIMEOUT_US) {
			dbg_printk("read_byte_timeout1\r\n");
			data = -ETIMEDOUT;
			goto out;
		}
	}

	// Assert NRFD, informing the talker to not change the data lines
	gpiod_direction_output(NRFD, 0);
	_delay(DELAY);

	// Read the data on the port, flip the bits, and read in the EOI line
	data = get_data_lines();
	*end = !gpiod_get_value(EOI);

	// Un-assert NDAC, informing talker that we have accepted the byte
	gpiod_direction_input(NDAC);

	// Wait for DAV to go high; the talkers knows that we have read the byte
	ktime_get_ts64(&before);
	while(gpiod_get_value(DAV) == 0) {
		_delay(DELAY);
		ktime_get_ts64(&after);
		if (usec_diff(&after, &before) > TIMEOUT_US) {
			dbg_printk("read_byte_timeout2\r\n");
			data = -ETIMEDOUT;
			goto out;
		}
	}

	// Get ready for the next byte by asserting NDAC
	gpiod_direction_output(NDAC, 0);
out:
	gpiod_direction_output(ACT_LED, 0);
        return data;
}

int raspi_gpib_read(gpib_board_t *board, uint8_t *buffer, size_t length, int *end, size_t *bytes_read)
{
	raspi_gpib_private_t *priv = board->private_data;
	struct timespec64 before;

	dbg_printk("RD(%ld) ", (long int)length);

	SET_DIR_READ();

	*end = 0;
	*bytes_read = 0;
	if (length == 0) return 0;

	ktime_get_ts64(&before);
	while(1) {
		_delay(DELAY);

		buffer[*bytes_read] = raspi_gpib_read_byte(end);

		if (buffer[*bytes_read] == (uint8_t)(-ETIMEDOUT))
			return *bytes_read;

		(*bytes_read)++;

		if ((*bytes_read >= length) || (*end == 1) ||
                check_for_eos(priv, buffer[*bytes_read-1])) break;

	}

	dbg_printk("\r\ngot %d bytes.\r\n", *bytes_read);

	return *bytes_read;
}

int check_for_eos(raspi_gpib_private_t *priv, uint8_t byte)
{
	static const uint8_t sevenBitCompareMask = 0x7f;

	if ((priv->eos_flags & REOS) == 0) return 0;

	if (priv->eos_flags & BIN) {
		if (priv->eos == byte)
			return 1;
	} else {
		if ((priv->eos & sevenBitCompareMask) == \
			(byte & sevenBitCompareMask))
			return 1;
	}
	return 0;
}



/***************************************************************************
 *                                                                         *
 * WRITE                                                                   *
 *                                                                         *
 ***************************************************************************/

int raspi_gpib_write(gpib_board_t *board, uint8_t *buffer, size_t length, int send_eoi, size_t *bytes_written)
{
	struct timespec64 before, after;
	size_t i = 0;
	*bytes_written = 0;

	gpiod_direction_output(ACT_LED, 1);

	dbg_printk("\r\nWR<%d> %s (", length, (send_eoi)?"w.EOI":" ");
	for (i=0; i < length; i++) {
		dbg_printk("%c=0x%x ", buffer[i], buffer[i]);
	}
	dbg_printk(")\r\n");

	SET_DIR_WRITE();
	gpiod_direction_output(DAV, 1);
	_delay(DELAY);

	ktime_get_ts64(&before);
	while ((gpiod_get_value(NRFD) == 0) || (gpiod_get_value(NDAC) == 1)) {
		_delay(DELAY);
		ktime_get_ts64(&after);
		if (usec_diff(&after, &before) > TIMEOUT_US) {
			printk("\r\nwrite timeout NDAC(%d)|NRFD(%d) S=%d!\r\n", gpiod_get_value(NDAC), gpiod_get_value(NRFD), raspi_gpib_line_status(board));
			return -ETIMEDOUT;
		}
	}

	for (i=0; i < length; i++) {
 	       gpiod_direction_input(NRFD);
		_delay(DELAY);

		if ((i >= length-1) && send_eoi) {
			gpiod_direction_output(EOI, 0);
		} else {
			gpiod_direction_output(EOI, 1);
		}

		_delay(DELAY);
		ktime_get_ts64(&before);

		while (gpiod_get_value(NRFD) == 0) {
			_delay(DELAY);
			ktime_get_ts64(&after);
			if (usec_diff(&after, &before) > TIMEOUT_US*10)
			{
				printk("\r\ntimeout NRF3(%d)@3\r\n", gpiod_get_value(NRFD));
				return -ETIMEDOUT;
			}
		}

		gpiod_direction_output(NRFD, 0);

		set_data_lines(buffer[i]);

		_delay(DELAY);

		gpiod_direction_output(DAV, 0);
		_delay(DELAY);

		ktime_get_ts64(&before);
		while (gpiod_get_value(NDAC) == 0) {
			_delay(DELAY);
			ktime_get_ts64(&after);
			if (usec_diff(&after, &before) > TIMEOUT_US)
			{
                                printk("timeout NDAC(%d)@4\r\n", gpiod_get_value(NDAC));
				return -ETIMEDOUT;
			}
		}

		gpiod_direction_output(DAV, 1);
		*bytes_written += 1;
	}

	dbg_printk("sent %d bytes.\r\n\r\n", *bytes_written);

	return i;
}

int raspi_gpib_command(gpib_board_t *board, uint8_t *buffer, size_t length, size_t *bytes_written)
{
	size_t ret,i;

	SET_DIR_WRITE();
	gpiod_direction_output(_ATN, 0);

	dbg_printk("CMD<%d>\r\n", length);
	for (i=0; i < length; i++) {
		if (buffer[i] & 0x40) {
			dbg_printk("0x%x= TLK%d", buffer[i], buffer[i]&0x1F);
		} else if (buffer[i] & 0x20) {
			dbg_printk("0x%x= LSN%d", buffer[i], buffer[i]&0x1F);
		} else {
			dbg_printk("0x%x", buffer[i]);
		}
	}

	ret = raspi_gpib_write(board, buffer, length, 0, bytes_written);
	gpiod_direction_output(_ATN, 1);

	return *bytes_written;
}


/***************************************************************************
 *                                                                         *
 * STATUS Management                                                       *
 *                                                                         *
 ***************************************************************************/


int raspi_gpib_take_control(gpib_board_t *board, int synchronous)
{
	_delay(DELAY);
	gpiod_direction_output(_ATN, 0);
        set_bit(CIC_NUM, &board->status);
	return 0;
}

int raspi_gpib_go_to_standby(gpib_board_t *board)
{
	_delay(DELAY);
	gpiod_direction_output(_ATN, 1);
	return 0;
}

void raspi_gpib_request_system_control(gpib_board_t *board, int request_control )
{
	_delay(DELAY);
        if (request_control)
                set_bit(CIC_NUM, &board->status);
        else
                clear_bit(CIC_NUM, &board->status);
}

void raspi_gpib_interface_clear(gpib_board_t *board, int assert)
{
	_delay(DELAY);
	if (assert)
		gpiod_direction_output(IFC, 0);
	else
		gpiod_direction_output(IFC, 1);
}

void raspi_gpib_remote_enable(gpib_board_t *board, int enable)
{
	_delay(DELAY);
	if (enable) {
                set_bit(REM_NUM, &board->status);
		gpiod_direction_output(REN, 0);
	} else {
                clear_bit(REM_NUM, &board->status);
		gpiod_direction_output(REN, 1);
	}
}

int raspi_gpib_enable_eos(gpib_board_t *board, uint8_t eos_byte, int compare_8_bits)
{
	raspi_gpib_private_t *priv = board->private_data;
	dbg_printk("EOS_en ");

        priv->eos = eos_byte;
        priv->eos_flags = REOS;
        if (compare_8_bits) priv->eos_flags |= BIN;

	return 0;
}

void raspi_gpib_disable_eos(gpib_board_t *board)
{
	raspi_gpib_private_t *priv = board->private_data;
	dbg_printk("EOS_dis ");

	priv->eos_flags &= ~REOS;
}

unsigned int raspi_gpib_update_status(gpib_board_t *board, unsigned int clear_mask )
{
	dbg_printk("\r\nUS 0x%lx mask 0x%x\r\n",board->status, clear_mask);
	board->status &= ~clear_mask;

	return board->status;
}

void raspi_gpib_primary_address(gpib_board_t *board, unsigned int address)
{
	dbg_printk("PA(%d) ", address);
	board->pad = address;
}

void raspi_gpib_secondary_address(gpib_board_t *board, unsigned int address, int enable)
{
	dbg_printk("SA(%d %d) ", address, enable);

	if (enable)
		board->sad = address;
}

int raspi_gpib_parallel_poll(gpib_board_t *board, uint8_t *result)
{
	dbg_printk("PP ");
	return 0;
}
void raspi_gpib_parallel_poll_configure(gpib_board_t *board, uint8_t config )
{
	dbg_printk("PPC ");
}
void raspi_gpib_parallel_poll_response(gpib_board_t *board, int ist )
{
}
void raspi_gpib_serial_poll_response(gpib_board_t *board, uint8_t status)
{
}
uint8_t raspi_gpib_serial_poll_status(gpib_board_t *board )
{
	return 0;
}
unsigned int raspi_gpib_t1_delay(gpib_board_t *board, unsigned int nano_sec )
{
	_delay(nano_sec/1000 + 1);

	return 0;
}

void raspi_gpib_return_to_local(gpib_board_t *board )
{
	dbg_printk("R2L\r\n ");
}

int raspi_gpib_line_status(const gpib_board_t *board )
{
	int line_status = 0x00;

	if (gpiod_get_value(REN) == 1) line_status |= BusREN;
	if (gpiod_get_value(IFC) == 1) line_status |= BusIFC;
	if (gpiod_get_value(NDAC) == 0) line_status |= BusNDAC;
	if (gpiod_get_value(NRFD) == 0) line_status |= BusNRFD;
	if (gpiod_get_value(DAV) == 0) line_status |= BusDAV;
	if (gpiod_get_value(EOI) == 0) line_status |= BusEOI;
	if (gpiod_get_value(_ATN) == 0) line_status |= BusATN;
	if (gpiod_get_value(SRQ) == 1) line_status |= BusSRQ;
	return line_status;
}


/***************************************************************************
 *                                                                         *
 * Module Management                                                       *
 *                                                                         *
 ***************************************************************************/


gpib_interface_t raspi_gpib_interface =
{
	name:		"raspi_gpib",
	attach:		raspi_gpib_attach,
	detach:		raspi_gpib_detach,
	read:		raspi_gpib_read,
	write:		raspi_gpib_write,
	command:	raspi_gpib_command,
	take_control:	raspi_gpib_take_control,
	go_to_standby:	raspi_gpib_go_to_standby,
	request_system_control:	raspi_gpib_request_system_control,
	interface_clear:	raspi_gpib_interface_clear,
	remote_enable:	raspi_gpib_remote_enable,
	enable_eos:	raspi_gpib_enable_eos,
	disable_eos:	raspi_gpib_disable_eos,
	parallel_poll:	raspi_gpib_parallel_poll,
	parallel_poll_configure:	raspi_gpib_parallel_poll_configure,
	parallel_poll_response:	raspi_gpib_parallel_poll_response,
	line_status:	raspi_gpib_line_status,
	update_status:	raspi_gpib_update_status,
	primary_address:	raspi_gpib_primary_address,
	secondary_address:	raspi_gpib_secondary_address,
	serial_poll_response:	raspi_gpib_serial_poll_response,
	serial_poll_status:	raspi_gpib_serial_poll_status,
	t1_delay: raspi_gpib_t1_delay,
	return_to_local: raspi_gpib_return_to_local,
};

static int allocate_private(gpib_board_t *board)
{
	board->private_data = kmalloc(sizeof(raspi_gpib_private_t), GFP_KERNEL);
	if (board->private_data == NULL)
		return -1;
	memset(board->private_data, 0, sizeof(raspi_gpib_private_t));
	return 0;
}

static void free_private(gpib_board_t *board)
{
	if (board->private_data) {
		kfree(board->private_data);
		board->private_data = NULL;
	}
}

struct timespec64 last_irq;

irqreturn_t raspi_gpib_interrupt(int irq, void *arg PT_REGS_ARG)
{
        unsigned long flags;
	struct timespec64 current_time;

        local_irq_save(flags);

        ktime_get_ts64(&current_time);
	if (usec_diff(&current_time, &last_irq) < IRQ_DEBOUNCE_US) {
		return IRQ_NONE;
	}

	dbg_printk("IRQ! (last was %ld ms ago)\r\n", (long int)msec_diff(&current_time, &last_irq));

        local_irq_restore(flags);

	ktime_get_ts64(&last_irq);

        return IRQ_HANDLED;
}

int raspi_gpib_attach(gpib_board_t *board, const gpib_board_config_t *config)
{
	raspi_gpib_private_t *raspi_gpib_priv;

	dbg_printk("ATTACH \r\n");

	board->status = 0;
	if (allocate_private(board))
		return -ENOMEM;
	raspi_gpib_priv = board->private_data;

	SET_DIR_WRITE();

	raspi_gpib_priv->irq = gpiod_to_irq(SRQ);

	if (request_irq(raspi_gpib_priv->irq, raspi_gpib_interrupt, IRQF_TRIGGER_FALLING, "gpib_gpio", board)) {
		printk("gpib: can't request IRQ %d\n", raspi_gpib_priv->irq);
		return -1;
	}
	dbg_printk("IRQ=%d registered\r\n", raspi_gpib_priv->irq);
	ktime_get_ts64(&last_irq); // initialize debounce

	return 0;
}

void raspi_gpib_detach(gpib_board_t *board)
{
	raspi_gpib_private_t *raspi_gpib_priv = board->private_data;

	dbg_printk("DETACH");

	if (raspi_gpib_priv->irq) {
		free_irq(raspi_gpib_priv->irq, board);
	}

	free_private(board);
}

static int __init raspi_gpib_init_module(void)
{
	int ret = 0;

	D01 = gpio_to_desc(D01_gpio_nr);
	D02 = gpio_to_desc(D02_gpio_nr);
	D03 = gpio_to_desc(D03_gpio_nr);
	D04 = gpio_to_desc(D04_gpio_nr);
	D05 = gpio_to_desc(D05_gpio_nr);
	D06 = gpio_to_desc(D06_gpio_nr);
	D07 = gpio_to_desc(D07_gpio_nr);
	D08 = gpio_to_desc(D08_gpio_nr);

	EOI = gpio_to_desc(EOI_gpio_nr);
	NRFD = gpio_to_desc(NRFD_gpio_nr);
	IFC = gpio_to_desc(IFC_gpio_nr);
	_ATN = gpio_to_desc(ATN_gpio_nr);
	REN = gpio_to_desc(REN_gpio_nr);
	DAV = gpio_to_desc(DAV_gpio_nr);
	NDAC = gpio_to_desc(NDAC_gpio_nr);
	SRQ = gpio_to_desc(SRQ_gpio_nr);
	PE = gpio_to_desc(PE_gpio_nr);
	DC = gpio_to_desc(DC_gpio_nr);
	TE = gpio_to_desc(TE_gpio_nr);
	ACT_LED = gpio_to_desc(ACT_LED_gpio_nr);

	SET_DIR_READ();
	gpiod_direction_input(IFC);
	gpiod_direction_input(_ATN);
	gpiod_direction_input(REN);
	gpiod_direction_input(SRQ);
	gpiod_direction_output(ACT_LED, 0);

	dbg_printk("gpib_gpio module loaded!\r\n");

	gpib_register_driver(&raspi_gpib_interface, THIS_MODULE);

	return ret;
}

static void __exit raspi_gpib_exit_module(void)
{
	// release GPIOs
	gpiod_put(D01);
	gpiod_put(D02);
	gpiod_put(D03);
	gpiod_put(D04);
	gpiod_put(D05);
	gpiod_put(D06);
	gpiod_put(D07);
	gpiod_put(D08);
	gpiod_put(EOI);
	gpiod_put(NRFD);
	gpiod_put(IFC);
	gpiod_put(_ATN);
	gpiod_put(REN);
	gpiod_put(DAV);
	gpiod_put(NDAC);
	gpiod_put(SRQ);
	gpiod_put(PE);
	gpiod_put(DC);
	gpiod_put(TE);
	gpiod_put(ACT_LED);

	dbg_printk("gpib_gpio module unloaded!");

	gpib_unregister_driver(&raspi_gpib_interface);
}

module_init(raspi_gpib_init_module);
module_exit(raspi_gpib_exit_module);



/***************************************************************************
 *                                                                         *
 * UTILITY Functions                                                       *
 *                                                                         *
 ***************************************************************************/

void _delay(uint16_t delay)
{
	struct timespec64 before, after;
	ktime_get_ts64(&before);

	while(1) {
		ktime_get_ts64(&after);
		if (usec_diff(&after, &before) > delay) {
			break;
		}
	}
}

inline long int usec_diff (struct timespec64 * a, struct timespec64 * b)
{
        return ((a->tv_sec - b->tv_sec)*1000000 +
                (a->tv_nsec - b->tv_nsec)/1000);
}

inline long int msec_diff (struct timespec64 * a, struct timespec64 * b)
{
        return ((a->tv_sec - b->tv_sec)*1000 +
                (a->tv_nsec - b->tv_nsec)/1000000);
}

inline int sec_diff (struct timespec64 * a, struct timespec64 * b)
{
        return ((a->tv_sec - b->tv_sec) +
                (a->tv_nsec - b->tv_nsec)/1000000000);
}

void set_data_lines(uint8_t byte)
{
	gpiod_direction_output(D01, !(byte & 0x01));
	gpiod_direction_output(D02, !(byte & 0x02));
	gpiod_direction_output(D03, !(byte & 0x04));
	gpiod_direction_output(D04, !(byte & 0x08));
	gpiod_direction_output(D05, !(byte & 0x10));
	gpiod_direction_output(D06, !(byte & 0x20));
	gpiod_direction_output(D07, !(byte & 0x40));
	gpiod_direction_output(D08, !(byte & 0x80));
}

uint8_t get_data_lines(void)
{
	uint8_t ret = 0;
	set_data_lines_input();
	_delay(DELAY);
	ret += gpiod_get_value(D01) * 0x01;
	ret += gpiod_get_value(D02) * 0x02;
	ret += gpiod_get_value(D03) * 0x04;
	ret += gpiod_get_value(D04) * 0x08;
	ret += gpiod_get_value(D05) * 0x10;
	ret += gpiod_get_value(D06) * 0x20;
	ret += gpiod_get_value(D07) * 0x40;
	ret += gpiod_get_value(D08) * 0x80;
	return ~ret;
}

void set_data_lines_input(void)
{
	gpiod_direction_input(D01);
	gpiod_direction_input(D02);
	gpiod_direction_input(D03);
	gpiod_direction_input(D04);
	gpiod_direction_input(D05);
	gpiod_direction_input(D06);
	gpiod_direction_input(D07);
	gpiod_direction_input(D08);
}

inline void SET_DIR_WRITE(void)
{
	_delay(DELAY);

	// select write mode on the SN75161/160 driver ic's
	gpiod_direction_output(DC, 0);
	gpiod_direction_output(PE, 1);
	gpiod_direction_output(TE, 1);

	gpiod_direction_output(DAV, 1);
	gpiod_direction_output(EOI, 1);

	gpiod_direction_input(NRFD);
	gpiod_direction_input(NDAC);
	set_data_lines(0);
}

inline void SET_DIR_READ(void)
{
	_delay(DELAY);
	// select read mode on the SN75161/160 driver ic's
	gpiod_direction_output(DC, 1);
	gpiod_direction_output(PE, 0);
	gpiod_direction_output(TE, 0);

	gpiod_direction_input(DAV);
	gpiod_direction_input(EOI);

	gpiod_direction_output(NRFD, 0);
	gpiod_direction_output(NDAC, 0);
	set_data_lines_input();
}
