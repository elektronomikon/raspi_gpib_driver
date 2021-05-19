/**************************************************************************
 *                      gpib_bitbang.c  -  description                    *
 *                           -------------------                          *
 *  This code has been developed at the Institute of Sensor and Actuator  *
 *  Systems (Technical University of Vienna, Austria) to enable the GPIO  *
 *  lines (e.g. of a raspberry pi) to function as a GPIO master device    *
 *                                                                        *
 *  begin                : March 2016                                     *
 *  copyright            : (C) 2016 Thomas Klima                          *
 *  email                : elektronomikon@gmail.com                       *
 *                                                                        *
 *************************************************************************/

/**************************************************************************
 *  Jan. 2017: widely modified to add SRQ and use interrupts for the      *
 *             long waits.                                                *
 *  Nov. 2020: update of types, structure references, gpio lines preset,  *
 *             documentation.                                             *
 *                                                                        *
 *  by:           Marcello Carla'                                         *
 *  at:           Department of Physics - University of Florence, Italy   *
 *  email:        carla@fi.infn.it                                        *
 *                                                                        *
 **************************************************************************/

/**************************************************************************
 * Mar. 2021:	switch to GPIO descriptor driver interface                *
 *		SN7516x driver option for compatability with raspi_gpib   *
 * by:		Thomas Klima                                              *
 **************************************************************************/


/**************************************************************************
 *                                                                        *
 *   This program is free software; you can redistribute it and/or modify *
 *   it under the terms of the GNU General Public License as published by *
 *   the Free Software Foundation; either version 2 of the License, or    *
 *   (at your option) any later version.                                  *
 *                                                                        *
 *************************************************************************/

/*
  not implemented:
        parallel/serial polling
        return2local?
*/

#define TIMEOUT_US 1000000
#define IRQ_DEBOUNCE_US 1000
#define DELAY 10
#define JIFFY (1000000 / HZ)

#define NAME "gpib_bitbang"
#define HERE  NAME, (char *) __FUNCTION__

#define dbg_printk(frm,...) if (debug) \
               printk(KERN_INFO "%s:%s - " frm, HERE, ## __VA_ARGS__ )
#define LINVAL gpiod_get_value(DAV),\
               gpiod_get_value(NRFD),\
               gpiod_get_value(SRQ)
#define LINFMT "DAV: %d  NRFD:%d  SRQ: %d"

#include "gpibP.h"
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>

static int sn7516x_used=0;
module_param(sn7516x_used,int,0660);

/**********************************************
 *  Signal pairing and pin wiring between the *
 *  Raspberry-Pi connector and the GPIB bus   *
 *                                            *
 *        signal           pin wiring         *
 *     GPIB  Pi-gpio     GPIB  ->  RPi        *
**********************************************/
typedef enum {
        D01_pin_nr =  20,     /*   1  ->  38  */
        D02_pin_nr =  26,     /*   2  ->  37  */
        D03_pin_nr =  16,     /*   3  ->  36  */
        D04_pin_nr =  19,     /*   4  ->  35  */
        D05_pin_nr =  13,     /*  13  ->  33  */
        D06_pin_nr =  12,     /*  14  ->  32  */
        D07_pin_nr =   6,     /*  15  ->  31  */
        D08_pin_nr =   5,     /*  16  ->  29  */
        EOI_pin_nr =   9,     /*   5  ->  21  */
        DAV_pin_nr =  10,     /*   6  ->  19  */
        NRFD_pin_nr = 24,     /*   7  ->  18  */
        NDAC_pin_nr = 23,     /*   8  ->  16  */
        IFC_pin_nr =  22,     /*   9  ->  15  */
        SRQ_pin_nr =  11,     /*  10  ->  23  */
        _ATN_pin_nr = 25,     /*  11  ->  22  */
        REN_pin_nr =  27,     /*  17  ->  13  */
/*
 *  GROUND PINS
 *    12,18,19,20,21,22,23,24  => 14,20,25,30,34,39
 */

/*
 *  These lines were used in the original project
 *  to control the external SN75160/161 driver chips.
 *  Not used in this module version, with reduced
 *  fan out; currently tested up to 4 devices.
 */
        PE_pin_nr =    7,    /*  26  ->   */
        DC_pin_nr =    8,    /*  24  ->   */
        TE_pin_nr =   18,    /*  12  ->   */
        ACT_LED_pin_nr = 4,    /*   7  ->   */
} lines_t;

/*
 * GPIO descriptors
 */

struct gpio_desc *D01, *D02, *D03, *D04, *D05, *D06, *D07, *D08, *EOI, *NRFD, *IFC, *_ATN, *REN, *DAV, *NDAC, *SRQ, *ACT_LED, *PE, *DC, *TE;


/* struct which defines private_data for gpio driver */

typedef struct
{
        int irq_NRFD;
        int irq_DAV;
        int irq_SRQ;
        uint8_t eos;       // eos character
        short eos_flags;   // eos mode
        int go_read;
        int go_write;
        short int end;
        int request;
        int count;
        uint8_t * rbuf;
        uint8_t * wbuf;
} bb_private_t;

inline long int usec_diff(struct timespec64 *a, struct timespec64 *b);
void set_data_lines(uint8_t byte);
uint8_t get_data_lines(void);
void set_data_lines_input(void);
int check_for_eos(bb_private_t *priv, uint8_t byte);

inline void SET_DIR_WRITE(void);
inline void SET_DIR_READ(void);

MODULE_LICENSE("GPL");

/****  global variables  ****/

static int debug = GPIB_CONFIG_KERNEL_DEBUG ? 1 : 0;
module_param (debug, int, S_IRUGO | S_IWUSR);

char printable (char x) {
        if (x < 32 || x > 126) return ' ';
        return x;
}

/***************************************************************************
 *                                                                         *
 * READ                                                                    *
 *                                                                         *
 ***************************************************************************/

int bb_read(gpib_board_t *board, uint8_t *buffer, size_t length,
            int *end, size_t *bytes_read)
{
        bb_private_t *priv = board->private_data;

        int retval=0;

        dbg_printk("board: %p  lock %d  length: %zu\n",
                board, mutex_is_locked(&board->user_mutex), length);

        priv->end = 0;
        priv->count = 0;
        priv->rbuf = buffer;

        if (length == 0) goto read_end;

        priv->request = length;

        SET_DIR_READ();
        udelay(DELAY);

        dbg_printk (".........." LINFMT "\n",LINVAL);

        /* interrupt wait for first DAV valid */

        priv->go_read = 1;
            gpiod_direction_input(NRFD);

        retval = wait_event_interruptible (board->wait,
                 board->status & TIMO  || priv->go_read == 0);

        if (board->status & TIMO) {
                retval = -ETIMEDOUT;
                dbg_printk ("timeout - " LINFMT "\n", LINVAL);
                goto read_end;
        }
        if (retval) goto read_end;  /* -ERESTARTSYS */

        /* poll loop for data read */

        while (1) {

                while (gpiod_get_value(DAV) && !(board->status & TIMO));

                if (board->status & TIMO) {
                        retval = -ETIMEDOUT;
                        dbg_printk ("timeout + " LINFMT "\n", LINVAL);
                        goto read_end;
                }

                gpiod_direction_output(NRFD, 0);

                priv->rbuf[priv->count++] = get_data_lines();
                priv->end = !gpiod_get_value(EOI);

                dbg_printk (LINFMT " count: %3d eoi: %d  val: %2x -> %c\n",
                            LINVAL, priv->count-1, priv->end,
                            priv->rbuf[priv->count-1],
                            printable(priv->rbuf[priv->count-1]));

                gpiod_direction_input(NDAC);

                if ((priv->count >= priv->request) || priv->end ||
                            check_for_eos(priv, priv->rbuf[priv->count-1])) {

                        dbg_printk ("wake_up with %d %d %x\n", priv->count,
                                            priv->end, priv->rbuf[priv->count-1]);
                        goto read_end;
                }

                while (!gpiod_get_value(DAV) && !(board->status & TIMO));

                if (board->status & TIMO) {
                        retval = -ETIMEDOUT;
                        dbg_printk ("timeout - " LINFMT "\n", LINVAL);
                        goto read_end;
                }

                gpiod_direction_output(NDAC, 0);
                udelay (DELAY);
                gpiod_direction_input(NRFD);
        }

read_end:
        dbg_printk("got %d bytes.\n", priv->count);
        *bytes_read = priv->count;
        *end = priv->end;

        priv->rbuf[priv->count] = 0;

        dbg_printk("return: %d  eoi: %d\n\n", retval, priv->end);

        return retval;
}

/***************************************************************************
 *                                                                         *
 *      READ interrupt routine (DAV line)                                  *
 *                                                                         *
 ***************************************************************************/

irqreturn_t bb_DAV_interrupt(int irq, void * arg)
{
        gpib_board_t * board = arg;
        bb_private_t *priv = board->private_data;

        if (!priv->go_read) return IRQ_HANDLED;
        priv->go_read = 0;

        dbg_printk ("n: %-3d"  LINFMT " val: %2x\n",
                priv->count, LINVAL, get_data_lines());

        wake_up_interruptible(&board->wait);

        return IRQ_HANDLED;
}

/***************************************************************************
 *                                                                         *
 * WRITE                                                                   *
 *                                                                         *
 ***************************************************************************/

int bb_write(gpib_board_t *board, uint8_t *buffer, size_t length,
             int send_eoi, size_t *bytes_written)
{
        size_t i = 0;
        int retval = 0;

        bb_private_t *priv = board->private_data;

        dbg_printk("board %p  lock %d  length: %zu\n",
                board, mutex_is_locked(&board->user_mutex), length);

        if (debug) {
                dbg_printk("<%zu %s>\n", length, (send_eoi)?"w.EOI":" ");
                for (i=0; i < length; i++) {
                        dbg_printk("%3zu  0x%x->%c\n", i, buffer[i],
                                    printable(buffer[i]));
                }
        }

        priv->count = 0;
        if (length == 0) goto write_end;
        priv->request = length;
        priv->end = send_eoi;

        /* interrupt wait for first NRFD valid */

        priv->go_write = 1;
        SET_DIR_WRITE();

        retval = wait_event_interruptible (board->wait,
                 gpiod_get_value(NRFD) || priv->go_write == 0 || board->status & TIMO);

        if (board->status & TIMO) {
                retval = -ETIMEDOUT;
                dbg_printk ("timeout - " LINFMT "\n", LINVAL);
                goto write_end;
        }
        if (retval) goto write_end;  /* -ERESTARTSYS */

        dbg_printk("NRFD: %d   NDAC: %d\n",
                    gpiod_get_value(NRFD), gpiod_get_value(NDAC));

        /*  poll loop for data write */

        for (i=0; i < length; i++) {

                while (!gpiod_get_value(NRFD) && !(board->status & TIMO));
                if (board->status & TIMO) {
                        retval = -ETIMEDOUT;
                        dbg_printk ("timeout - " LINFMT "\n", LINVAL);
                        goto write_end;
                }

                if ((i >= length-1) && send_eoi) {
                        gpiod_direction_output(EOI, 0);
                } else {
                        gpiod_direction_output(EOI, 1);
                }

                dbg_printk("sending %zu\n", i);

                set_data_lines(buffer[i]);
                gpiod_direction_output(DAV, 0);

                while (!gpiod_get_value(NDAC) && !(board->status & TIMO));

                dbg_printk("accepted %zu\n", i);

                udelay(DELAY);
                gpiod_direction_output(DAV, 1);
        }
        retval = i;

write_end:
        *bytes_written = i;

        dbg_printk("sent %zu bytes.\r\n\r\n", *bytes_written);

        return retval;
}

/***************************************************************************
 *                                                                         *
 *      WRITE interrupt routine (NRFD line)                                *
 *                                                                         *
 ***************************************************************************/

irqreturn_t bb_NRFD_interrupt(int irq, void * arg)
{
        gpib_board_t * board = arg;
            bb_private_t *priv = board->private_data;

        if (!priv->go_write) return IRQ_HANDLED;
        priv->go_write = 0;

        dbg_printk (" %-3d " LINFMT " val: %2x\n",
                priv->count, LINVAL, get_data_lines());

        wake_up_interruptible(&board->wait);

        return IRQ_HANDLED;
}

/***************************************************************************
 *                                                                         *
 *      interrupt routine for SRQ line                                     *
 *                                                                         *
 ***************************************************************************/

irqreturn_t bb_SRQ_interrupt(int irq, void * arg)
{
        gpib_board_t  * board = arg;

        int val = gpiod_get_value(SRQ);

        dbg_printk ("   " LINFMT " -> %d   st: %4lx\n", LINVAL,
                    val, board->status);

        if (!val) set_bit(SRQI_NUM, &board->status);  /* set_bit() is atomic */

        wake_up_interruptible(&board->wait);

        return IRQ_HANDLED;
}

int bb_command(gpib_board_t *board, uint8_t *buffer,
                   size_t length, size_t *bytes_written)
{
        size_t ret,i;

    dbg_printk("%p  %p\n", buffer, board->buffer);

        SET_DIR_WRITE();
        gpiod_direction_output(_ATN, 0);
        gpiod_direction_output(NRFD, 0);

        if (debug) {
                dbg_printk("CMD(%zu):\n", length);
                for (i=0; i < length; i++) {
            if (buffer[i] & 0x40) {
                    dbg_printk("0x%x=TLK%d\n", buffer[i], buffer[i]&0x1F);
            } else {
                    dbg_printk("0x%x=LSN%d\n", buffer[i], buffer[i]&0x1F);
                        }
                }
        }

        ret = bb_write(board, buffer, length, 0, bytes_written);
        gpiod_direction_output(_ATN, 1);

        return ret;
}

/***************************************************************************
 *                                                                         *
 * STATUS Management                                                       *
 *                                                                         *
 ***************************************************************************/


int bb_take_control(gpib_board_t *board, int synchronous)
{
        udelay(DELAY);
    dbg_printk("%d\n", synchronous);
        gpiod_direction_output(_ATN, 0);
        set_bit(CIC_NUM, &board->status);
        return 0;
}

int bb_go_to_standby(gpib_board_t *board)
{
    dbg_printk("\n");
        udelay(DELAY);
        gpiod_direction_output(_ATN, 1);
        return 0;
}

void bb_request_system_control(gpib_board_t *board, int request_control )
{
    dbg_printk("%d\n", request_control);
        udelay(DELAY);
        if (request_control)
                set_bit(CIC_NUM, &board->status);
        else
                clear_bit(CIC_NUM, &board->status);
}

void bb_interface_clear(gpib_board_t *board, int assert)
{
        udelay(DELAY);
    dbg_printk("%d\n", assert);
        if (assert)
                gpiod_direction_output(IFC, 0);
        else
                gpiod_direction_output(IFC, 1);
}

void bb_remote_enable(gpib_board_t *board, int enable)
{
    dbg_printk("%d\n", enable);
        udelay(DELAY);
        if (enable) {
                set_bit(REM_NUM, &board->status);
                gpiod_direction_output(REN, 0);
        } else {
                clear_bit(REM_NUM, &board->status);
                gpiod_direction_output(REN, 1);
        }
}

int bb_enable_eos(gpib_board_t *board, uint8_t eos_byte, int compare_8_bits)
{
        bb_private_t *priv = board->private_data;
        dbg_printk("%s\n", "EOS_en");
        priv->eos = eos_byte;
        priv->eos_flags = REOS;
        if (compare_8_bits) priv->eos_flags |= BIN;

        return 0;
}

void bb_disable_eos(gpib_board_t *board)
{
        bb_private_t *priv = board->private_data;
        dbg_printk("\n");
        priv->eos_flags &= ~REOS;
}

unsigned int bb_update_status(gpib_board_t *board, unsigned int clear_mask )
{
        dbg_printk("0x%lx mask 0x%x\n",board->status, clear_mask);
        board->status &= ~clear_mask;

        if (gpiod_get_value(SRQ)) {                    /* SRQ asserted low */
                clear_bit (SRQI_NUM, &board->status);
        } else {
                set_bit (SRQI_NUM, &board->status);
        }

        return board->status;
}

int bb_primary_address(gpib_board_t *board, unsigned int address)
{
        dbg_printk("%d\n", address);
        board->pad = address;
        return 0;
}

int bb_secondary_address(gpib_board_t *board, unsigned int address, int enable)
{
        dbg_printk("%d %d\n", address, enable);
        if (enable)
                board->sad = address;
        return 0;
}

int bb_parallel_poll(gpib_board_t *board, uint8_t *result)
{
        dbg_printk("%s\n", "not implemented");
        return 0;
}
void bb_parallel_poll_configure(gpib_board_t *board, uint8_t config )
{
        dbg_printk("%s\n", "not implemented");
}
void bb_parallel_poll_response(gpib_board_t *board, int ist )
{
}
void bb_serial_poll_response(gpib_board_t *board, uint8_t status)
{
        dbg_printk("%s\n", "not implemented");
}
uint8_t bb_serial_poll_status(gpib_board_t *board )
{
        dbg_printk("%s\n", "not implemented");
        return 0;
}
unsigned int bb_t1_delay(gpib_board_t *board, unsigned int nano_sec )
{
        udelay(nano_sec/1000 + 1);

        return 0;
}
void bb_return_to_local(gpib_board_t *board )
{
        dbg_printk("%s\n", "not implemented");
}

int bb_line_status(const gpib_board_t *board )
{
            int line_status = 0x00;

        if (gpiod_get_value(REN) == 1) line_status |= BusREN;
        if (gpiod_get_value(IFC) == 1) line_status |= BusIFC;
        if (gpiod_get_value(NDAC) == 0) line_status |= BusNDAC;
        if (gpiod_get_value(NRFD) == 0) line_status |= BusNRFD;
        if (gpiod_get_value(DAV) == 0) line_status |= BusDAV;
        if (gpiod_get_value(EOI) == 0) line_status |= BusEOI;
        if (gpiod_get_value(_ATN) == 0) line_status |= BusATN;
        if (gpiod_get_value(SRQ) == 0) line_status |= BusSRQ;

        dbg_printk("status lines: %4x\n", line_status);

            return line_status;
}

/***************************************************************************
 *                                                                         *
 * Module Management                                                       *
 *                                                                         *
 ***************************************************************************/

static int allocate_private(gpib_board_t *board)
{
        board->private_data = kmalloc(sizeof(bb_private_t), GFP_KERNEL);
        if (board->private_data == NULL)
                return -1;
        memset(board->private_data, 0, sizeof(bb_private_t));
        return 0;
}

static void free_private(gpib_board_t *board)
{
        if (board->private_data) {
                kfree(board->private_data);
                board->private_data = NULL;
        }
}

int bb_attach(gpib_board_t *board, const gpib_board_config_t *config)
{
        bb_private_t *priv;
        struct timespec64 before, after;

        dbg_printk("%s\n", "Enter ...");

        board->status = 0;
        if (allocate_private(board)) return -ENOMEM;
        priv = board->private_data;

        SET_DIR_WRITE();

        priv->irq_DAV = gpiod_to_irq(DAV);
        priv->irq_NRFD = gpiod_to_irq(NRFD);
        priv->irq_SRQ = gpiod_to_irq(SRQ);

        dbg_printk("%s:%s - IRQ's: %d %d %d\n", HERE, priv->irq_DAV, priv->irq_NRFD, priv->irq_SRQ);

        /* request DAV interrupt for read */

        gpiod_direction_input(DAV);
        ktime_get_ts64(&before);
        if (request_irq(priv->irq_DAV, bb_DAV_interrupt, IRQF_TRIGGER_FALLING,
                    "NAME", board)) {
                printk("gpib: can't request IRQ for DAV %d\n", priv->irq_DAV);
                return -1;
        }
        ktime_get_ts64(&after);
        dbg_printk("%s:%s - IRQ for DAV in %ld us\n", HERE,
                                         usec_diff(&after, &before));
        /* request NRFD interrupt for write */

        ktime_get_ts64(&before);
        if (request_irq(priv->irq_NRFD, bb_NRFD_interrupt, IRQF_TRIGGER_RISING,
                    "NAME", board)) {
                printk(KERN_ALERT "%s:%s - can't request IRQ for NRFD %d\n", HERE, priv->irq_NRFD);
                return -1;
        }
        ktime_get_ts64(&after);
        dbg_printk("%s:%s - IRQ for NRFD in %ld us\n", HERE, usec_diff(&after, &before));

        /* request SRQ interrupt for Service Request */

        ktime_get_ts64(&before);
        if (request_irq(priv->irq_SRQ, bb_SRQ_interrupt, IRQF_TRIGGER_FALLING,
                    "NAME", board)) {
                printk(KERN_ALERT "%s:%s - can't request IRQ for SRQ %d\n", HERE, priv->irq_SRQ);
                return -1;
        }
        ktime_get_ts64(&after);
        dbg_printk("%s:%s - IRQ for SRQ in %ld us\n", HERE,
                                         usec_diff(&after, &before));
        /* done */

        dbg_printk("registered board: %p with IRQ=%d %d\n", board,
                priv->irq_DAV, priv->irq_NRFD);

        return 0;
}

void bb_detach(gpib_board_t *board)
{        
    struct timespec64 before, after;
        bb_private_t *bb_priv = board->private_data;

        dbg_printk("%s\n", "enter... ");

        if (bb_priv->irq_DAV) {
        ktime_get_ts64(&before);
                free_irq(bb_priv->irq_DAV, board);
        ktime_get_ts64(&after);
        printk("%s:%s - IRQ DAV free in %ld us\n",
               HERE, usec_diff(&after, &before));
        }

        if (bb_priv->irq_NRFD) {
        ktime_get_ts64(&before);
                free_irq(bb_priv->irq_NRFD, board);
        ktime_get_ts64(&after);
        printk("%s:%s - IRQ NRFD free in %ld us\n",
               HERE, usec_diff(&after, &before));
        }

        if (bb_priv->irq_SRQ) {
        ktime_get_ts64(&before);
                free_irq(bb_priv->irq_SRQ, board);
        ktime_get_ts64(&after);
        printk("%s:%s - IRQ DAV free in %ld us\n",
               HERE, usec_diff(&after, &before));
        }

        free_private(board);
}

gpib_interface_t bb_interface =
{
        name:                     NAME,
        attach:                   bb_attach,
        detach:                   bb_detach,
        read:                     bb_read,
        write:                    bb_write,
        command:                  bb_command,
        take_control:             bb_take_control,
        go_to_standby:            bb_go_to_standby,
        request_system_control:   bb_request_system_control,
        interface_clear:          bb_interface_clear,
        remote_enable:            bb_remote_enable,
        enable_eos:               bb_enable_eos,
        disable_eos:              bb_disable_eos,
        parallel_poll:            bb_parallel_poll,
        parallel_poll_configure:  bb_parallel_poll_configure,
        parallel_poll_response:   bb_parallel_poll_response,
        line_status:              bb_line_status,
        update_status:            bb_update_status,
        primary_address:          bb_primary_address,
        secondary_address:        bb_secondary_address,
        serial_poll_response:     bb_serial_poll_response,
        serial_poll_status:       bb_serial_poll_status,
        t1_delay:                 bb_t1_delay,
        return_to_local:          bb_return_to_local,
};

static int __init bb_init_module(void)
{
	int ret = 0;

        D01 = gpio_to_desc(D01_pin_nr);
        D02 = gpio_to_desc(D02_pin_nr);
        D03 = gpio_to_desc(D03_pin_nr);
        D04 = gpio_to_desc(D04_pin_nr);
        D05 = gpio_to_desc(D05_pin_nr);
        D06 = gpio_to_desc(D06_pin_nr);
        D07 = gpio_to_desc(D07_pin_nr);
        D08 = gpio_to_desc(D08_pin_nr);

        EOI = gpio_to_desc(EOI_pin_nr);
        NRFD = gpio_to_desc(NRFD_pin_nr);
        IFC = gpio_to_desc(IFC_pin_nr);
        _ATN = gpio_to_desc(_ATN_pin_nr);
        REN = gpio_to_desc(REN_pin_nr);
        DAV = gpio_to_desc(DAV_pin_nr);
        NDAC = gpio_to_desc(NDAC_pin_nr);
        SRQ = gpio_to_desc(SRQ_pin_nr);

	if (sn7516x_used)
	{
        PE = gpio_to_desc(PE_pin_nr);
        DC = gpio_to_desc(DC_pin_nr);
        TE = gpio_to_desc(TE_pin_nr);
        gpiod_direction_output(DC, 1);
        gpiod_direction_output(PE, 0);
        gpiod_direction_output(TE, 0);
	}

        ACT_LED = gpio_to_desc(ACT_LED_pin_nr);

        SET_DIR_READ();
        gpiod_direction_input(IFC);
        gpiod_direction_input(_ATN);
        gpiod_direction_input(REN);
        gpiod_direction_input(SRQ);
        gpiod_direction_output(ACT_LED, 0);

        gpib_register_driver(&bb_interface, THIS_MODULE);

        dbg_printk("module loaded%s!", (sn7516x_used)?" with SN7516x driver support":"");
        return ret;
}

static void __exit bb_exit_module(void)
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
	if (sn7516x_used)
	{
        gpiod_put(PE);
        gpiod_put(DC);
        gpiod_put(TE);
	}
        gpiod_put(ACT_LED);

        dbg_printk("%s\n", "module unloaded!");

        gpib_unregister_driver(&bb_interface);
}

module_init(bb_init_module);
module_exit(bb_exit_module);



/***************************************************************************
 *                                                                         *
 * UTILITY Functions                                                       *
 *                                                                         *
 ***************************************************************************/

inline long int usec_diff (struct timespec64 * a, struct timespec64 * b)
{
        return ((a->tv_sec - b->tv_sec)*1000000 +
                (a->tv_nsec - b->tv_nsec)/1000);
}

int check_for_eos(bb_private_t *priv, uint8_t byte)
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
        udelay(DELAY);
	if (sn7516x_used)
	{
        gpiod_set_value(DC, 0);
        gpiod_set_value(PE, 1);
        gpiod_set_value(TE, 1);
	}
        gpiod_direction_output(DAV, 1);
        gpiod_direction_output(EOI, 1);

        gpiod_direction_input(NRFD);
        gpiod_direction_input(NDAC);
        set_data_lines(0);
}

inline void SET_DIR_READ(void)
{
        udelay(DELAY);
	if (sn7516x_used)
	{
        gpiod_set_value(DC, 1);
        gpiod_set_value(PE, 0);
        gpiod_set_value(TE, 0);
	}
        gpiod_direction_output(NRFD, 0);
        gpiod_direction_output(NDAC, 0); // Assert NDAC, informing the talker we have not yet accepted the byte

        gpiod_direction_input(DAV);
        gpiod_direction_input(EOI);
        set_data_lines_input();
}
