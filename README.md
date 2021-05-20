# raspi_gpib_driver
A driver for linux-gpib to connect the Raspberry Pi to any GPIB device via its GPIO interface.

Intended to be used in conjunktion with the raspi_gpib_shield (https://github.com/elektronomikon/raspi_gpib_shield)
Preassembled devices can be found at https://elektronomikon.org

UPDATE 2021-05:
The raspi_gpib_driver has been integrated into linux-gpib (https://linux-gpib.sourceforge.io/) with Version 4.3.4 as
a general "gpib_bitbang" driver, please use the official source or package.

This repository will be preserved to maintain support for older linux-gpib Versions.

USAGE

The raspi-gpib-shield uses the gpib_bitbang driver, example gpib.conf:

https://github.com/elektronomikon/raspi_gpib_driver/blob/master/gpib.conf

(the entry "board_type" must match the name of the driver: gpib_bitbang)

To get quick access, try:
`gpib_config` to initialize linux-gpib and create /dev/gpib0
`ibterm -d<primary_addr> -s<secondary_addr>`

Thanks go to:
Marcello Carla for adding the interrupt driven handshake
Sebastian Weiss (dl3yc) for the timespec64 update

have fun,
Thomas
