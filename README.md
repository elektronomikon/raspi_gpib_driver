# raspi_gpib_driver
A driver for linux-gpib to connect the Raspberry Pi to any GPIB device via its GPIO interface.

Intended to be used in conjunktion with the raspi_gpib_shield (https://github.com/elektronomikon/raspi_gpib_shield)
Preassembled devices can be found at https://elektronomikon.org

INSTALLATION

clone or download the patch in a new folder

download the current stable linux-gpib release (4.0.3 at this time)

extract linux-gpib with folder names in the new folder:

`  tar xzf linux-gpib-4.1.0.tar.gz`

configure linux-gpib:

`  cd linux-gpib-4.1.0`

`  ./configure`

this creates the needed build files. Apply the patch to add the gpio driver:

`  patch -p1 < gpio_driver_4.1.0.patch`

build linux-gpib

`  make`

`  sudo make install`


USAGE

The driver creates a new type of board_type for gpib.conf

example gpib.conf:

https://github.com/elektronomikon/raspi_gpib_driver/blob/master/gpib.conf

(the entry "board_type" must match the name of the compiled kernel module)


To get quick access, try:
`gpib_config` to initialize linux-gpib and create /dev/gpib0
`ibterm -d<primary_addr>`

have fun,
Thomas
