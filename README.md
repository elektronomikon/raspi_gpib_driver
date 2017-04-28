# raspi_gpib_driver
A driver for linux-gpib to connect the Raspberry Pi to any GPIB device via its GPIO interface.

Intended to be used in conjunktion with the raspi_gpib_shield (https://github.com/elektronomikon/raspi_gpib_shield)


INSTALLATION

clone or download the patch in a new folder

download the current stable linux-gpib release (4.0.3 at this time)

extract linux-gpib with folder names in the new folder:

`  tar xzf linux-gpib-4.0.3.tar.gz`

configure linux-gpib:

`  cd linux-gpib-4.0.3`

`  ./configure`

this creates the needed build files

change back to the other folder and apply the patch with:

`  cd ..`

`  patch -p0 < RasPi_GPIB_driver.patch`

build linux-gpib

`cd linux-gpib-4.0.3`

`  make`

`  sudo make install`


USAGE

The driver creates a new type of board_type for gpib.conf

example gpib.conf:

https://github.com/elektronomikon/raspi_gpib_driver/blob/master/gpib.conf

To get quick access, try:
`ibterm -d0`


have fun,
Thomas
