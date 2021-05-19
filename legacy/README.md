# raspi_gpib_driver - OBSOLETE

UPDATE 2021-05:
The raspi_gpib_driver has been integrated into linux-gpib (https://linux-gpib.sourceforge.io/) with Version 4.3.4 as a general "gpib_bitbang" driver, please use the official source or package.
This repository will be preserved to maintain support for older linux-gpib Versions.

INSTALLATION

clone or download the patch in a new folder

download the current stable linux-gpib release (4.2.0 at this time)

extract linux-gpib with folder names in the new folder:

`  tar xzf linux-gpib-4.2.0.tar.gz`  
`  cd linux-gpib-4.2.0`

extract and configure linux-gpib-kernel:

`  tar xzf linux-gpib-kernel-4.2.0.tar.gz`  
`  cd linux-gpib-kernel-4.2.0`  
`  ./configure`

this creates the needed build files. Download the patch file and copy it to the linux-gpib-4.2.0 folder (the parent of linux-gpib-kernel-4.2.0). Apply it to add the raspi_gpib driver:

`  cd ..`  
`  patch -p0 < linux-gpib-kernel-4.2.0_raspi_gpib.patch`

build linux-gpib-kernel:

`  cd linux-gpib-kernel-4.2.0`  
`  make`  
`  sudo make install`  


USAGE

The driver creates a new type of board_type for gpib.conf

example gpib.conf:

https://github.com/elektronomikon/raspi_gpib_driver/blob/master/gpib.conf

(the entry "board_type" must match the name of the compiled kernel module)


To get quick access, try:
`gpib_config` to initialize linux-gpib and create /dev/gpib0
`ibterm -d<primary_addr> -s<secondary_addr>`

Thanks go to:
Marcello Carla for adding the interrupt driven handshake
Sebastian Weiss (dl3yc) for the timespec64 update

have fun,
Thomas
