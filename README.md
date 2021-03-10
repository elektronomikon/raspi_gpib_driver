NEWS 2021-03-10:
Work is currently underway to merge this driver with work from Marcello Carla into the official linux-gpib tree.
(preview under for_linux-gpib-4.3.4)

# raspi_gpib_driver
A driver for linux-gpib to connect the Raspberry Pi to any GPIB device via its GPIO interface.

Intended to be used in conjunktion with the raspi_gpib_shield (https://github.com/elektronomikon/raspi_gpib_shield)
Preassembled devices can be found at https://elektronomikon.org

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
