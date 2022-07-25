<!-- vim: tw=80:cc=80:spell
-->
# PYNQ-Z2 Basic Setup

This is an setup guide focused on the PYNQ-Z2 Board running a stock PYNQ Image,
the complete installation instructions can be found
[here](https://pynq.readthedocs.io/en/latest/getting_started.html).

## Prequisites

- PYNQ-Z2 board
- Empty MicroSD Card (Minimum 8GB)
- Computer with Internet access and Browser
- Micro USB Cable
- Ethernet Cable

## Flashing The MicroSD Card

(If you have a pre-flashed MicroSD card, you can skip this step)

The PYNQ board runs a customized version of a Linux environment based on Ubuntu
Bionic 18.04. In order to utilize your PYNQ board, you must first flash a
bootable image in order to use it.

You can download a prebuilt image from [here](http://www.pynq.io/board.html)

If you were provided a custom image for the purposes of running a bootcamp, that
will also be fine as well.

### Windows

- (1) Download [Rufus](https://rufus.ie/en/), this will be out tool of choice to
  flash a custom image to the SD card.
- (2) Connect your SD card to your Computer
- (3) Open Rufus, under the "Device" dropdown, select your SD card.
- (4) Under the "Boot Selection" dropdown, choose the PYNQ .img file that you
  downloaded.
- (5) Leave "Partition Scheme", "Target system", "File system", and "Cluster
  size" untouched, the defaults are all we need.
- (6) Click the 'Start' button at the bottom right-hand corner of the window.
  This will start the flashing process

### macOS/Linux

- (1) Install [Belena Etcher](https://www.balena.io/etcher/)
- (2) Open etcher, and click the 'Flash from file button', and browse to the
  `.img` file that you downloaded with the PYNQ image.
- (3) Next select the SD card with the 'Select Target' button. (It should only
  display removable storage devices)
- (4) Click the 'Flash' button and wait until it finishes.

## Board Setup

![Board Diagram](https://pynq.readthedocs.io/en/latest/_images/pynqz2_setup.png)


- (1) Set the Boot jumper to the SD position. (This sets the board to boot from
  the Micro-SD card)
- (2) To power the board from the micro USB cable, set the Power jumper to the
  USB position. (You can also power the board from an external 12V power
  regulator by setting the jumper to REG.)
- (3) Insert the Micro SD card loaded with the PYNQ-Z2 image into the Micro SD
  card slot underneath the board
- (4) Connect the USB cable to your PC/Laptop, and to the PROG - UART MicroUSB
  port on the board. This step is only necessary if you did not connect to DC
  power or if you dont plan to communicate to the board via serial USB.
- (5) Connect the Ethernet port from the board to a router or your computer (see
  the networking section for more info).
- (6) Power on the board by sliding the power switch to the 'ON' position.

To verify if your board successfully booted, there should be two LEDs that turn
on.

## Networking

Networking the PYNQ can be a bit tricky, but these are the basic guidelines.

- The best way to setup your boards internet connection is to connect the
  Ethernet port to either a router, or a network switch connected to a router.
- Connecting via ethernet to a computer can establish a connection locally, but
  requires extra setup to get a working internet connection.

For more detailed information and guidance, check out the [Networking
Guide](./02_networking.md)

## Cloning the PYNQ_Bootcamp repository.

If you used a pre-flashed SD card in your board setup, there should be a file
`'Welcome to Pynq.ipynb'` that will contain a command to clone the repository to
your notebooks directory.

If you instead flashed an offical image to an SD card manually, this is how you
clone the repository for the bootcamp.

In the Jupyter web interface, click 'New' -> 'Terminal' and enter the following
commands in the shell:

```bash
# switch to a non-root user
su xilinx
# go the the notbooks directory
cd ~/jupyter_notebooks
# clone the repository
git clone https://github.com/Xilinx/PYNQ_Bootcamp
```

Next: [Networking Guide](./02_networking.md)
