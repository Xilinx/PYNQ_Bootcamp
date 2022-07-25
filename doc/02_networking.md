<!-- vim: tw=80:cc=80:spell:nowrap
-->
# Networking Configuration

Netowrking the PYNQ Board can be tricky, and the official documentation can add
to that confusion, but this document should clear the confusion and make a
network configuration easy and painless.

There are **2** methods that you can use to obtain an internet connection:

- DHCP, via connection to router
- Static IP, via connection to computer (assumed ethernet)

DHCP is the default method which the board obtains a connection, and should be
the choice picked.

## Wired Connection

### DHCP from Router

- (1) Connect an Ethernet cable from your board to either
	- Your network router
	- A network switch connected to a router
- (2) Turn on the PYNQ board and wit for it to boot.
- (3) Once booted, on your computer, go to your router's admin page (usually
  accessed by typing `192.168.1.1` into your browser's address bar.)
- (4) Depending on your router model, there should be a 'Attached Devices' or
  similarly named menu item to list the IP addresses of connected devices.
  - Scan for a hostname of 'PYNQ' or similar.
- (5) Once you have obtained the IP address, type in your browser
  `http://<PYNQ-ip-address>` to connect to the notebook web interface.

### Static IP from Computer

With a direct Ethernet connection from your computer to the PYNQ, you will be
able to use PYNQ and interact with notebooks. However, if you require internet
access, you must bridge your internet connection. To do this:

- (1) [Assign a static IP to the host machine](https://pynq.readthedocs.io/en/latest/appendix/assign_a_static_ip.html#assign-a-static-ip-address)
- (2) Connect the PYNQ to your computer via Ethernet
- (3) Open `http://192.168.2.99` in your web browser.

## Wireless connection

Once you are connected to the PYNQ via a wired method, you can optionally
configure a wireless connection. Pre-flashed images for bootcamp purposes my
come with wifi configuration already set.

You want to make sure that you have a USB wireless networking card connected to
your PYNQ board.

First connect to a shell session, wether through serial, ssh, or the web
interface, and type the following command as the `root` user

```bash
nano /boot/boot.py
```

In this file, you can append the following to the bottom.

```python
ssid = "<Your-network-name>"
passwd = "<Network-password>"

try:
    Wifi().connect(ssid, passwd, auto=True)
except:
    print("ERROR: Unable to connect to wireless network.")
```

This will automatically connect to wifi on boot. At this point you can reboot
your PYNQ board with the following command as `root`:

```bash
reboot
```

Next: [Contributing Guide](./03_contributing.md)
