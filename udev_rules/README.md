# udev_rules

This directory contains udev rules to assign attached hardware to a specific
device driver.

## Creating a `udev` Rules File

1) Attach your device (i.e. Arduino Mega) to your computer.

2) Run `lsusb`.  Look for an entry with your device:
```
Bus 003 Device 002: ID 2341:0042 Arduino SA Mega 2560 R3 (CDC ACM)
```

We need the vendor and product IDs from this entry.  For the Arduino Mega the
vendor ID is `2341` and the product ID is `0042`.

3) Add a `udev` rule in `/etc/udev/rules.d`:

```
sudo vim /etc/udev/rules.d/99-arduino.rules
```

Within this file add the following:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="<Vendor ID>", ATTRS{idProduct}=="<Product
ID>", MODE="0666", SYMLINK+="<symbolic name for your device file>"
```

This entry will create a symbolic link for your device in `/dev/<symbolic
name>`.  The link will grant read/write permissions to your device (`0666` is
the file permissions `rw-rw-rw-`).

4) Apply your `udev` rules with:
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```

The `udev` rules are applied in numerical order with the number in front of the
filename.  Higher numbers will be applied last, meaning they take precedence.

## Included `udev` Rules

The following is a list of `udev` rules for the Robomagellan robot:
1) `99-arduino.rules` - Associates an Arduino Mega to the device file
`/dev/arduino`.
