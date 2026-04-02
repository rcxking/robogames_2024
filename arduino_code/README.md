# arduino_code

This directory contains the code that the Arduino runs for the Robomagellan
competition.

## Electrical/Mechanical Arduino Connections

The Arduino is responsible for writing motor commands to the Talon SRX motor
controllers and reading/computing the wheel velocities from the encoders.

This robot is using 2 [Talon SRX motor controllers](https://store.ctr-electronics.com/products/talon-srx).  Each motor has its own [CimCoder V2 encoder](https://files.andymark.com/PDFs/CIMcoderV2++Spec-Sheet+Rev+01.pdf).

To avoid encoder inaccuracies due to gear slippage the CimCoders are on their
respective motor shaft.  The motor shaft also has an 11-tooth gear connection to
a 72-tooth gear on the drive shaft with the wheel  (TODO: Add geartrain picture here).

## Encoder Ticks-to-Radians Conversion

The Arduino code communicates with the hardware interface defined in the
`robomagellan_firmware` package.  The interface's command interfaces will send
desired wheel velocities in rads/s to the Arduino; the state interfaces expect
the Arduino to send back each wheel's angular velocity in rad/s.

The CimCoders give 20 ticks per shaft revolution.  Therefore, we have the
following conversion:

```
 20 ticks    2*pi radians             pi
---------- = ------------ -> 1 tick = -- radians
revolution    revolution              10
```

We also know that the gear train is 11 to 72, so for each wheel we have:

```
         pi           11   11*pi
1 tick = -- radians * -- = ----- radians
         10           72    720
```

To compute each wheel's angular velocity for a given time delta (secs) count the
number of ticks, multiply the tick count by this tick-to-radians conversion
factor, and divide by the time delta.

## Software Dependencies

We need the [Magnetic Quadrature Encoder Library](https://www.pjrc.com/teensy/td_libs_Encoder.html).  This can be installed within the Arduino IDE's library manager and
searching for the `Encoder by Paul Stoffregen` library version 1.4.4.

## Serial Communications

The Arduino communicates using a serial connection with 115200 baud, 8N1.  Every
100 ms the wheel velocities are published as a string of the form:
```
L<left wheel velocity rad/s, 2 decimal places>;R<right wheel velocity rad/s, 2
decimal places>;
```
