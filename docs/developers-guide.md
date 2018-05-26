# Developer's Guide

So you want to write code for microcontrollers (MCUs)? If you have a background
in high-level programming (i.e. desktop or web applications -- managed
languaged from JavaScript to Java), prepare to forget everything you know. If
you don't have any experience at all, well buckle up for the wild ride. This
document describes a few common paradigms that one may encounter in low-level
development, including good practices to embrace and dark patterns to steer
away from. Specifically, this is written with the Arduino Due platform in mind,
however there are many general take-aways as well.

**TL;DR? Look at the bolded parts and pretend you understand the rest.**

TODO explain microcontrollers a bit and mention ARM

## What++ the(syntax)?

You may have heard that Arduinos are programmed in neither C or C++, but rather
just "the Arduino language" or some other sugar-coated terminology. Where we're
going though, there's no room for hand-holding. In fact, ask any sane embedded
developer and they'll probably tell you that this project far exceeds the
intended use-case of the Arduino platform. Nevertheless, Arduino is very useful
to us and provides a few features that would take several more documents just
like this to describe, even with the many quirks that we have to work around as
a result.

But really though, what language _are_ we writing? Short answer: **its C++**.
Long answer: its C++. The Arduino IDE compiles your code with `arm-g++`, the de
facto C++ compiler for the ARM microcontroller family. Don't get it wrong,
there is a lot of processing done to your code before it reaches the compiler,
but everything you write will be valid C++. This doesn't mean that you need to
use classes or template metaprogramming or [any of the cool features introduced
in modern C++][arduino-cpp11], but it does mean that the libraries you rely on
might. In practice, you'll come across both C and C++ styles of design and get
to decide when writing your own code. At the very least, **try to remain
consistent**.

[arduino-cpp11]: https://hackaday.com/2017/05/05/using-modern-c-techniques-with-arduino/

> _An aside_. While the Arduino platform tries its best to convince you
> otherwise, the Due is very different from its other Arduino brethren. Not
> different as in the Arduino Uno is Italian and the Due is half-French, half-
> Luxembourger; different as in the Uno is a bearded dragon but our Due is a
> highland cow. That is to say, the Due is an ARM device, while almost all of
> its predecessors have been AVR devices.
>
> `Serial.println` might look exactly the same in your code, but under the hood
> it is being compiled into a completely different instruction set. Even for
> our contrived setup, we won't be needing to write any assembly ourself, so
> this is more of a neat fact to tell your mom than anything else.

When writing any Due code, there are two fundemental functions we need to
provide, not because we're writing C++ but because it's Arduino.

```
void setup() {
    // ...
}

void loop() {
    // ...
}
```

Arduino provides a nice wrapper around these two functions for us, and will
carry out necessary functionality like initializing the hardware. This sounds
helpful and usually is, until you ever want to configure the fine details. 

### Not Your Grandmother's Uno

Although most of the differences are out of sight, there are a few things that
have to be done differently on the Due.

1. Instead of `Serial`, **use `SerialUSB`**. Notice that there are two USB
   ports on the Due? Hopefully by now you've figured out that the one closest
   to the power port is for programming, with the other one not seeming to do
   anything useful. Ironically, this programming port is not connected to the
   SAM3X, while the one closest to the reset button is. This port, known as the
   native serial port, allows for significantly higher throughput by the fact
   that it talks over the native USB 2.0 protocol, reaching up to 480 Mb/s.
   Existing code can be trivially migrated from `Serial` to `SerialUSB`, except
   only the baud rate passed to `SerialUSB.begin` is entirely ignored.

   ```
   void setup() {
       SerialUSB.begin(1337 /* put your favorite number here */);
       SerialUSB.println("Hello, world!");
   }

   void loop() {
       while (SerialUSB.available() > 0) {
           // Echo everything back
           SerialUSB.write(SerialUSB.read());
       }
   }
   ```

2. Given the number of pins on its package, the Due was given the Ardunio
   Mega's form factor for compatibility purposes. 

## Peripherals Galore

While it's not quite a system-on-a-chip (SoC), the SAM3X provides a lot of
periphery that we might want to take advantage of. Let's see:

- USB 2.0 host with 10 endpoints and DMA
- 4 configurable USARTS and 1 UART
- 9 32-bit timer/counter channels
- 8 16-bit PWM channels
- 32-bit real-time timer and real-time clock
- 16 12-bit 1 MSPS ADC channels
- 2 12-bit 1 MSPS DAC channels
- Ethernet 10/100 with DMA

Very few of these are setup out of the box by the Arduino platform, and even
less have a nice interface for controlling them. If you haven't already, now
would be a good time to make friends with the MCU datasheet.