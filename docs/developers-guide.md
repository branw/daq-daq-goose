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

## Contents

- What's a Microcontroller and Why Should I Care?
  - Pointers and Memory
- What++ the(syntax)?
  - Keywords
  - Bits, Bytes, and Binary
    - Floating-Point Data Types
  - Not Your Grandmother's Uno
- Interrupts and When It's Okay to Be Rude
- Peripherals Galore

## What's a Microcontroller and Why Should I Care?

!!!

### Pointers and Memory

Left as an exercise to the reader.

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

Arduino provides a nice wrapper around these two functions for us, carrying out
necessary functionality like initializing the hardware and calling these
methods at the appropriate time. This sounds helpful and usually is, until you
ever want to fine tune any of the details. [Here's what is actually being
compiled for us (note that this is SAM family specific)][arduino-main]:

[arduino-main]: https://github.com/arduino/ArduinoCore-sam/blob/master/cores/arduino/main.cpp

```
int main( void )
{
  // Initialize watchdog
  watchdogSetup();

  init();

  initVariant();

  delay(1);

#if defined(USBCON)
  USBDevice.attach();
#endif

  setup();

  for (;;)
  {
    loop();
    if (serialEventRun) serialEventRun();
  }

  return 0;
}
```

A quick walkthrough:
- `watchdogSetup` disables the watchdog timer, a very common module that acts
  as a deadman's switch for the microcontroller. If the watchdog timer is kept
  enabled, then your program must continually keep resetting it, for should it
  go a certain interval without being reset, it will restart your program. This
  is a good safety measure for critical applications, but is only bothersome
  for most applications.
- `init` will initialize everything that is in common for all devices in the
  SAM family, such as the output pins that will be used and the USB/serial
  connections.
- `initVariant` actually does nothing in this case, but typically would do any
  device-specific stuff, i.e. initializing things only present on our SAM3X.
- `setup` and `loop` are the very functions that we have provided.
- `if (serialEventRun) serialEventRun()` might be a strange looking line, and
  it is. The if-statement checks if the method `serialEventRun` is defined by
  the SAM variant, and if it is, calls it. The SAM variant defines this method
  as:

  ```
  void serialEventRun(void)
  {
    if (Serial.available()) serialEvent();
    if (Serial1.available()) serialEvent1();
    if (Serial2.available()) serialEvent2();
    if (Serial3.available()) serialEvent3();
  }
  ```

  So, for instance, [in our user code, we could have make a function `void
  serialEvent()` and it would be called every time there is data available in
  the serial buffer][serial-event].

[serial-event]: https://www.arduino.cc/en/Reference/SerialEvent

Looking at the Arduino source can prove useful when you begin to break out of
the training harnesses of the standard library, especially considering that
very little of Arduino's internals are documented. [If we look at the Due's
`variant.cpp` file, the place where all of the board-specific stuff like
`init()` is stored ][arduino-variant], we can learn that USARTS 0, 1, and 3 are
connected to the serial library, and that the ADC module is initialized.

[arduino-variant]: https://github.com/arduino/ArduinoCore-sam/blob/f5cbedd4e845b0fbb8165552f6ebb94363655234/variants/arduino_due_x/variant.cpp 

### Keywords

Hopefully you are familiar enough with `for` and `void`, but what about
`volatile` or `auto`?

- `volatile` lets the compiler know that a variable might get accessed in an
  unexpected way, so it shouldn't perform any crazy optimizations on it. This
  has nothing to do with the data type of the variable at all, but is very
  necessary when a variable is used in an interrupt handler: more on that
  later.
- `auto` is one of the quality-of-life features introduced in modern C++. It's
  a godsend when programming with complex types, but only muddies the waters in
  low-level development. Avoid the temptations.
- `void *` might be a bit perplexing. `void` can be used as a function return
  type to mean that a function doesn't return anything, so how exactly can a
  variable be "nothing"? 
- `register` is a legacy keyword and should be avoided. According to ancient
  history, it used to serve as a hint to the compiler to store the variable in
  a register. Nowadays, the compiler is smarter than us and knows when its best
  to do that.

### Bits, Bytes, and Binary

When writing high-level code in the days of 16GB RAM laptops and seemingly
endless Chrome tabs, it is easy to forget that underneath it all, variables,
data, and code are all just numbers to the CPU. We are passed the days of
worrying about code space limitations and other bottlenecks, but still need to
be mindful when targetting microcontrollers and other real-time devices. No
longer will you be able to use a native string type for representing text, nor
other abstractions taken from 30,000 ft above the silicon.

The SAM family is based on the ARM Cortex-M series of processors, one of the
most prevalent 32-bit microcontroller cores. 32-bit means that all of the data
paths inside of the CPU are 32-bit, so your variables can be trivially (but not
at most) 32-bits in size, and up to 2^32 or 4GB of memory can be addressed.
This leads to some interesting properties:

- Data processing is most likely the fastest when using 32-bit variables, as
  all of the machine instructions are designed to input and output 32-bit
  values. When you declare an 8-bit variable, it's probably being turned into a
  32-bit variable by the compiler anyway.
- Variables can be larger than 32-bit, in which case they'll be broken up into
  **words** of 32-bits each. A 64-bit variable, sometimes called a double word
  or dword, will merely comprise two 32-bit words. If you ever try to multiply
  two words, the result could potentially be larger than 32 bits, so the CPU
  instead produces two values, a high word and low word. When you join these
  two values together, you get 64 bits and the product.

This "word" (32-bit) is almost always expressed as the `int` data type in C or
C++. Likewise, the "double word" (64-bit) is the `long` data type. There's also
`short`, which is 16-bit, or a half-word, and `char`, which is a single byte.

Next, signedness. Every data type is available in `signed` and `unsigned`
flavors, a nice way of expressing how negative numbers are treated. Almost
every variable is `signed` by default, which means that it can represent negative
numbers as you would expect it to.

> _An aside_. `char` is an exception to the rule of default signedness. You
> should never rely on the default signedness of `char` and explicitly state it
> when using it for things other than storing text. Better yet, don't use
> `char` to store anything other than text. Thank the C specification writers
> for this strangely implementation-specific detail.

**It pays to be very explicit with variable declarations**. Not as in you're
getting paid by the number of characters you write, but as in you or the future
maintainer will thank you for reducing their mental overhead. Type keywords
like `int`, `char`, `long`, and `short` generally have common definitions
across modern desktops, but the same can't always be said for the
microcontroller world. Whereas an `int` on the Due is almost definitely 32-bit,
the word size of an Uno is only 16-bit and therefore their `int`s fit 65,000
times less values. You might remember this fact for now, but after switching to
and fro different target devices, the lines get easily blurred and can lead to
costly, hard-to-find mistakes.

Enter `cstdlib`. This C++ standard library, adapted from C's `stdlib.h`,
provides a couple of really handle types. Instead of worrying about how many
bytes are in an `int`, we can just write `uint32_t` and get an unsigned 32-bit
variable. **Use `(u?)int(8|16|32|64)_t` types whenever possible.**

| Type | Sign | Bytes | Range |
| ---- | ---- | ----- | ----- |
| `int8_t` | Signed | 1 | -128..127 |
| `uint8_t` | Unsigned | 1 | 0..255 |
| `int16_t` | Signed | 2 | -32,768..32,767 |
| `uint16_t` | Unsigned | 2 | 0..65,535 |
| `int32_t` | Signed | 4 | -2,147,483,648..2,147,483,647 |
| `uint32_t` | Unsigned | 4 | 0..4,294,967,295 |
| `int64_t` | Signed | 8 | -9,223,372,036,854,775,808..9,223,372,036,854,775,807 |
| `uint64_t` | Unsigned | 8 | 0..18,446,744,073,709,551,615 |

When declaring a variable, think about exactly what you will be storing and
choose an appropriate variable. Be conscious of the sign as well: it is almost
never a good idea to choose a `uint` over an `int` type just because of the
extra bit. On the other hand, it's okay to choose `int32_t` when you want to
count something, especially a loop index, and be pretty sure it won't reach 2
billion. `uint32_t` will be common in low-level code as well, but for a
different reason: we're more interested in the individual bits of a value than
it as a whole. And as always, use `char` for storing text or characters.

#### Floating-Point Data Types

All of that talk was about integers, and while they are handy for computers to
manipulate, we typically think in terms of analog, real numbers. Floating-point
numbers, universally defined by the IEEE 754 specification, provide a practical
implementation of fractional values. They allow for 

| Type | Precision | Bytes | Range |
| ---- | --------- | ----- | ----- |
| `float` | Single | 4 | -3.4E38 .. 3.4E38 |
| `double` | Double | 8 | -1.7E308 .. 1.7E308 |

Floating-points also have some strange gotchas that must be accounted for:

- We only have a finite number of bits, but there are an infinite number of
  values in between 0.000000000001 and 0.000000000002. Inevitably, a line has
  to be drawn some where and only so many different values can be expressed.
  [This leads to some weirdness][float-guide] that every programmer comes
  across one day: `0.1 + 0.2 != 0.3`. Seriously, go ahead and try it out.
  `0.1 + 0.2` actually evaluates to `0.30000000000000004`. Well, thanks
  computer, but that's obviously not correct!

  Because floating-points are an engineering approximation, we can use another
  engineering approximation to account for these errors. If we take the
  absolute value of the difference of both sides, comparing it against an
  "epsilon" value will practically indicate equality. Epsilon is a an
  application-specific value, but really just needs to be small enough to

  If you actually need truly precise data, first ask yourself if you really do,
  and then [consult wiser advice][float-advice].

!!!

[float-guide]: http://floating-point-gui.de/
[float-advice]: https://stackoverflow.com/a/2729750

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
   Mega's form factor for compatibility purposes. In doing this however, the
   pin numbers from the SAM3X datasheet are almost useless to us. Fret not,
   Arduino provides a few utilities to perform the conversion for us.

   !!!

3. The Due uses a **3.3V logic level**. In other words, you do not want to plug
   a 5V data pin into any of the I/O ports on the board. This has major
   implications for Mega shields: though the form factor may be compatible, the
   Mega uses a 5V logic level and therefore you must also check that the shield
   is Due-compatible. Also, if an external component only produces or consumes
   5V signals, you will need to pick up a [logic-level converter][llc].

   So, when you `digitalWrite(13, HIGH)`, pin 13 will output about 3.3V. If you
   were to connect pin 13 to pin 14, then `analogRead(14)` would return about
   `4095` (we have a 12-bit analog-digital-converter, so the maximum value is
   2^12-1=4095).

[llc]: https://www.sparkfun.com/products/12009

## Interrupts and When It's Okay to Be Rude

!!!

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
