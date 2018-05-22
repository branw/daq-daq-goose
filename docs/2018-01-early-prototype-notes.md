# Daq Daq Goose Early Prototype (Spring 2018)

## On-Chip Peripherals

- The SAM3X includes a bunch of on-chip peripherals that are worth looking into
  before exploring external options

## AD7476 External 1MSPS Serial ADC

- Controlled and interfaced entirely over SPI ("Serial")
  - Max f_SCLK is 20MHz 
  - Conversions are triggered by activating CS
    - Most SPI use cases don't account on toggling CS 1 million times a
        second, so we'll have to be crafty at working around this

- The +- 0.5 LSB linearity is after at least one dummy cycle: it is probably a
  good idea to spend a few cycles warming everything up anyway

### Interfacing the AD7476

- We want to poll at least two (and up to four) ADCs simultaneously
  - The on-board ADC implementation involved sequencing multiple channels
      right after one-another; likewise, it will probably not be possible to
      exactly sync the external devices up
- There are up to two SPI modules present
  - Each module controls up to 4 CS pins for multi-device control, not
      exactly useful for us though
  - MCK/4=21MHz slightly exceeds our spec but _might_ still work, worst case
      MCK/5=16.8MHz gives us a maximum sample rate of 16.8MHz/20=840kHz
  - But SPI1 (the second module) is only available on the 217-pin package
      SAM3X (not our Due)!
- USART modules are another option
  - There are four independent modules
  - SPI mode can achieve only up to MCK/6=14MHz SPCK, which gives a sample rate
    of about 700kHz
