#define VERSION_MAJOR 0
#define VERSION_MINOR 1

// 1.6 MHz is the max frequency of the DAC, so it's easiest
// just to scale everything off of it
const int freq = 1.6e6;

// Duration of a single chirp
const double chirp_duration = 2E-3;

// Number of samples per buffer provided to DAC
const int dac_block_size = 100;
// Total number of samples per waveform
// Must be multiple of dac_block_size
const int num_dac_samples = freq * chirp_duration;
// Number of separate buffers per waveform 
const int num_dac_blocks = num_dac_samples / dac_block_size;

// Internal index of the current block within the DAC buffer
volatile uint16_t dac_block_index = 0;
// The data buffer for the DAC to produce. Limited to 12-bits
uint16_t output_waveform[num_dac_samples];

// The ADC channels to sample. Note that the port numbers on the
// Due (A0-7) count in reverse order to the SAM3X's channels (CH7-0),
// i.e. A0 is CH7 and A1 is CH6
const int adc_channels = ADC_CHER_CH7 | ADC_CHER_CH6;
// Number of channels listed above
const int num_adc_channels = 2;
// Size of each block in the ADC buffer
const int adc_block_size = 200 * num_adc_channels;
// Number of samples to collect in total, for all channels
const int num_adc_samples = 20000;
// Number of blocks in the ADC buffer
const int num_adc_blocks = num_adc_samples / adc_block_size;

// Internal index of the current block within the DAC buffer
volatile uint16_t adc_block_index = 0;
// Buffers to store data that is converted from the ADC. Sampling for
// a longer time requires either flushing these buffers out perioidcally,
// or just increasing their size. Currently this does the latter which is
// not very scalable and should be resolved later on
volatile uint16_t input_waveforms[num_adc_blocks][adc_block_size];

// Flag to indicate that the data collection is finished and the buffers
// can be dumped
volatile bool data_ready = false;

// Generate the waveform for an enveloped, linear cosine sweep
void generate_chirp()
{ 
  // Duration of chirp
  const double t1 = chirp_duration;

  // Phase shift (rads)
  const double phi = 0;

  // Initial frequency (Hz)
  const double f0 = 5E3;
  // Final frequency (Hz)
  const double f1 = 105E3;

  // "Chirpyness" or rate of frequency change
  const double k = (f1 - f0) / t1;
  
  for (int i = 0; i < num_dac_samples; ++i)
  {
    double t = t1 * ((double)i / num_dac_samples);
    // Create a chirp from the frequency f(t) = f0 + kt
    double chirp = cos(phi + 2*PI * (f0*t + k/2 * t*t));
    // Create a Hanning window to envelope the chirp
    double window = 0.5 * (1 - cos(2*PI * i/(num_dac_samples - 1)));
    // Move the signal across a difference reference
    output_waveform[i] = 4095/2 + 4095/2 * (chirp * window);
  }
}

// Initialize the timer peripheral TC0
void setup_timer() {
  // Enable the clock of the peripheral
  pmc_enable_periph_clk(TC_INTERFACE_ID);

  TC_Configure(TC0, 0,
    // Waveform mode
    TC_CMR_WAVE |
    // Count-up with RC as the threshold
    TC_CMR_WAVSEL_UP_RC |
    // Clear on RA and set on RC
    TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET |
    TC_CMR_ASWTRG_CLEAR |
    // Prescale by 2 (MCK/2=42MHz)
    TC_CMR_TCCLKS_TIMER_CLOCK1);

  uint32_t rc = SystemCoreClock / (2 * freq);
  // Achieve a duty cycle of 50% by clearing after half a period
  TC_SetRA(TC0, 0, rc / 2);
  // Set the period
  TC_SetRC(TC0, 0, rc);
  TC_Start(TC0, 0);

  // Enable the interrupts with the controller
  NVIC_EnableIRQ(TC0_IRQn);
}

// Initialize the ADC peripheral
void setup_adc() {
  // Reset the controller
  ADC->ADC_CR = ADC_CR_SWRST;
  // Reset all of the configuration options
  ADC->ADC_MR = 0;
  // Stop any transfers
  ADC->ADC_PTCR = (ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS);

  // Setup timings
  ADC->ADC_MR |= ADC_MR_PRESCAL(1);
  ADC->ADC_MR |= ADC_MR_STARTUP_SUT24;
  ADC->ADC_MR |= ADC_MR_TRACKTIM(1);
  ADC->ADC_MR |= ADC_MR_SETTLING_AST3;
  ADC->ADC_MR |= ADC_MR_TRANSFER(1);
  // Use a hardware trigger
  ADC->ADC_MR |= ADC_MR_TRGEN_EN;
  // Trigger on timer 0, channel 0 (TC0)
  ADC->ADC_MR |= ADC_MR_TRGSEL_ADC_TRIG1;
  // Enable the necessary channels
  ADC->ADC_CHER = adc_channels;

  // Load the DMA buffer
  ADC->ADC_RPR  = (uint32_t)input_waveforms[0];
  ADC->ADC_RCR  = adc_block_size;
  if (num_adc_blocks > 1) {
    ADC->ADC_RNPR = (uint32_t)input_waveforms[1];
    ADC->ADC_RNCR = adc_block_size;
    adc_block_index++;
  }

  // Enable an interrupt when the end of the DMA buffer is reached
  ADC->ADC_IDR = ~ADC_IDR_ENDRX;
  ADC->ADC_IER = ADC_IER_ENDRX;
  NVIC_EnableIRQ(ADC_IRQn);
  
  // Enable receiving data
  ADC->ADC_PTCR = ADC_PTCR_RXTEN;
  // Wait for a trigger
  ADC->ADC_CR |= ADC_CR_START;
}

// A junk variable we point the DMA buffer to before stopping the ADC.
// This is to ensure that we don't overwrite anything, but might not
// be entirely necessary
uint16_t adc_junk_space[1] = {0};

// Interrupt handler for the ADC peripheral
void ADC_Handler() {
  if (ADC->ADC_ISR & ADC_ISR_ENDRX)  {
    if (adc_block_index >= num_adc_blocks) {
      adc_stop(ADC);
      TC_Stop(TC0, 0);
      
      data_ready = true;
      
      ADC->ADC_RPR = ADC->ADC_RNPR = (uint32_t)adc_junk_space;
      ADC->ADC_RCR = ADC->ADC_RNCR = 1;
      return;
    }
    ADC->ADC_RNPR  = (uint32_t)input_waveforms[++adc_block_index % num_adc_blocks];
    ADC->ADC_RNCR  = adc_block_size;
  }
}

// Initialize the DAC peripheral DACC0
void setup_dac() {
  // Enable the clock of the peripheral
  pmc_enable_periph_clk(DACC_INTERFACE_ID);
  
  dacc_reset(DACC);
  dacc_set_transfer_mode(DACC, 0);
  dacc_set_power_save(DACC, 0, 1);
  
  dacc_set_analog_control(DACC, DACC_ACR_IBCTLCH0(0x02) | DACC_ACR_IBCTLCH1(0x02) | DACC_ACR_IBCTLDACCORE(0x01));
  dacc_set_trigger(DACC, 1);

  dacc_set_channel_selection(DACC, 0);
  dacc_enable_channel(DACC, 0);

  NVIC_DisableIRQ(DACC_IRQn);
  NVIC_ClearPendingIRQ(DACC_IRQn);
  NVIC_EnableIRQ(DACC_IRQn);

  // Enable PDC TX requests
  DACC->DACC_PTCR = DACC_PTCR_TXTEN;

  dacc_enable_interrupt(DACC, DACC_IER_ENDTX);
  
  DACC->DACC_TPR = (uint32_t)(output_waveform);
  DACC->DACC_TCR = dac_block_size;
  if (num_dac_blocks > 1) {
    DACC->DACC_TNPR = (uint32_t)(output_waveform + dac_block_size);
    DACC->DACC_TNCR = dac_block_size;
  }
}

// A constant buffer for DMA to force the DAC to output its lowest value
// after stopping it
uint16_t dac_zero[1] = {0};

// Interrupt handler for the DAC peripheral
void DACC_Handler() {
  if (DACC->DACC_ISR & DACC_ISR_ENDTX) {
    if (dac_block_index >= num_dac_blocks) {
      dacc_disable_interrupt(DACC, DACC_IER_ENDTX);
      DACC->DACC_TPR = DACC->DACC_TNPR = (uint32_t)dac_zero;
      DACC->DACC_TCR = DACC->DACC_TNCR = 1;
      return;
    }
    DACC->DACC_TNPR = (uint32_t)(output_waveform + dac_block_size * (++dac_block_index % num_dac_blocks));
    DACC->DACC_TNCR = dac_block_size;
  }
}

// Reset all of the peripherals
void reset() {
  adc_block_index = dac_block_index = 0;

  // Temporarily disable write-protection for the power controller
  // while we enable peripheral clocks
  pmc_set_writeprotect(false);
  setup_dac();
  setup_adc();
  setup_timer();
  pmc_set_writeprotect(true);
}

void setup() {
  // USB serial is performed at native speed, negotiated by the host.
  // The baud rate set here will be ignored
  Serial.begin(1337);

  // Enable output on B ports
  REG_PIOB_OWER = 0xFFFFFFFF;
  REG_PIOB_OER =  0xFFFFFFFF;

  // Pre-generate a chirp signal
  generate_chirp();
}

void loop() {
  static bool collect_data = false;
  
  // Don't poll while we're collecting data (although this is unlikely to be
  // hit considering all of the cycles are consumed by the timer)
  if (collect_data && !data_ready) {
    return;
  }

  // Transmit the collected data when it's ready
  if (data_ready) {
    int body_len = num_adc_samples * 2;
    char header[5] = { 
      0x82,
      (body_len >> 0*8) & 0xff,
      (body_len >> 1*8) & 0xff,
      (body_len >> 2*8) & 0xff,
      (body_len >> 3*8) & 0xff,
    };
    SerialUSB.write(header, 5);

    // Assuming that we have two channels of data, it was interleaved.
    // To simplify processing, we return the two channels separately
    char data_point[2];
    for (int n = 0; n < num_adc_blocks; n += 1) {
      for (int m = 0; m < adc_block_size; m += 2) {
        data_point[0] = (input_waveforms[n][m] >> 0*8) & 0xff;
        data_point[1] = (input_waveforms[n][m] >> 1*8) & 0xff;
        SerialUSB.write(data_point, 2);
      }
    }
    for (int n = 0; n < num_adc_blocks; n += 1) {
      for (int m = 1; m < adc_block_size; m += 2) {
        data_point[0] = (input_waveforms[n][m] >> 0*8) & 0xff;
        data_point[1] = (input_waveforms[n][m] >> 1*8) & 0xff;
        SerialUSB.write(data_point, 2);
      }
    }
    
    data_ready = false;
    collect_data = false;
  }

  // Poll for incoming request packets
  // A packet header consists of an opcode (1 byte) and body length (4 bytes)
  if (SerialUSB.available() >= 5) {
    uint8_t opcode = SerialUSB.read();
    // We enforce little-endian for all communications.
    // Do not combine these lines: the lack of a sequence point
    // would cause undefined behavior as the calls to read() have
    // side effects
    uint32_t input_len = SerialUSB.read();
    input_len = input_len | (SerialUSB.read() << 8);
    input_len = input_len | (SerialUSB.read() << 16);
    input_len = input_len | (SerialUSB.read() << 24);

    // Dispatch the packet to its handler
    switch (opcode) {
      // Hello
      case 0x00: {
        char response[7] = {
          opcode | 0x80,
          2, 0, 0, 0,
          VERSION_MAJOR, VERSION_MINOR
        };
        SerialUSB.write(response, 7);
        break;
      }

      /*
      // Queue data
      case 0x01: {
        //TODO error handling

        // Queue the data to the DAC
        for (int i = 0; i < input_len; i++) {
          uint16_t data_point = SerialUSB.read();
          data_point = data_point | (SerialUSB.read() << 8);
          output_waveform[i] = data_point;
        }

        // Acknowledge the data
        char response[5] = { opcode | 0x80, 0, 0, 0, 0 };
        SerialUSB.write(response, 5);
        break;
      }
      */

      //TODO channel configuration packets

      // Collect data
      case 0x02: {
        char response[5] = { opcode | 0x80, 0, 0, 0, 0 };
        SerialUSB.write(response, 5);
        
        collect_data = true;

        reset();
        break;
      }

      // Unknown
      default: {
        char response[255] = {0};
        response[0] = 0xff;
        int msg_len = snprintf(&response[5], 250, "Unknown opcode %i", opcode);
        response[1] = (uint8_t)(msg_len);
        response[2] = (uint8_t)(msg_len >> 8);
        response[3] = (uint8_t)(msg_len >> 16);
        response[4] = (uint8_t)(msg_len >> 24);
        SerialUSB.write(response, msg_len + 5);
        break;
      }
    }
  }
}

