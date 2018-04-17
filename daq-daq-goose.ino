const int sample_rate = 0.4e6;

// Duration of a single chirp
const double chirp_duration = 2E-3;

// Number of samples per buffer provided to PDC
// Should not exceed 100
const int samples_per_block = 100;
// Total number of samples per waveform
// Must be multiple of samples_per_block
const int num_samples = 1997;
// Number of separate buffers per waveform 
const int num_blocks = (int)((double)num_samples/samples_per_block+0.5);

volatile uint16_t block_index = 0;

const int num_output_channels = 1;

uint16_t output_waveform[num_samples];

const int num_input_channels = 2;
const int input_channel_mask = ADC_CHER_CH7 | ADC_CHER_CH6;

uint16_t input_waveform[num_samples*num_input_channels];

volatile bool dataReady = false;


// Generate the waveform for an enveloped, linear cosine sweep
void generate_chirp()
{ 
  // Phase shift (rads)
  const double phi = 0;
  // Initial frequency (Hz)
  const double f0 = 5E3;
  // Final frequency (Hz)
  const double f1 = 105E3;
  
  // Duration of chirp
  const double t1 = chirp_duration;
  // "Chirpyness" or rate of frequency change
  const double k = (f1 - f0) / t1;
  
  for (int i = 0; i < num_samples - 1; ++i)
  {
    double t = t1 * ((double)i / num_samples);
    // Create a chirp from the frequency f(t) = f0 + kt
    double chirp = cos(phi + 2*PI * (f0*t + k/2 * t*t));
    // Create a Hanning window to envelope the chirp
    double window = 0.5 * (1 - cos(2*PI * i/(num_samples - 1)));
    // Move the signal across a difference reference
    output_waveform[i] = 4095/2 + 4095/2 * (chirp * window);
  }
}

void setup_timer() {
  pmc_enable_periph_clk(TC_INTERFACE_ID);
  
  TC_Configure(TC0, 0, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET | TC_CMR_ASWTRG_CLEAR | TC_CMR_TCCLKS_TIMER_CLOCK1);
  
  uint32_t rc = SystemCoreClock / (2 * sample_rate);
  TC_SetRA(TC0, 0, rc / 2);
  TC_SetRC(TC0, 0, rc);
  TC_Start(TC0, 0);
  
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  TC0->TC_CHANNEL[0].TC_IDR = ~TC_IER_CPCS;

  NVIC_EnableIRQ(TC0_IRQn);
}

void setup_adc() {
  ADC->ADC_CR = ADC_CR_SWRST;         // Reset the controller.
  ADC->ADC_MR = 0;                    // Reset Mode Register.
  ADC->ADC_PTCR = (ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS); // Reset PDC transfer.
  
  ADC->ADC_MR |= ADC_MR_PRESCAL(1);
  ADC->ADC_MR |= ADC_MR_STARTUP_SUT24; //TODO increase startup time so first reading is good
  ADC->ADC_MR |= ADC_MR_TRACKTIM(1);
  ADC->ADC_MR |= ADC_MR_SETTLING_AST3;
  ADC->ADC_MR |= ADC_MR_TRANSFER(1);
  ADC->ADC_MR |= ADC_MR_TRGEN_EN;
  ADC->ADC_MR |= ADC_MR_TRGSEL_ADC_TRIG1;
  ADC->ADC_CHER = input_channel_mask;

  ADC->ADC_RPR  = (uint32_t)input_waveform;
  ADC->ADC_RCR  = samples_per_block * num_input_channels;
  if (num_blocks > 1) {
    ADC->ADC_RNPR  = (uint32_t)(input_waveform + num_input_channels * samples_per_block);
    ADC->ADC_RNCR  = samples_per_block * num_input_channels;
  }
  
  ADC->ADC_IDR   = ~(ADC_IDR_ENDRX);
  ADC->ADC_IER = ADC_IER_ENDRX;
  
  NVIC_EnableIRQ(ADC_IRQn);
  ADC->ADC_PTCR = ADC_PTCR_RXTEN;  // Enable receiving data.
  ADC->ADC_CR |= ADC_CR_START; //start waiting for trigger.
}

void setup_dac() {
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

  dacc_enable_interrupt(DACC, DACC_IER_ENDTX);

  // Enable PDC TX requests
  DACC->DACC_PTCR = DACC_PTCR_TXTEN;

  DACC->DACC_TPR = (uint32_t)(waveform);      // DMA buffer
  DACC->DACC_TCR = samples_per_block;
  if (num_blocks > 1) {
    DACC->DACC_TNPR = (uint32_t)(waveform + samples_per_block);
    DACC->DACC_TNCR = samples_per_block;
  }
}

void setup() {
  Serial.begin(115200);

  // Enable output on B ports
  REG_PIOB_OWER = 0xFFFFFFFF;
  REG_PIOB_OER =  0xFFFFFFFF;

  generate_chirp();

  // Enable peripherals
  pmc_set_writeprotect(false);
  setup_dac();
  setup_adc();
  setup_timer();
  pmc_set_writeprotect(true);

  for (;;) {
    if (dataReady) {
      for (int n = 0; n < NUMBER_OF_BUFFERS; ++n) {
        for (int m = 0; m < BUFFER_SIZE; ++m) {
          Serial.print(n);
          Serial.print('\t');
          Serial.print(m);
          Serial.print('\t');
          Serial.println(adcBuffer[n][m]);
          Serial.flush();
        }
      }

      dataReady = false;
    }
  }
}

void TC0_Handler() {
  // Poll status register to acknowledge interrupt
  REG_TC0_SR0;

  static int i = 0;
  
  if (i == num_samples) {
    // Disable timer, DAC, and ADC
    TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKDIS;
    DACC->DACC_TNCR = 0;
    ADC->ADC_RNCR = 0;
    
    dataReady = true;
  }
  
  i++;
}

void ADC_Handler() {
  REG_PIOB_SODR = 0x1 << 27; // 13
  REG_PIOB_CODR = 0x1 << 27; // 13
  if (ADC->ADC_ISR & ADC_ISR_ENDRX)  {
    adcDMAIndex = (adcDMAIndex + 1) % NUMBER_OF_BUFFERS;
        
    ADC->ADC_RNPR  = (unsigned long) adcBuffer[adcDMAIndex];
    ADC->ADC_RNCR  = BUFFER_SIZE;
  }
}

void DACC_Handler() {
  REG_PIOB_SODR = 0x1 << 26; // 13
  REG_PIOB_CODR = 0x1 << 26; // 13
  if (DACC->DACC_ISR & DACC_ISR_ENDTX) {
    DACC->DACC_TNPR =  (uint32_t)(waveform + samples_per_block * (++block_index % num_blocks));
    DACC->DACC_TNCR =  samples_per_block;
  }
}

void loop() {}
