// Sample frequency
#define FS 1.33e6

void setup_timer() {
  pmc_enable_periph_clk(TC_INTERFACE_ID);
  
  TC_Configure(TC0, 0, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET | TC_CMR_ASWTRG_CLEAR | TC_CMR_TCCLKS_TIMER_CLOCK1);
  
  uint32_t rc = SystemCoreClock / (2 * FS);
  TC_SetRA(TC0, 0, rc / 2);
  TC_SetRC(TC0, 0, rc);
  TC_Start(TC0, 0);
  
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  TC0->TC_CHANNEL[0].TC_IDR = ~TC_IER_CPCS;
  
  NVIC_EnableIRQ(TC0_IRQn);
}

#define NUM_CHANNELS 1
#define ADC_CHANNELS ADC_CHER_CH7 //| ADC_CHER_CH6 | ADC_CHER_CH5
#define BUFFER_SIZE 200*NUM_CHANNELS
#define NUMBER_OF_BUFFERS 5 /// Make this 3 or greater

volatile bool dataReady;
uint16_t adcBuffer[NUMBER_OF_BUFFERS][BUFFER_SIZE];
unsigned int adcDMAIndex;        //!< This hold the index of the next DMA buffer

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
  ADC->ADC_CHER = ADC_CHANNELS;

  ADC->ADC_RPR  = (unsigned long)adcBuffer[0];
  ADC->ADC_RCR  = BUFFER_SIZE;
  
  adcDMAIndex = (adcDMAIndex + 1) % NUMBER_OF_BUFFERS;
  
  ADC->ADC_IDR   = ~ADC_IDR_ENDRX;
  ADC->ADC_IER = ADC_IER_ENDRX;
  
  NVIC_EnableIRQ(ADC_IRQn);
  ADC->ADC_PTCR = ADC_PTCR_RXTEN;  // Enable receiving data.
  ADC->ADC_CR |= ADC_CR_START; //start waiting for trigger.
}

// Duration of a single chirp
const double chirp_duration = 2E-3;

// Frequency to trigger conversions at (bound at ~1.666E6)
const int sample_freq = 2E6;

// Number of samples per buffer provided to DAC
// Should not exceed 100
const int samples_per_block = 100;
// Total number of samples per waveform
// Must be multiple of samples_per_block
const int num_samples = 3200;
// Number of separate buffers per waveform 
const int num_blocks = num_samples/samples_per_block;

uint16_t waveform[num_samples];

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
  
  for (int i = 0; i < num_samples; ++i)
  {
    double t = t1 * ((double)i / num_samples);
    // Create a chirp from the frequency f(t) = f0 + kt
    double chirp = cos(phi + 2*PI * (f0*t + k/2 * t*t));
    // Create a Hanning window to envelope the chirp
    double window = 0.5 * (1 - cos(2*PI * i/(num_samples - 1)));
    // Move the signal across a difference reference
    waveform[i] = 4095/2 + 4095/2 * (chirp * window);
  }
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

  // Enable PDC TX requests
  DACC->DACC_PTCR = DACC_PTCR_TXTEN;

  dacc_enable_interrupt(DACC, DACC_IER_ENDTX);
  
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

  pmc_set_writeprotect(false);

  setup_dac();
  setup_adc();
  setup_timer();

  pmc_set_writeprotect(true);
}

void TC0_Handler() {
  REG_TC0_SR0;

  static int i = 0;

  /*
  if (i == 400) {
    adc_stop(ADC);
    TC_Stop(TC0, 0);

    dataReady = true;
  }
  */

  i++;

  REG_PIOB_SODR = 0x1 << 27; // 13
  REG_PIOB_CODR = 0x1 << 27; // 13
}

void ADC_Handler() {
  if (ADC->ADC_ISR & ADC_ISR_ENDRX)  {
    REG_PIOB_SODR = 0x1 << 26; // 22
    REG_PIOB_CODR = 0x1 << 26; // 22
    
    ADC->ADC_RNPR  = (unsigned long) adcBuffer[(adcDMAIndex + 1) % NUMBER_OF_BUFFERS];
    ADC->ADC_RNCR  = BUFFER_SIZE;

    adcDMAIndex = (adcDMAIndex + 1) % NUMBER_OF_BUFFERS;
  }
}

volatile uint16_t block_index = 0;
void DACC_Handler() {
  if (DACC->DACC_ISR & DACC_ISR_ENDTX) {
    DACC->DACC_TNPR =  (uint32_t)(waveform + samples_per_block * (++block_index % num_blocks));
    DACC->DACC_TNCR =  samples_per_block;
  }
}

void loop() { }
