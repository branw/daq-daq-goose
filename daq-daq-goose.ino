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

void setup() {
  Serial.begin(115200);
  
  REG_PIOB_OWER = 0xFFFFFFFF;
  REG_PIOB_OER =  0xFFFFFFFF;

  pmc_set_writeprotect(false);

  setup_adc();
  setup_timer();

  pmc_set_writeprotect(true);
}

void TC0_Handler() {
  REG_TC0_SR0;

  static int i = 0;

  if (i == 400) {
    adc_stop(ADC);
    TC_Stop(TC0, 0);

    dataReady = true;
  }

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

void loop() {
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
