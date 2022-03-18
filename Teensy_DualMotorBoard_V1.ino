typedef union {
  float fp;
  byte bin[4];
  unsigned int uint;
  int sint;
  bool bl;
} binaryFloat;





#define chopperpin 33 //Digital output for chopper resistor
#define debugpin 32
#define engate 34

//#define PRREG(x) Serial.print(#x" 0x"); Serial.println(x,HEX)
#include <Biquad.h>
#include <Math.h>
#include <ControlTools.h>
#include "muziek.c"
#include <SPI.h>
#include <QuadEncoder.h>

#include <MotionProfile2.h>
#include <arm_math.h>
#include "defines.h"

const float Busadc2Vbus = 1 / 4095.0 * 3.3 * ((68.3 + 5.05) / 5.05); //5.1 changed to 5.05 to improve accuracy. May differ board to board.

int n_senscalib;
bool setupready;

void setup() {
  Serial.begin(9600);

  pinMode( 10, OUTPUT);
  pinMode( engate , OUTPUT);
  digitalWrite( engate , 1);

  // Disable for DRV8302:
  SPI_init();
  xbar_init();
  adc_init();
  adc_etc_init();
  flexpwm2_init();
  flexpwm4_init();
  syncflexpwm();
  Encoders_init();

  //  pinMode(debugpin, OUTPUT);
  //  pinMode(chopperpin, OUTPUT);

  while (Serial.available() > 4) {
    processSerialIn();
  }

  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMA_EN( 7 ); // Activate all A channels
  FLEXPWM4_OUTEN |= FLEXPWM_OUTEN_PWMA_EN( 7 ); // Activate all A channels

  delay(100); //Allow the lowpass filters in current measurement to settle before calibration
  setupready = 1;
}

void SPI_init() {
  SPI.begin();
  SPI.beginTransaction(SPISettings( 10e6 , MSBFIRST, SPI_MODE1)); //DRV8301 specsheet: 100 ns -> 10 Mhz. Set correct mode (fallingedgeof the clock).

  int SSpin = 35;
  pinMode(SSpin, OUTPUT);

  int receivedVal16;

  while (receivedVal16 != 0x1038) {
    digitalWrite(SSpin, LOW);
    // receivedVal16 = SPI.transfer16( (0 << 15) | (0x02 << 11) | (1 << 3) ); //Set 3 PWM inputs mode
    receivedVal16 = SPI.transfer16( (0 << 15) | (0x02 << 11) | (1 << 5) | (1 << 4) | (1 << 3) ); //Set 3 PWM inputs mode, disable OC mode
    digitalWrite(SSpin, HIGH);
    digitalWrite(SSpin, LOW);
    receivedVal16 = SPI.transfer16( (1 << 15) | (0x02 << 11)  ); // Read register two
    digitalWrite(SSpin, HIGH);
    digitalWrite(SSpin, LOW);
    receivedVal16 = SPI.transfer16( (1 << 15) ); // Get the data
    digitalWrite(SSpin, HIGH);
  }
  SPI.endTransaction();
  //Serial.println( receivedVal16 , HEX); // This should give 0x1008
}

void xbar_init() {
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);   //turn clock on for xbara1
  xbar_connect(XBARA1_IN_FLEXPWM2_PWM1_OUT_TRIG0, XBARA1_OUT_ADC_ETC_TRIG00); //FlexPWM to adc_etc
}

void adc_init() {
  //Tried many configurations, but this seems to be best:
  ADC1_CFG = ADC_CFG_ADICLK(0)       // input clock select - IPG clock
             | ADC_CFG_MODE(2)      // 12-bit conversion 0 8-bit conversion 1 10-bit conversion 2  12-bit conversion
             | ADC_CFG_ADIV(2)      // Input clock / 4
             | ADC_CFG_ADSTS(0)     // Sample period (ADC clocks) = 3 if ADLSMP=0b
             | ADC_CFG_ADHSC        // High speed operation
             | ADC_CFG_ADTRG;       // Hardware trigger selected
  ADC2_CFG = ADC_CFG_ADICLK(0)       // input clock select - IPG clock
             | ADC_CFG_MODE(2)      // 12-bit conversion 0 8-bit conversion 1 10-bit conversion 2  12-bit conversion
             | ADC_CFG_ADIV(2)      // Input clock / 4
             | ADC_CFG_ADSTS(0)     // Sample period (ADC clocks) = 3 if ADLSMP=0b
             | ADC_CFG_ADHSC        // High speed operation
             | ADC_CFG_ADTRG;       // Hardware trigger selected

  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 &= ~ (1 << 12) ; // disable keeper pin 14, as per manual
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_03 &= ~ (1 << 12) ; // disable keeper pin 15, as per manual
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_07 &= ~ (1 << 12) ; // disable keeper pin 16, as per manual
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_06 &= ~ (1 << 12) ; // disable keeper pin 17, as per manual
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_01 &= ~ (1 << 12) ; // disable keeper pin 18, as per manual
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_00 &= ~ (1 << 12) ; // disable keeper pin 19, as per manual

  //Calibration of ADC
  ADC1_GC |= ADC_GC_CAL;   // begin cal ADC1
  while (ADC1_GC & ADC_GC_CAL) ;
  ADC2_GC |= ADC_GC_CAL;   // begin cal ADC2
  while (ADC2_GC & ADC_GC_CAL) ;

  ADC1_HC0 = 16;   // ADC_ETC channel
  ADC2_HC0 = 16;
}

void adc_etc_init() {
  /* Must disable software reset first */
  ADC_ETC_CTRL &= ~(1 << 31); // SOFTRST

  /* Must clear ADC_ETC_CTRL_TSC_BYPASS bit if using ADC2 */
  ADC_ETC_CTRL &= ~(1 << 30); // TSC_BYPASS

  /* Enable the external XBAR trigger0 and trigger1. Trigger4 uses sync mode to get triggered */
  ADC_ETC_CTRL = 1;  // TRIG_ENABLE

  /* ADC channel, pin number
    7,  // 14/A0  AD_B1_02
    8,  // 15/A1  AD_B1_03
    12, // 16/A2  AD_B1_07
    11, // 17/A3  AD_B1_06
    6,  // 18/A4  AD_B1_01
    5,  // 19/A5  AD_B1_00


    12, // 16/A2  AD_B1_07  M1 Ia
    8,  // 15/A1  AD_B1_03  M1 Ib

    6,  // 18/A4  AD_B1_01  M1 Va
    5,  // 19/A5  AD_B1_00  M1 Vb

    11, // 17/A3  AD_B1_06 Vbus

  */

  ADC_ETC_TRIG0_CTRL = ADC_ETC_TRIG_CTRL_TRIG_CHAIN(2); //TRIG chain length (0->1, 1->2, etc)

  ADC_ETC_TRIG0_CHAIN_1_0 =
    ADC_ETC_TRIG_CHAIN_IE1(0) |
    ADC_ETC_TRIG_CHAIN_B2B1 |
    ADC_ETC_TRIG_CHAIN_HWTS1(1) |
    ADC_ETC_TRIG_CHAIN_CSEL1(6) |
    ADC_ETC_TRIG_CHAIN_IE0(0) |
    ADC_ETC_TRIG_CHAIN_B2B0 |
    ADC_ETC_TRIG_CHAIN_HWTS0(1) |
    ADC_ETC_TRIG_CHAIN_CSEL0(12);

  ADC_ETC_TRIG0_CHAIN_3_2 =
    ADC_ETC_TRIG_CHAIN_IE0(1) |
    ADC_ETC_TRIG_CHAIN_B2B0 |
    ADC_ETC_TRIG_CHAIN_HWTS0(1) |
    ADC_ETC_TRIG_CHAIN_CSEL0(11);

  ADC_ETC_TRIG4_CTRL = ADC_ETC_TRIG_CTRL_TRIG_CHAIN(2) | ADC_ETC_TRIG_CTRL_SYNC_MODE; //TRIG chain length (0->1, 1->2, etc), turn on sync mode
  ADC_ETC_TRIG4_CHAIN_1_0 =
    ADC_ETC_TRIG_CHAIN_IE1(0) |
    ADC_ETC_TRIG_CHAIN_B2B1 |
    ADC_ETC_TRIG_CHAIN_HWTS1(1) |
    ADC_ETC_TRIG_CHAIN_CSEL1(5) |
    ADC_ETC_TRIG_CHAIN_IE0(0) |
    ADC_ETC_TRIG_CHAIN_B2B0 |
    ADC_ETC_TRIG_CHAIN_HWTS0(1) |
    ADC_ETC_TRIG_CHAIN_CSEL0(8);

  ADC_ETC_TRIG4_CHAIN_3_2 =
    ADC_ETC_TRIG_CHAIN_IE0(2) |
    ADC_ETC_TRIG_CHAIN_B2B0 |
    ADC_ETC_TRIG_CHAIN_HWTS0(1) |
    ADC_ETC_TRIG_CHAIN_CSEL0(5);

  attachInterruptVector(IRQ_ADC_ETC0, adcetc0_isr);
  NVIC_ENABLE_IRQ(IRQ_ADC_ETC0);
  attachInterruptVector(IRQ_ADC_ETC1, adcetc1_isr);
  NVIC_ENABLE_IRQ(IRQ_ADC_ETC1);
}

void flexpwm2_init() {     //set PWM
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK( 7 );//  Clear Load Okay LDOK(SM) -> no reload of PWM settings
  FLEXPWM2_SM0CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_HALF | FLEXPWM_SMCTRL_PRSC(0); //Fixed at no prescaler. Prescaler is only usefull for slow PWM
  FLEXPWM2_SM1CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_HALF | FLEXPWM_SMCTRL_PRSC(0); //Fixed at no prescaler. Prescaler is only usefull for slow PWM
  FLEXPWM2_SM2CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_HALF | FLEXPWM_SMCTRL_PRSC(0); //Fixed at no prescaler. Prescaler is only usefull for slow PWM
  FLEXPWM2_SM0VAL1 = (uint32_t)((float)F_BUS_ACTUAL / f_pwm - 1) / 2; //Set the modulus value (dictates the frequency)
  FLEXPWM2_SM1VAL1 = FLEXPWM2_SM0VAL1;  //Set the modulus value (dictates the frequency)
  FLEXPWM2_SM2VAL1 = FLEXPWM2_SM0VAL1;  //Set the modulus value (dictates the frequency)
  FLEXPWM2_SM0INIT = -FLEXPWM2_SM0VAL1; //Good for center aligned PWM, see manual
  FLEXPWM2_SM1INIT = -FLEXPWM2_SM1VAL1; //Good for center aligned PWM, see manual
  FLEXPWM2_SM2INIT = -FLEXPWM2_SM2VAL1; //Good for center aligned PWM, see manual

  FLEXPWM2_SM0VAL3 = FLEXPWM2_SM0VAL1 / 2; //50% duty cycle
  FLEXPWM2_SM0VAL2 = -FLEXPWM2_SM0VAL3; //50% duty cycle
  FLEXPWM2_SM1VAL3 = FLEXPWM2_SM0VAL1 / 2; //50% duty cycle
  FLEXPWM2_SM1VAL2 = -FLEXPWM2_SM0VAL3; //50% duty cycle
  FLEXPWM2_SM2VAL3 = FLEXPWM2_SM0VAL1 / 2; //50% duty cycle
  FLEXPWM2_SM2VAL2 = -FLEXPWM2_SM0VAL3; //50% duty cycle

  //int adc_shift = 0; //New idea. Do not shift the ADC. Probably the ADC takes a quick sample of the signal at the right moment, rest of the time is to get the value.
  //Back to old idea of shifting. Current measurements are better this way (less issue when high duty cycle:
  const int adc_shift = 0; //Tuned by making it finish at the switch moment, then divide offset by 2.

  FLEXPWM2_SM0VAL4 = 0 + adc_shift; // adc trigger 1
  FLEXPWM2_SM0VAL5 = FLEXPWM2_SM0VAL1 + adc_shift; // adc trigger 2


  FLEXPWM2_SM2VAL4 = FLEXPWM2_SM0VAL4; // adc trigger 1 to show on digital output 9
  FLEXPWM2_SM2VAL5 = FLEXPWM2_SM0VAL5; // adc trigger 2 to show on digital output 9

  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK( 7 );// Load Okay LDOK(SM) -> reload setting again

  //This triggers twice per PWM cycle:
  FLEXPWM2_SM0TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1 << 4) | FLEXPWM_SMTCTRL_OUT_TRIG_EN(1 << 5); //  val 4 of Flexpwm sm0 as trigger; #define FLEXPWM_SMTCTRL_OUT_TRIG_EN(n)   ((uint16_t)(((n) & 0x3F) << 0))

  //This triggers once per PWM cycle:
  //FLEXPWM2_SM0TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1 << 5); //  val 4 of Flexpwm sm0 as trigger; #define FLEXPWM_SMTCTRL_OUT_TRIG_EN(n)   ((uint16_t)(((n) & 0x3F) << 0))

  *(portConfigRegister(4)) = 1; //Set port 4 to the right mux value (found in pwm.c)
  *(portConfigRegister(5)) = 1; //Set port 5 to the right mux value (found in pwm.c)
  *(portConfigRegister(6)) = 2; //Set port 6 to the right mux value (found in pwm.c)

  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMB_EN( 4 ); // Activate B channel
  *(portConfigRegister(9)) = 2; //Set port 9 to the right mux value (found in pwm.c)
}


void flexpwm4_init() {     //set PWM
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_CLDOK( 7 );//  Clear Load Okay LDOK(SM) -> no reload of PWM settings
  FLEXPWM4_SM0CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_HALF | FLEXPWM_SMCTRL_PRSC(0); //Fixed at no prescaler. Prescaler is only usefull for slow PWM
  FLEXPWM4_SM1CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_HALF | FLEXPWM_SMCTRL_PRSC(0); //Fixed at no prescaler. Prescaler is only usefull for slow PWM
  FLEXPWM4_SM2CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_HALF | FLEXPWM_SMCTRL_PRSC(0); //Fixed at no prescaler. Prescaler is only usefull for slow PWM
  FLEXPWM4_SM0VAL1 = (uint32_t)((float)F_BUS_ACTUAL / f_pwm - 1) / 2; //Set the modulus value (dictates the frequency)
  FLEXPWM4_SM1VAL1 = FLEXPWM4_SM0VAL1;  //Set the modulus value (dictates the frequency)
  FLEXPWM4_SM2VAL1 = FLEXPWM4_SM0VAL1;  //Set the modulus value (dictates the frequency)
  FLEXPWM4_SM0INIT = -FLEXPWM4_SM0VAL1; //Good for center aligned PWM, see manual
  FLEXPWM4_SM1INIT = -FLEXPWM4_SM1VAL1; //Good for center aligned PWM, see manual
  FLEXPWM4_SM2INIT = -FLEXPWM4_SM2VAL1; //Good for center aligned PWM, see manual

  FLEXPWM4_SM0VAL3 = FLEXPWM4_SM0VAL1 / 2; //50% duty cycle
  FLEXPWM4_SM0VAL2 = -FLEXPWM4_SM0VAL3; //50% duty cycle
  FLEXPWM4_SM1VAL3 = FLEXPWM4_SM0VAL1 / 2; //50% duty cycle
  FLEXPWM4_SM1VAL2 = -FLEXPWM4_SM0VAL3; //50% duty cycle
  FLEXPWM4_SM2VAL3 = FLEXPWM4_SM0VAL1 / 2; //50% duty cycle
  FLEXPWM4_SM2VAL2 = -FLEXPWM4_SM0VAL3; //50% duty cycle

  //int adc_shift = 0; //New idea. Do not shift the ADC. Probably the ADC takes a quick sample of the signal at the right moment, rest of the time is to get the value.
  //Back to old idea of shifting. Current measurements are better this way (less issue when high duty cycle:
  const int adc_shift = 0; //Tuned by making it finish at the switch moment, then divide offset by 2.

  FLEXPWM4_SM0VAL4 = 0 + adc_shift; // adc trigger 1
  FLEXPWM4_SM0VAL5 = FLEXPWM4_SM0VAL1 + adc_shift; // adc trigger 2

  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_LDOK( 7 );// Load Okay LDOK(SM) -> reload setting again

  *(portConfigRegister(22)) = 1; //Set port 22 to the right mux value (found in pwm.c)
  *(portConfigRegister(23)) = 1; //Set port 23 to the right mux value (found in pwm.c)
  *(portConfigRegister(2)) = 1; //Set port 2 to the right mux value (found in pwm.c)
}

void syncflexpwm() {
  cli()
  //Set CTRL2 to the normal settings:
  FLEXPWM2_SM0CTRL2 = FLEXPWM_SMCTRL2_INDEP; //Enable Independent pair, but disable Debug Enable and WAIT Enable. When set to one, the PWM will continue to run while the chip is in debug/WAIT mode.
  FLEXPWM2_SM1CTRL2 = FLEXPWM_SMCTRL2_INDEP; //Enable Independent pair, but disable Debug Enable and WAIT Enable. When set to one, the PWM will continue to run while the chip is in debug/WAIT mode.
  FLEXPWM2_SM2CTRL2 = FLEXPWM_SMCTRL2_INDEP; //Enable Independent pair, but disable Debug Enable and WAIT Enable. When set to one, the PWM will continue to run while the chip is in debug/WAIT mode.
  FLEXPWM4_SM0CTRL2 = FLEXPWM_SMCTRL2_INDEP; //Enable Independent pair, but disable Debug Enable and WAIT Enable. When set to one, the PWM will continue to run while the chip is in debug/WAIT mode.
  FLEXPWM4_SM1CTRL2 = FLEXPWM_SMCTRL2_INDEP; //Enable Independent pair, but disable Debug Enable and WAIT Enable. When set to one, the PWM will continue to run while the chip is in debug/WAIT mode.
  FLEXPWM4_SM2CTRL2 = FLEXPWM_SMCTRL2_INDEP; //Enable Independent pair, but disable Debug Enable and WAIT Enable. When set to one, the PWM will continue to run while the chip is in debug/WAIT mode.

  FLEXPWM2_SM0CTRL2 |= FLEXPWM_SMCTRL2_FRCEN;  // pin 4
  FLEXPWM2_SM1CTRL2 |= FLEXPWM_SMCTRL2_INIT_SEL(2); // pin 5 Master sync from submodule 0 causes initialization.
  FLEXPWM2_SM2CTRL2 |= FLEXPWM_SMCTRL2_INIT_SEL(2); // pin 6 Master sync from submodule 0 causes initialization.

  FLEXPWM4_SM0CTRL2 |= FLEXPWM_SMCTRL2_FRCEN; // pin 22
  FLEXPWM4_SM1CTRL2 |= FLEXPWM_SMCTRL2_INIT_SEL(2); // pin 23 Master sync from submodule 0 causes initialization.
  FLEXPWM4_SM2CTRL2 |= FLEXPWM_SMCTRL2_INIT_SEL(2); // pin 2 Master sync from submodule 0 causes initialization.

  //Sync flexpwm2 and flexpwm4
  FLEXPWM2_SM0CTRL2 |= FLEXPWM_SMCTRL2_FORCE; // Force FlexPWM2
  FLEXPWM4_SM0CTRL2 |= FLEXPWM_SMCTRL2_FORCE; // Force FlexPWM4. This is about 50 ns later than force of flexpwm2. No idea on how to improve this

  sei();
}

void Encoders_init() {
  Encoder1.setInitConfig();
  //Optional filters
  Encoder1.EncConfig.filterCount = 3; //Bit of filtering to avoid spurious triggering.
  Encoder1.EncConfig.filterSamplePeriod = 3; //Bit of filtering to avoid spurious triggering.
  Encoder1.EncConfig.INDEXTriggerMode  = 1;
  Encoder1.EncConfig.IndexTrigger  = 1;
  Encoder1.EncConfig.positionInitialValue = 0;
  Encoder1.init();

  Encoder2.setInitConfig();
  Encoder2.EncConfig.filterCount = 3; //Bit of filtering to avoid spurious triggering.
  Encoder2.EncConfig.filterSamplePeriod = 3; //Bit of filtering to avoid spurious triggering.
  Encoder2.EncConfig.INDEXTriggerMode = 1;
  Encoder2.EncConfig.IndexTrigger  = 1;
  Encoder2.EncConfig.positionInitialValue = 0;
  Encoder2.init();
}

//////////////////////////////////////////////////////////////
// Start of the real time code
//////////////////////////////////////////////////////////////

void loop() {
}

void adcetc0_isr() {
  //  digitalWrite( debugpin , HIGH);
  ADC_ETC_DONE0_1_IRQ |= 1;   // clear
  is_v7 = (FLEXPWM2_SM0STS & FLEXPWM_SMSTS_CMPF(2));  //is_v7 = True when in v7
  FLEXPWM2_SM0STS |= FLEXPWM_SMSTS_CMPF(2); //Reset flag
  sens1 = (ADC_ETC_TRIG0_RESULT_1_0 & 4095) * 0.0008058608; // 4095.0 * 3.3;
  sens1_lp = lowpassIsens1->process( sens1 );
  sens3 = ((ADC_ETC_TRIG0_RESULT_1_0 >> 16) & 4095) * 0.0008058608; // 4095.0 * 3.3;
  sens3_lp = lowpassIsens3->process( sens3 );
  sensBus = (ADC_ETC_TRIG0_RESULT_3_2 & 4095) * Busadc2Vbus;   // 4095.0 * 3.3 * ((68.3+5.05)/5.05);
  if (sensBus > V_Bus + 1 ) {
    //    digitalWrite( chopperpin , HIGH);
  }
  else {
    //    digitalWrite( chopperpin , LOW);
  }
  if (sensBus > 47.25 ) {
    OutputOn = false;
    if (firsterror == 0) {
      firsterror = 41;
    }
  }
  //asm("dsb");
}

void adcetc1_isr() {
  //  digitalWrite( debugpin , LOW);
  curtime = micros();
  ADC_ETC_DONE0_1_IRQ |= 1 << 20;   // clear
  sens2 = (ADC_ETC_TRIG4_RESULT_1_0 & 4095) * 0.0008058608; // 4095.0 * 3.3;
  sens2_lp = lowpassIsens2->process( sens2 );
  sens4 = ((ADC_ETC_TRIG4_RESULT_1_0 >> 16) & 4095) * 0.0008058608; // 4095.0 * 3.3;
  sens4_lp = lowpassIsens4->process( sens4 );
  //asm("dsb");

  if (setupready == 1) {
    if (n_senscalib < 1e4) {
      n_senscalib++;
      sens1_calib += sens1_lp;
      sens2_calib += sens2_lp;
      sens3_calib += sens3_lp;
      sens4_calib += sens4_lp;
    }
    else if (n_senscalib == 1e4) {
      sens1_calib /= n_senscalib;
      sens2_calib /= n_senscalib;
      sens3_calib /= n_senscalib;
      sens4_calib /= n_senscalib;
      //      Serial.println(sens1_calib);
      //      Serial.println(sens2_calib);
      n_senscalib++;
    }
    else {
      updateDisturbance(); //Switched this before readENC to have a fresher encoder position
      GenSetpoint();
      readENC();
      Control();
      Transforms();
      changePWM();
      communicationProcess();
    }
  }
}

void updateDisturbance() {
  //PRBS
  downsamplePRBS++;
  if ( downsamplePRBS > NdownsamplePRBS) {
    downsamplePRBS = 1;
    noisebit  = ((lfsr >> 5) ^ (lfsr >> 7) ) & 1;  // taps: 11 9; feedback polynomial: x^11 + x^9 + 1
    lfsr =  (lfsr >> 1) | (noisebit << 15);
    ++period; // Dit lijkt nergens gebruikt te worden?
  }

  //Single Sine
  if ( curtime / 1e6 >= (ss_tstart + ss_n_aver / ss_f )) {
    ss_f += ss_fstep;
    ss_tstart = curtime / 1e6;
    if (ss_f > ss_fend)
    {
      ss_f = 0;
      ss_phase = 0;
      ss_tstart = 1e8;
      ss_gain = 0;
      ss_offset = 0;
    }
  }
  ss_phase += ss_f * 2 * M_PI * T;
  if ( ss_phase > 2 * M_PI) {
    ss_phase -= 2 * M_PI; //Required, because high value floats are inaccurate
  }
  float ss_offset_lp = lowpass_ss_offset->process( ss_offset );
  //ss_out = ss_offset_lp + ss_gain * arm_sin_f32( ss_phase ); //sin() measured to be faster then sinf(); arm_sin_f32() is way faster!
  ss_out = ss_offset_lp + ss_gain * sin( ss_phase );
  dist = distval * 1 * (noisebit - 0.5) + distoff + ss_out;
}


void GenSetpoint() {
  SPprofile->REFidir = SPdir;
  SPprofile->rdelay = rdelay;

  if (SPprofile->REFstatus != 1) { // if not running SP
    SPprofile->REFstatus = 0; //Make sure sp generator is ready for sp generation
    if (spNgo > 0) {
      spGO = 1;
      spNgo -= 1;
    }
    else {
      spGO = 0;
    }
  }
  REFstatus = SPprofile->REFstatus;
  rmech = SPprofile->stateCalculation( spGO );

  offsetVel_lp = lowpassSP->process( offsetVel );
  offsetVelTot += offsetVel_lp * T;
  rmech += offsetVelTot ;

  rmech2 = -rmech;
  rmech  += rmechoffset;
  rmech2 += rmechoffset2;

  acc = SPprofile->aref;
  vel = SPprofile->vref;

  acc2 = -acc;
  vel2 = -vel;

  //When no setpoint is running, allways convert reference to nearest encoder count to avoid noise
  if (SPprofile->REFstatus == 0 && offsetVel_lp == 0) {
    rmech = int((rmech / enc2rad)) * enc2rad;
  }
  if (SPprofile->REFstatus == 0 && offsetVel_lp == 0) {
    rmech2 = int((rmech2 / enc2rad2)) * enc2rad2;
  }

}


void readENC() {
  encoderPos1 = Encoder1.read();
  encoderPos2 = Encoder2.read();
  IndexFound1 = Encoder1.indexfound();
  IndexFound2 = Encoder2.indexfound();

}

void Control() {
  ymech1 = encoderPos1 * enc2rad;
  ymech2 = encoderPos2 * enc2rad2;
  emech1 = rmech - ymech1;
  emech2 = rmech2 - ymech2;
  if (haptic == 1) {
    emech1 = rmech - ymech1 - ymech2;
    emech2 = 0;
  }

  if (abs(emech1) > 0.5 & Kp > 0 )
  {
    OutputOn = false;
    if (firsterror == 0) {
      firsterror = 1;
    }
  }
  if (OutputOn == false) {
    emech1 = 0;
    integrator->setState(0);
    lowpass->InitStates(0);
  }
  //  if (abs(emech2) > 0.5 & Kp2 > 0 )
  //  {
  //    OutputOn = false;
  //    if (firsterror == 0) {
  //      firsterror = 21;
  //    }
  //  }
  if (OutputOn == false) {
    emech2 = 0;
    integrator2->setState(0);
    lowpass2->InitStates(0);
  }
  if (Kp == 0) {
    integrator->setState(0);
  }
  if (Kp2 == 0) {
    integrator2->setState(0);
  }

  mechcontout = Kp * leadlag->process( emech1 );
  mechcontout = lowpass->process( mechcontout );

  mechcontout2 = Kp2 * leadlag2->process( emech2 );
  mechcontout2 = lowpass2->process( mechcontout2 );

  Iout = integrator->processclip( mechcontout , -I_max * Kt_Nm_Apeak - mechcontout , I_max * Kt_Nm_Apeak - mechcontout );
  Iout2 = integrator2->processclip( mechcontout2 , -I_max * Kt_Nm_Apeak2 - mechcontout2 , I_max * Kt_Nm_Apeak2 - mechcontout2 );

  mechcontout += Iout;
  mechcontout += acc * Jload * OutputOn;
  mechcontout += vel * velFF * OutputOn;
  mechcontout += dist * mechdistgain;

  mechcontout2 += Iout2;
  mechcontout2 += acc2 * Jload2 * OutputOn;
  mechcontout2 += vel2 * velFF2 * OutputOn;
  mechcontout2 += dist * mechdistgain;

  if (haptic == 1) {
    mechcontout2 = mechcontout;
  }


  if (OutputOn == false) {
    integrator_Id->setState(0);
    integrator_Iq->setState(0);
    integrator_Id2->setState(0);
    integrator_Iq2->setState(0);
  }

  if (Kp != 0) {
    Iq_SP = mechcontout / Kt_Nm_Apeak;
  }


  Iq_SP2 = mechcontout2 / Kt_Nm_Apeak2;
}

void Transforms()
{
  // Calculate currents
  if (useIlowpass == 1)
  {
    ia = adc2A1 * (sens1_lp - sens1_calib);
    ib = adc2A1 * (sens2_lp - sens2_calib);
    ia2 = adc2A2 * (sens3_lp - sens3_calib);
    ib2 = adc2A2 * (sens4_lp - sens4_calib);
  }
  else {
    ia = adc2A1 * (sens1 - sens1_calib);
    ib = adc2A1 * (sens2 - sens2_calib);
    ia2 = adc2A2 * (sens3 - sens3_calib);
    ib2 = adc2A2 * (sens4 - sens4_calib);
  }

  // For Park and Clarke see https://www.cypress.com/file/222111/download
  // Power-variant Clarke transform. Asuming ia+ib+ic=0:
  Ialpha = ia;
  Ibeta = one_by_sqrt3 * ia + two_by_sqrt3 * ib;

  Ialpha2 = ia2;
  Ibeta2 = one_by_sqrt3 * ia2 + two_by_sqrt3 * ib2;

  // Park transform, ride the wave option %enccountperrev
  thetaPark_enc = 4 * (encoderPos1 % enccountperrev) * enc2rad + commutationoffset; //Modulo on the encoder counts to keep the floating point 0 to 2pi for numerical accuracy
  if (revercommutation1) {
    thetaPark_enc *= -1;
  }
  while ( thetaPark_enc > 2 * M_PI) {
    thetaPark_enc -= 2 * M_PI;
  }
  while ( thetaPark_enc < 0) {
    thetaPark_enc += 2 * M_PI;
  }

  //Angle observer by mxlemming
  float L = (Ld + Lq) / 2;
  BEMFa = BEMFa + (Valpha - R * Ialpha) * T -
          L * (Ialpha - Ialpha_last);
  BEMFb = BEMFb + (Vbeta - R * Ibeta) * T -
          L * (Ibeta - Ibeta_last)  ;
  Ialpha_last = Ialpha;
  Ibeta_last = Ibeta;
  if (BEMFa > Lambda_m  ) {
    BEMFa = Lambda_m ;
  }
  if (BEMFa < -Lambda_m ) {
    BEMFa = -Lambda_m ;
  }
  if (BEMFb > Lambda_m ) {
    BEMFb = Lambda_m ;
  }
  if (BEMFb < -Lambda_m ) {
    BEMFb = -Lambda_m ;
  }
  thetaPark_obs = fast_atan2(BEMFb, BEMFa);

  while ( thetaPark_obs > 2 * M_PI) {
    thetaPark_obs -= 2 * M_PI;
  }
  while ( thetaPark_obs < 0) {
    thetaPark_obs += 2 * M_PI;
  }


  //Angle observer (VESC)
  float L_ia = L * Ialpha;
  float L_ib = L * Ibeta;
  float gamma_half = observer_gain * 0.5;

  float err = sq(Lambda_m) - (sq(x1 - L_ia) + sq(x2 - L_ib));
  if (err > 0.0) {
    err = 0.0;
  }

  // Misschien Valpha en beta nog wat draaien voor motor rotatie?
  float x1_dot = Valpha - R * Ialpha + gamma_half * (x1 - L_ia) * err;
  float x2_dot = Vbeta  - R * Ibeta  + gamma_half * (x2 - L_ib) * err;
  x1 += x1_dot * T;
  x2 += x2_dot * T;

  UTILS_NAN_ZERO(x1);
  UTILS_NAN_ZERO(x2);

  // Prevent the magnitude from getting too low, as that makes the angle very unstable.
  float mag = NORM2_f(x1, x2);
  if (mag < (Lambda_m * 0.5)) {
    x1 *= 1.1;
    x2 *= 1.1;
  }
  thetaPark_vesc = utils_fast_atan2( x2 - L_ib, x1 - L_ia);
  while ( thetaPark_vesc > 2 * M_PI) {
    thetaPark_vesc -= 2 * M_PI;
  }
  while ( thetaPark_vesc < 0) {
    thetaPark_vesc += 2 * M_PI;
  }
  if (x1 > Lambda_m  ) {
    x1 = Lambda_m ;
  }
  if (x1 < -Lambda_m ) {
    x1 = -Lambda_m ;
  }
  if (x2 > Lambda_m ) {
    x2 = Lambda_m ;
  }
  if (x2 < -Lambda_m ) {
    x2 = -Lambda_m ;
  }




  if (anglechoice == 0) {
    thetaPark = thetaPark_enc;
  }
  else if (anglechoice == 1) {
    thetaPark = thetaPark_obs;
  }
  else if (anglechoice == 2) {
    thetaPark = thetaPark_vesc;
  }
  else {
    thetaPark = 0;
  }


  // erpm estimator
  edeltarad = thetaPark - thetaParkPrev;
  thetaParkPrev = thetaPark;
  if (edeltarad > M_PI) {
    edeltarad -= 2 * M_PI;
  }
  if (edeltarad < -M_PI) {
    edeltarad += 2 * M_PI;
  }
  eradpers_lp = lowpass_eradpers->process( edeltarad / T );
  erpm = eradpers_lp * 60 / 2 / M_PI;





  //  thetaPark2 = 8 * (encoderPos2 % enccountperrev2) * enc2rad2 + commutationoffset2; //Modulo on the encoder counts to keep the floating point 0 to 2pi for numerical accuracy
  //  while ( thetaPark2 > 2 * M_PI) {
  //    thetaPark2 -= 2 * M_PI;
  //  }
  //  while ( thetaPark2 < 0) {
  //    thetaPark2 += 2 * M_PI;
  //  }

  if (ridethewave == 1 ) {
    if ((IndexFound1) < 1 ) {
      thetaPark = thetawave;
      thetawave -= 10 * 2 * M_PI * T;
      Vq = 1.5;
    }
    else {
      Vq = 0;
      ridethewave = 2;
      thetawave = 0;
    }
  }

  if (ridethewave2 == 1 ) {
    if ((IndexFound2) < 1 ) {
      thetaPark2 = thetawave2;
      thetawave2 -= 10 * 2 * M_PI * T;
      Vq2 = 1.5;
    }
    else {
      Vq2 = 0;
      ridethewave2 = 2;
      thetawave2 = 0;
    }
  }

  Iq_SP += muziek_gain * muziek[ (curloop / (50 / Ts)) % (sizeof(muziek) / 4) ];
  Iq_SP += dist * Iq_distgain;

  Id_SP = Id_offset_SP;
  Id_SP += dist * Id_distgain;

  Iq_SP2 += muziek_gain * muziek[ (curloop / (50 / Ts)) % (sizeof(muziek) / 4) ];
  Iq_SP2 += dist * Iq_distgain;

  Id_SP2 = Id_offset_SP2;
  Id_SP2 += dist * Id_distgain;



  co = cos(thetaPark);
  si = sin(thetaPark);

  co2 = cos(thetaPark2);
  si2 = sin(thetaPark2);


  // Park transform
  Id_meas = co * Ialpha + si * Ibeta;
  Iq_meas = co * Ibeta  - si * Ialpha;

  Id_meas2 = co2 * Ialpha2 + si2 * Ibeta2;
  Iq_meas2 = co2 * Ibeta2  - si2 * Ialpha2;

  if (useIlowpass == 2)
  {
    Id_meas = lowpassId1->process( Id_meas );
    Iq_meas = lowpassIq1->process( Iq_meas );
    Id_meas2 = lowpassId2->process( Id_meas2 );
    Iq_meas2 = lowpassIq2->process( Iq_meas2 );
  }

  float maxVolt = maxDutyCycle * sensBus * one_by_sqrt3;
  if (ridethewave != 1 ) {
    if (OutputOn == true) {
      Id_e = Id_SP - Id_meas;
      Iq_e = Iq_SP - Iq_meas;
    }
    else {
      Id_e = 0;
      Iq_e = 0;
    }
    Vq = Icontgain * Iq_e;
    Vq += integrator_Iq->process( Vq );


    if ( abs( Vq ) > maxVolt) {
      if (Vq > 0) {
        Vq = maxVolt;
      }
      else {
        Vq = -maxVolt;
      }
      integrator_Iq->setState( Vq );
    }

    //Additional Vq
    Vq += VSP;
    Vq += dist * Vq_distgain;

    Vd = Icontgain * Id_e;
    Vd += integrator_Id->process( Vd  );

    if ( abs( Vd ) > maxVolt) {
      if (Vd > 0) {
        Vd = maxVolt;
      }
      else {
        Vd = -maxVolt;
      }
      integrator_Id->setState( Vd );
    }

    //Additional Vd
    Vd += dist * Vd_distgain;


    we = vel * 4;  //Electrical speed [rad/s], based on setpoint

    // PMSM decoupling control and BEMF FF
    VqFF = we * ( Ld * Id_meas + Lambda_m);

    // q axis induction FFW based on setpoint FFW
    VqFF += SPprofile->jref * Jload * Lq * Kt_Nm_Apeak * OutputOn;

    Vq += VqFF;
    VdFF = -we * Lq * Iq_meas;
    Vd += VdFF;


  }

  Vq += muziek_gain_V * muziek[ (curloop / (50 / Ts)) % (sizeof(muziek) / 4) ];


  float Vtot = NORM2_f( Vd , Vq );
  if ( Vtot > maxVolt) {
    if ( abs( Vd ) > maxVolt) {
      if (Vd > 0) {
        Vd = maxVolt;
      }
      else {
        Vd = -maxVolt;
      }
    }
    if ( Vq > 0 ) {
      Vq = sqrt(sq(maxVolt) - sq(Vd)) ;
    }
    else {
      Vq = -sqrt(sq(maxVolt) - sq(Vd)) ;
    }
  }

  // Inverse park transform
  Valpha = co * Vd - si * Vq;
  Vbeta  = co * Vq + si * Vd;

  if ( hfi_on ) {
    hfi_cursample += 1;
    if (( hfi_cursample > 0) && (hfi_cursample <= hfi_maxsamples )) {
      if (is_v7) { //Remeber that we get the response of 2 cycles ago here
        hfi_curprev    = (Ialpha * cos(hfi_dir) + Ibeta  * sin(hfi_dir) );
        hfi_curortprev = (Ibeta  * cos(hfi_dir) - Ialpha * sin(hfi_dir) );        
      }
      else {
        float cur_act = Ialpha * cos(hfi_dir) + Ibeta  * sin(hfi_dir);
        hfi_curtot    += (cur_act - hfi_curprev);
        float cur_actort = Ibeta  * cos(hfi_dir) - Ialpha * sin(hfi_dir);
        hfi_curorttot += (cur_actort - hfi_curortprev);       
      }
    }
      //  Id_meas = co * Ialpha + si * Ibeta;
      //  Iq_meas = co * Ibeta  - si * Ialpha;

      
    // Inverse park transform (only voltage on angle)
    //  Valpha = co * Vd;
    //  Vbeta  = si * Vd;

    // Update hfi_fir here
    // Reset cursample counter

    if (is_v7) {
      Valpha_offset_hfi = -hfi_V * cos(hfi_dir);
      Vbeta_offset_hfi = -hfi_V * sin(hfi_dir);
    }
    else {
      Valpha_offset_hfi = hfi_V * cos(hfi_dir);
      Vbeta_offset_hfi = hfi_V * sin(hfi_dir);
    }
  }
  else {
    Valpha_offset_hfi = 0;
    Vbeta_offset_hfi = 0;
  }


  Valpha += Valpha_offset + Valpha_offset_hfi;
  Vbeta += Vbeta_offset + Vbeta_offset_hfi;

  // Inverse Power-variant Clarke transform
  Va = Valpha;
  Vb = -0.5 * Valpha + sqrt3_by_2 * Vbeta;
  Vc = -0.5 * Valpha - sqrt3_by_2 * Vbeta;

  //See https://microchipdeveloper.com/mct5001:start Zero Sequence Modulation Tutorial
  float Vcm = -(max(max(Va, Vb), Vc) + min(min(Va, Vb), Vc)) / 2;
  Va += Vcm + sensBus / 2;
  Vb += Vcm + sensBus / 2;
  Vc += Vcm + sensBus / 2;

  //Calculate modulation times
  tA = Va / sensBus;
  tB = Vb / sensBus;
  tC = Vc / sensBus;

  //Motor 2:
  if (ridethewave2 != 1 ) {
    if (OutputOn == true) {
      Id_e2 = Id_SP2 - Id_meas2;
      Iq_e2 = Iq_SP2 - Iq_meas2;
    }
    else {
      Id_e2 = 0;
      Iq_e2 = 0;
    }
    Vq2 = Icontgain2 * Iq_e2;
    Vq2 += integrator_Iq2->processclip( Vq2 , -V_Bus - Vq2 , V_Bus - Vq2 );

    //Additional Vq
    Vq2 += VSP;
    Vq2 += dist * Vq_distgain;

    Vd2 = Icontgain2 * Id_e2;
    Vd2 += integrator_Id2->processclip( Vd2 , -V_Bus - Vd2 , V_Bus - Vd2 );

    //Additional Vd
    Vd2 += dist * Vd_distgain;

    we2 = vel2 * 8;  //Electrical speed [rad/s], based on setpoint

    // PMSM decoupling control and BEMF FF
    VqFF2 = we2 * ( Ld2 * Id_meas2 + Lambda_m2);

    // q axis induction FFW based on setpoint FFW
    VqFF2 += SPprofile->jref * Jload2 * Lq2 * Kt_Nm_Apeak2 * OutputOn;

    Vq2 += VqFF2;
    VdFF2 = -we2 * Lq2 * Iq_meas2;
    Vd2 += VdFF2;
  }

  Vq2 += muziek_gain_V * muziek[ (curloop / (50 / Ts)) % (sizeof(muziek) / 4) ];

  // Inverse park transform
  Valpha2 = co2 * Vd2 - si2 * Vq2;
  Vbeta2  = co2 * Vq2 + si2 * Vd2;

  Valpha2 += Valpha2_offset;

  // Inverse Power-variant Clarke transform
  Va2 = Valpha2;
  Vb2 = -0.5 * Valpha2 + sqrt3_by_2 * Vbeta2;
  Vc2 = -0.5 * Valpha2 - sqrt3_by_2 * Vbeta2;

  //See https://microchipdeveloper.com/mct5001:start Zero Sequence Modulation Tutorial
  // These lines make SVM happen:
  float Vcm2 = -(max(max(Va2, Vb2), Vc2) + min(min(Va2, Vb2), Vc2)) / 2;
  Va2 += Vcm2 + sensBus / 2;
  Vb2 += Vcm2 + sensBus / 2;
  Vc2 += Vcm2 + sensBus / 2;

  //Calculate modulation times
  tA2 = Va2 / sensBus;
  tB2 = Vb2 / sensBus;
  tC2 = Vc2 / sensBus;

}

void changePWM() {
  if ( (abs(tA - 0.5) > 0.499) || (abs(tB - 0.5) > 0.499) || (abs(tC - 0.5) > 0.499)  ) {
    Novervolt += 1;
    if (Novervolt >= 5) {
      OutputOn = false;
      if (firsterror == 0) {
        firsterror = 2;
      }
    }
  }
  else {
    Novervolt = 0;
  }

  if ( (abs(tA2 - 0.5) > 0.499) || (abs(tB2 - 0.5) > 0.499) || (abs(tC2 - 0.5) > 0.499)  ) {
    Novervolt2 += 1;
    if (Novervolt2 >= 5) {
      OutputOn = false;
      if (firsterror == 0) {
        firsterror = 22;
      }
    }
  }
  else {
    Novervolt2 = 0;
  }


  if ( (abs(Id_SP) > I_max) || (abs(Iq_SP) > I_max) ) {
    OutputOn = false;
    if (firsterror == 0) {
      firsterror = 3;
    }
  }
  if ( (abs(Id_meas) > I_max) || (abs(Iq_meas) > I_max)  ) {
    OutputOn = false;
    if (firsterror == 0) {
      firsterror = 4;
    }
  }

  //  if ( (abs(Id_SP2) > I_max) || (abs(Iq_SP2) > I_max) ) {
  //    OutputOn = false;
  //    if (firsterror == 0) {
  //      firsterror = 23;
  //    }
  //  }
  //  if ( (abs(Id_meas2) > I_max) || (abs(Iq_meas2) > I_max)  ) {
  //    OutputOn = false;
  //    if (firsterror == 0) {
  //      firsterror = 24;
  //    }
  //  }

  //  //Motor 1, flexpwm2:
  //  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK( 7 );  //Enable changing of settings
  //  if (OutputOn == false) {
  //    FLEXPWM2_SM0VAL3 = FLEXPWM2_SM0VAL1 / 2;
  //    FLEXPWM2_SM1VAL3 = FLEXPWM2_SM1VAL1 / 2;
  //    FLEXPWM2_SM2VAL3 = FLEXPWM2_SM2VAL1 / 2;
  //  }
  //  else {
  //    // Set duty cycles. FTM3_MOD = 100% (1800 for current settings, 20 kHz).
  //    FLEXPWM2_SM0VAL3 = FLEXPWM2_SM0VAL1 * tA;
  //    FLEXPWM2_SM1VAL3 = FLEXPWM2_SM1VAL1 * tB;
  //    FLEXPWM2_SM2VAL3 = FLEXPWM2_SM2VAL1 * tC;
  //  }
  //  FLEXPWM2_SM0VAL2 = -FLEXPWM2_SM0VAL3;
  //  FLEXPWM2_SM1VAL2 = -FLEXPWM2_SM1VAL3;
  //  FLEXPWM2_SM2VAL2 = -FLEXPWM2_SM2VAL3;
  //  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK( 7 ); //Activate settings

  //Motor 1, flexpwm4:
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_CLDOK( 7 );  //Enable changing of settings
  if (OutputOn == false) {
    digitalWrite( engate , 0);
    FLEXPWM4_SM0VAL3 = FLEXPWM4_SM0VAL1 / 2;
    FLEXPWM4_SM1VAL3 = FLEXPWM4_SM1VAL1 / 2;
    FLEXPWM4_SM2VAL3 = FLEXPWM4_SM2VAL1 / 2;
  }
  else {
    // Set duty cycles. FTM3_MOD = 100% (1800 for current settings, 20 kHz).
    digitalWrite( engate , 1);
    FLEXPWM4_SM0VAL3 = FLEXPWM4_SM0VAL1 * tB;
    FLEXPWM4_SM1VAL3 = FLEXPWM4_SM1VAL1 * tC;
    FLEXPWM4_SM2VAL3 = FLEXPWM4_SM2VAL1 * tA;
  }
  FLEXPWM4_SM0VAL2 = -FLEXPWM4_SM0VAL3;
  FLEXPWM4_SM1VAL2 = -FLEXPWM4_SM1VAL3;
  FLEXPWM4_SM2VAL2 = -FLEXPWM4_SM2VAL3;
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_LDOK( 7 ); //Activate settings
  //  digitalWrite( debugpin , HIGH);
}

void communicationProcess() {
  processSerialIn();

  curloop++;
  if (Nsend > 0) {
    downsample--;
    if ( downsample < 1) {
      downsample = Ndownsample;
      bf.uint  = curloop;        Serial.write( bf.bin , 4);
      bf.uint  = curtime;        Serial.write( bf.bin , 4);
      trace( );
      Nsend--;
    }
  }
  if (sendall > 0 & Nsend == 0) {
    if (sendall == 1) {
      for (int i = 0; i < 14; ++i)
      {
        tracearray[i] = i;
      }
    }
    if (sendall > 1) {
      for (int i = 0; i < 14; ++i)
      {
        tracearray[i] += 14;
      }
    }
    trace( );
    sendall++;
    if ((sendall - 1) * 14 >= 204 ) {
      sendall = 0;
    }
  }

  //  digitalWrite( debugpin , LOW);
}

void xbar_connect(unsigned int input, unsigned int output)
{
  if (input >= 88) return;

  if (output >= 132) return;
  volatile uint16_t *xbar = &XBARA1_SEL0 + (output / 2);
  uint16_t val = *xbar;
  if (!(output & 1)) {
    val = (val & 0xFF00) | input;
  } else {
    val = (val & 0x00FF) | (input << 8);
  }
  *xbar = val;
}


void processSerialIn() {
  if (Serial.available() > 4) {
    char settingByte = Serial.read();
    for ( int i = 0; i < 4; i++) {
      ser_in.bin[i] = Serial.read();
    }
    if (settingByte == 's') {
      ss_gain = ser_in.fp;
      ss_f = ss_fstart;
      ss_phase = 0;
      ss_tstart = (timePrev + Ts) / 1e6; //timePrev gebruik ik niet meer?!!
    }
    if (settingByte == 'o') {
      OutputOn = true;
      offsetVelTot = 0;
      //      encoderPos1 = 0;
      //      encoderPos2 = 0;
      // Reset positions to zero, as the floating point number is most accurate here
      SPprofile->REFqmem = 0;
      if (haptic == 1) {
        rmechoffset = ymech1 + ymech2;
      }
      else {
        rmechoffset = ymech1;
      }
      rmechoffset2 = ymech2;

      integrator->setState(0);
      integrator_Id->setState(0);
      integrator_Iq->setState(0);
      integrator2->setState(0);
      integrator_Id2->setState(0);
      integrator_Iq2->setState(0);

      firsterror = 0;
    }
    if (settingByte == 'p') {
      if (spGO == 0) {
        spGO = 1;
      }
      else {
        spGO = 0;
      }
    }
    if (settingByte == '1') {
      SPprofile->t1 = ser_in.fp;
    }
    if (settingByte == '2') {
      SPprofile->t2 = ser_in.fp;
    }
    if (settingByte == '3') {
      SPprofile->t3 = ser_in.fp;
    }
    if (settingByte == '4') {
      SPprofile->p = ser_in.fp;
    }
    if (settingByte == '5') {
      SPprofile->v_max = ser_in.fp;
    }
    if (settingByte == '6') {
      SPprofile->a_max = ser_in.fp;
    }
    if (settingByte == '7') {
      SPprofile->j_max = ser_in.fp;
      SPprofile->init();
    }
    if (settingByte == '8') {
      int adc_shift = ser_in.sint;
      FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK( 7 );//  Clear Load Okay LDOK(SM) -> no reload of PWM settings
      FLEXPWM2_SM0VAL4 = 0 + adc_shift; // adc trigger
      FLEXPWM2_SM0VAL5 = FLEXPWM2_SM0VAL1 + adc_shift; // adc trigger
      FLEXPWM2_SM2VAL4 = FLEXPWM2_SM0VAL4; // adc trigger output
      FLEXPWM2_SM2VAL5 = FLEXPWM2_SM0VAL5; // adc trigger output
      FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK( 7 );// Load Okay LDOK(SM) -> reload setting again
    }

    if (settingByte == '9') {
      integrator_Id = new Integrator( ser_in.fp , 1 / T);
      integrator_Iq = new Integrator( ser_in.fp , 1 / T);
    }
    if (settingByte == '0') {
      integrator_Id2 = new Integrator( ser_in.fp , 1 / T);
      integrator_Iq = new Integrator( ser_in.fp , 1 / T);
    }


    if (settingByte == 't') {
      tracearray[Serial.read()] = ser_in.uint;
    }
    if (settingByte == 'T') {
      printSignals( ser_in.uint );
    }
    if (settingByte == 'S') {
      for ( int i = 0; i < 4; i++) {
        bf.bin[i] = Serial.read();
      }
      setpar( ser_in.uint , bf );
    }

    if (settingByte == 'C') {
      ContSelect = ser_in.uint;
      if ( ContSelect == 1) {
        Kp = 23.31;
        fBW = 100.0;
        alpha1 = 3.0;
        alpha2 = 4.0;
        fInt = fBW / 6.0;
        fLP = fBW * 8.0;

        Kp2 = 23.31;
        fBW2 = 100.0;
        alpha1_2 = 3.0;
        alpha2_2 = 4.0;
        fInt2 = fBW2 / 6.0;
        fLP2 = fBW2 * 8.0;
      }
      if ( ContSelect == 2) {
        Kp = 5;
        fBW = 30;
        alpha1 = 3.0;
        alpha2 = 4.0;
        fInt = 0;
        fLP = fBW * 8.0;

        Kp2 = 5;
        fBW2 = 30;
        alpha1_2 = 3.0;
        alpha2_2 = 4.0;
        fInt2 = 0;
        fLP2 = fBW2 * 8.0;
      }
      if ( ContSelect == 3) {
        Kp2 = 8.3;
        fBW = 100;
        alpha1 = 3;
        alpha2 = 3;
        fInt = fBW / 6;
        fLP = fBW * 8;

        Kp2 = 8.3;
        fBW2 = 100;
        alpha1_2 = 3;
        alpha2_2 = 3;
        fInt2 = fBW2 / 6;
        fLP2 = fBW2 * 8;

      }
      if ( ContSelect == 4) {
        Kp = 150;
        fBW = 400;
        alpha1 = 5;
        alpha2 = 5;
        fInt = fBW / 8;
        fLP = fBW * 8;

        Kp2 = 150;
        fBW2 = 400;
        alpha1_2 = 5;
        alpha2_2 = 5;
        fInt2 = fBW2 / 8;
        fLP2 = fBW2 * 8;
      }
      if ( ContSelect == 5) {
        Kp = 0;
        fBW = 400;
        alpha1 = 5;
        alpha2 = 5;
        fInt = fBW / 8;
        fLP = fBW * 8;

        Kp2 = 0;
        fBW2 = 400;
        alpha1_2 = 5;
        alpha2_2 = 5;
        fInt2 = fBW2 / 8;
        fLP2 = fBW2 * 8;
      }
      integrator = new Integrator( fInt , 1 / T);
      leadlag       = new LeadLag( fBW , alpha1 , alpha2 , 1 / T);
      lowpass        = new Biquad( bq_type_lowpass , fLP , 0.7, 1 / T);
      integrator2 = new Integrator( fInt2 , 1 / T);
      leadlag2       = new LeadLag( fBW2 , alpha1_2 , alpha2_2 , 1 / T);
      lowpass2        = new Biquad( bq_type_lowpass , fLP2 , 0.7 , 1 / T);
    }
  }
}

float utils_fast_atan2(float y, float x) {
  float abs_y = fabsf(y) + 1e-20; // kludge to prevent 0/0 condition
  float angle;

  if (x >= 0) {
    float r = (x - abs_y) / (x + abs_y);
    float rsq = r * r;
    angle = ((0.1963 * rsq) - 0.9817) * r + (M_PI / 4.0);
  } else {
    float r = (x + abs_y) / (abs_y - x);
    float rsq = r * r;
    angle = ((0.1963 * rsq) - 0.9817) * r + (3.0 * M_PI / 4.0);
  }

  UTILS_NAN_ZERO(angle);

  if (y < 0) {
    return (-angle);
  } else {
    return (angle);
  }
}

float fast_atan2(float y, float x) {
  // a := min (|x|, |y|) / max (|x|, |y|)
  float abs_y = fabs(y);
  float abs_x = fabs(x);
  // inject FLT_MIN in denominator to avoid division by zero
  float a = min(abs_x, abs_y) / (max(abs_x, abs_y));
  // s := a * a
  float s = a * a;
  // r := ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a
  float r =
    ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
  // if |y| > |x| then r := 1.57079637 - r
  if (abs_y > abs_x) r = 1.57079637f - r;
  // if x < 0 then r := 3.14159274 - r
  if (x < 0.0f) r = 3.14159274f - r;
  // if y < 0 then r := -r
  if (y < 0.0f) r = -r;

  return r;
}



void trace( ) {
  for( int i = 0; i<14; i++){
    int isignal = tracearray[i];
    switch( isignal ){
      case   0: bf.fp   = maxDutyCycle; break;
      case   1: bf.fp   = BEMFa; break;
      case   2: bf.fp   = BEMFb; break;
      case   3: bf.fp   = Ialpha_last; break;
      case   4: bf.fp   = Ibeta_last; break;
      case   5: bf.fp   = commutationoffset; break;
      case   6: bf.fp   = DQdisturbangle; break;
      case   7: bf.fp   = Vq; break;
      case   8: bf.fp   = Vd; break;
      case   9: bf.fp   = Valpha; break;
      case  10: bf.fp   = Vbeta; break;
      case  11: bf.fp   = thetawave; break;
      case  12: bf.fp   = Id_meas; break;
      case  13: bf.fp   = Iq_meas; break;
      case  14: bf.fp   = VSP; break;
      case  15: bf.fp   = commutationoffset2; break;
      case  16: bf.fp   = DQdisturbangle2; break;
      case  17: bf.fp   = Vq2; break;
      case  18: bf.fp   = Vd2; break;
      case  19: bf.fp   = Valpha2; break;
      case  20: bf.fp   = Vbeta2; break;
      case  21: bf.fp   = thetawave2; break;
      case  22: bf.fp   = Id_meas2; break;
      case  23: bf.fp   = Iq_meas2; break;
      case  24: bf.fp   = Va; break;
      case  25: bf.fp   = Vb; break;
      case  26: bf.fp   = Vc; break;
      case  27: bf.fp   = Va2; break;
      case  28: bf.fp   = Vb2; break;
      case  29: bf.fp   = Vc2; break;
      case  30: bf.fp   = adc2A1; break;
      case  31: bf.fp   = adc2A2; break;
      case  32: bf.fp   = one_by_sqrt3; break;
      case  33: bf.fp   = two_by_sqrt3; break;
      case  34: bf.fp   = sqrt_two_three; break;
      case  35: bf.fp   = sqrt3_by_2; break;
      case  36: bf.fp   = mechcontout; break;
      case  37: bf.fp   = Iout; break;
      case  38: bf.fp   = mechcontout2; break;
      case  39: bf.fp   = Iout2; break;
      case  40: bf.fp   = muziek_gain; break;
      case  41: bf.fp   = muziek_gain_V; break;
      case  42: bf.fp   = distval; break;
      case  43: bf.fp   = distoff; break;
      case  44: bf.fp   = ss_phase; break;
      case  45: bf.fp   = ss_fstart; break;
      case  46: bf.fp   = ss_fstep; break;
      case  47: bf.fp   = ss_fend; break;
      case  48: bf.fp   = ss_gain; break;
      case  49: bf.fp   = ss_offset; break;
      case  50: bf.fp   = ss_f; break;
      case  51: bf.fp   = ss_tstart; break;
      case  52: bf.fp   = ss_out; break;
      case  53: bf.fp   = T; break;
      case  54: bf.fp   = enc2rad; break;
      case  55: bf.fp   = enc2rad2; break;
      case  56: bf.fp   = I_max; break;
      case  57: bf.fp   = V_Bus; break;
      case  58: bf.fp   = rmech; break;
      case  59: bf.fp   = rdelay; break;
      case  60: bf.fp   = emech1; break;
      case  61: bf.fp   = ymech1; break;
      case  62: bf.fp   = rmech2; break;
      case  63: bf.fp   = emech2; break;
      case  64: bf.fp   = ymech2; break;
      case  65: bf.fp   = rmechoffset; break;
      case  66: bf.fp   = rmechoffset2; break;
      case  67: bf.fp   = Kp; break;
      case  68: bf.fp   = fBW; break;
      case  69: bf.fp   = alpha1; break;
      case  70: bf.fp   = alpha2; break;
      case  71: bf.fp   = fInt; break;
      case  72: bf.fp   = fLP; break;
      case  73: bf.fp   = Kp2; break;
      case  74: bf.fp   = fBW2; break;
      case  75: bf.fp   = alpha1_2; break;
      case  76: bf.fp   = alpha2_2; break;
      case  77: bf.fp   = fInt2; break;
      case  78: bf.fp   = fLP2; break;
      case  79: bf.fp   = Vout; break;
      case  80: bf.fp   = fIntCur; break;
      case  81: bf.fp   = Icontgain; break;
      case  82: bf.fp   = Vout2; break;
      case  83: bf.fp   = fIntCur2; break;
      case  84: bf.fp   = Icontgain2; break;
      case  85: bf.fp   = sensCalVal1; break;
      case  86: bf.fp   = sensCalVal2; break;
      case  87: bf.fp   = sensCalVal3; break;
      case  88: bf.fp   = sensCalVal4; break;
      case  89: bf.fp   = sens1; break;
      case  90: bf.fp   = sens2; break;
      case  91: bf.fp   = sens3; break;
      case  92: bf.fp   = sens4; break;
      case  93: bf.fp   = sens1_lp; break;
      case  94: bf.fp   = sens2_lp; break;
      case  95: bf.fp   = sens3_lp; break;
      case  96: bf.fp   = sens4_lp; break;
      case  97: bf.fp   = sens1_calib; break;
      case  98: bf.fp   = sens2_calib; break;
      case  99: bf.fp   = sens3_calib; break;
      case 100: bf.fp   = sens4_calib; break;
      case 101: bf.fp   = sensBus; break;
      case 102: bf.fp   = Jload; break;
      case 103: bf.fp   = velFF; break;
      case 104: bf.fp   = R; break;
      case 105: bf.fp   = Jload2; break;
      case 106: bf.fp   = velFF2; break;
      case 107: bf.fp   = offsetVelTot; break;
      case 108: bf.fp   = offsetVel; break;
      case 109: bf.fp   = offsetVel_lp; break;
      case 110: bf.fp   = acc; break;
      case 111: bf.fp   = vel; break;
      case 112: bf.fp   = dist; break;
      case 113: bf.fp   = Ialpha; break;
      case 114: bf.fp   = Ibeta; break;
      case 115: bf.fp   = thetaPark; break;
      case 116: bf.fp   = thetaParkPrev; break;
      case 117: bf.fp   = edeltarad; break;
      case 118: bf.fp   = eradpers_lp; break;
      case 119: bf.fp   = erpm; break;
      case 120: bf.fp   = thetaPark_enc; break;
      case 121: bf.fp   = thetaPark_obs; break;
      case 122: bf.fp   = thetaPark_vesc; break;
      case 123: bf.fp   = co; break;
      case 124: bf.fp   = si; break;
      case 125: bf.fp   = D; break;
      case 126: bf.fp   = Q; break;
      case 127: bf.fp   = tA; break;
      case 128: bf.fp   = tB; break;
      case 129: bf.fp   = tC; break;
      case 130: bf.fp   = Id_e; break;
      case 131: bf.fp   = Id_SP; break;
      case 132: bf.fp   = Iq_e; break;
      case 133: bf.fp   = Iq_SP; break;
      case 134: bf.fp   = ia; break;
      case 135: bf.fp   = ib; break;
      case 136: bf.fp   = ic; break;
      case 137: bf.fp   = acc2; break;
      case 138: bf.fp   = vel2; break;
      case 139: bf.fp   = Ialpha2; break;
      case 140: bf.fp   = Ibeta2; break;
      case 141: bf.fp   = thetaPark2; break;
      case 142: bf.fp   = co2; break;
      case 143: bf.fp   = si2; break;
      case 144: bf.fp   = D2; break;
      case 145: bf.fp   = Q2; break;
      case 146: bf.fp   = tA2; break;
      case 147: bf.fp   = tB2; break;
      case 148: bf.fp   = tC2; break;
      case 149: bf.fp   = Id_e2; break;
      case 150: bf.fp   = Id_SP2; break;
      case 151: bf.fp   = Iq_e2; break;
      case 152: bf.fp   = Iq_SP2; break;
      case 153: bf.fp   = ia2; break;
      case 154: bf.fp   = ib2; break;
      case 155: bf.fp   = ic2; break;
      case 156: bf.fp   = Vq_distgain; break;
      case 157: bf.fp   = Vd_distgain; break;
      case 158: bf.fp   = Iq_distgain; break;
      case 159: bf.fp   = Id_distgain; break;
      case 160: bf.fp   = mechdistgain; break;
      case 161: bf.fp   = Kt_Nm_Arms; break;
      case 162: bf.fp   = Kt_Nm_Apeak; break;
      case 163: bf.fp   = we; break;
      case 164: bf.fp   = Ld; break;
      case 165: bf.fp   = Lq; break;
      case 166: bf.fp   = Lambda_m; break;
      case 167: bf.fp   = observer_gain; break;
      case 168: bf.fp   = x1; break;
      case 169: bf.fp   = x2; break;
      case 170: bf.fp   = Kt_Nm_Arms2; break;
      case 171: bf.fp   = Kt_Nm_Apeak2; break;
      case 172: bf.fp   = we2; break;
      case 173: bf.fp   = Ld2; break;
      case 174: bf.fp   = Lq2; break;
      case 175: bf.fp   = Lambda_m2; break;
      case 176: bf.fp   = hfi_V; break;
      case 177: bf.fp   = hfi_dir; break;
      case 178: bf.fp   = Valpha_offset_hfi; break;
      case 179: bf.fp   = Vbeta_offset_hfi; break;
      case 180: bf.fp   = hfi_curtot; break;
      case 181: bf.fp   = hfi_curorttot; break;
      case 182: bf.fp   = VqFF; break;
      case 183: bf.fp   = VdFF; break;
      case 184: bf.fp   = VqFF2; break;
      case 185: bf.fp   = VdFF2; break;
      case 186: bf.fp   = Id_offset_SP; break;
      case 187: bf.fp   = Id_offset_SP2; break;
      case 188: bf.fp   = Valpha_offset; break;
      case 189: bf.fp   = Vbeta_offset; break;
      case 190: bf.fp   = Valpha2_offset; break;
      case 191: bf.sint = Ts; break;
      case 192: bf.sint = anglechoice; break;
      case 193: bf.sint = timeremain; break;
      case 194: bf.sint = spNgo; break;
      case 195: bf.sint = REFstatus; break;
      case 196: bf.sint = incomingByte; break;
      case 197: bf.sint = encoderPos1; break;
      case 198: bf.sint = encoderPos2; break;
      case 199: bf.sint = enccountperrev; break;
      case 200: bf.sint = enccountperrev2; break;
      case 201: bf.sint = SP_input_status; break;
      case 202: bf.sint = spGO; break;
      case 203: bf.sint = hfi_cursample; break;
      case 204: bf.sint = hfi_maxsamples; break;
      case 205: bf.uint = ridethewave; break;
      case 206: bf.uint = ridethewave2; break;
      case 207: bf.uint = sendall; break;
      case 208: bf.uint = curloop; break;
      case 209: bf.uint = Ndownsample; break;
      case 210: bf.uint = downsample; break;
      case 211: bf.uint = Novervolt; break;
      case 212: bf.uint = Novervolt2; break;
      case 213: bf.uint = NdownsamplePRBS; break;
      case 214: bf.uint = downsamplePRBS; break;
      case 215: bf.uint = ss_n_aver; break;
      case 216: bf.uint = Nsend; break;
      case 217: bf.uint = timePrev; break;
      case 218: bf.uint = curtime; break;
      case 219: bf.uint = overloadcount; break;
      case 220: bf.uint = useIlowpass; break;
      case 221: bf.uint = ContSelect; break;
      case 222: bf.uint = firsterror; break;
      case 223: bf.uint = N_pp; break;
      case 224: bf.uint = N_pp2; break;
      case 225: bf.bl   = SPdir; break;
      case 226: bf.bl   = is_v7; break;
      case 227: bf.bl   = haptic; break;
      case 228: bf.bl   = revercommutation1; break;
      case 229: bf.bl   = IndexFound1; break;
      case 230: bf.bl   = IndexFound2; break;
      case 231: bf.bl   = OutputOn; break;
      case 232: bf.bl   = hfi_on; break;
    }
    Serial.write( bf.bin , 4);
  }
}

void setpar( int isignal , binaryFloat bf ) {
  switch( isignal ){
    case   0: maxDutyCycle = bf.fp; break;
    case   1: BEMFa = bf.fp; break;
    case   2: BEMFb = bf.fp; break;
    case   3: Ialpha_last = bf.fp; break;
    case   4: Ibeta_last = bf.fp; break;
    case   5: commutationoffset = bf.fp; break;
    case   6: DQdisturbangle = bf.fp; break;
    case   7: Vq = bf.fp; break;
    case   8: Vd = bf.fp; break;
    case   9: Valpha = bf.fp; break;
    case  10: Vbeta = bf.fp; break;
    case  11: thetawave = bf.fp; break;
    case  12: Id_meas = bf.fp; break;
    case  13: Iq_meas = bf.fp; break;
    case  14: VSP = bf.fp; break;
    case  15: commutationoffset2 = bf.fp; break;
    case  16: DQdisturbangle2 = bf.fp; break;
    case  17: Vq2 = bf.fp; break;
    case  18: Vd2 = bf.fp; break;
    case  19: Valpha2 = bf.fp; break;
    case  20: Vbeta2 = bf.fp; break;
    case  21: thetawave2 = bf.fp; break;
    case  22: Id_meas2 = bf.fp; break;
    case  23: Iq_meas2 = bf.fp; break;
    case  24: Va = bf.fp; break;
    case  25: Vb = bf.fp; break;
    case  26: Vc = bf.fp; break;
    case  27: Va2 = bf.fp; break;
    case  28: Vb2 = bf.fp; break;
    case  29: Vc2 = bf.fp; break;
    case  36: mechcontout = bf.fp; break;
    case  37: Iout = bf.fp; break;
    case  38: mechcontout2 = bf.fp; break;
    case  39: Iout2 = bf.fp; break;
    case  40: muziek_gain = bf.fp; break;
    case  41: muziek_gain_V = bf.fp; break;
    case  42: distval = bf.fp; break;
    case  43: distoff = bf.fp; break;
    case  44: ss_phase = bf.fp; break;
    case  45: ss_fstart = bf.fp; break;
    case  46: ss_fstep = bf.fp; break;
    case  47: ss_fend = bf.fp; break;
    case  48: ss_gain = bf.fp; break;
    case  49: ss_offset = bf.fp; break;
    case  50: ss_f = bf.fp; break;
    case  51: ss_tstart = bf.fp; break;
    case  52: ss_out = bf.fp; break;
    case  56: I_max = bf.fp; break;
    case  57: V_Bus = bf.fp; break;
    case  58: rmech = bf.fp; break;
    case  59: rdelay = bf.fp; break;
    case  60: emech1 = bf.fp; break;
    case  61: ymech1 = bf.fp; break;
    case  62: rmech2 = bf.fp; break;
    case  63: emech2 = bf.fp; break;
    case  64: ymech2 = bf.fp; break;
    case  65: rmechoffset = bf.fp; break;
    case  66: rmechoffset2 = bf.fp; break;
    case  67: Kp = bf.fp; break;
    case  68: fBW = bf.fp; break;
    case  69: alpha1 = bf.fp; break;
    case  70: alpha2 = bf.fp; break;
    case  71: fInt = bf.fp; break;
    case  72: fLP = bf.fp; break;
    case  73: Kp2 = bf.fp; break;
    case  74: fBW2 = bf.fp; break;
    case  75: alpha1_2 = bf.fp; break;
    case  76: alpha2_2 = bf.fp; break;
    case  77: fInt2 = bf.fp; break;
    case  78: fLP2 = bf.fp; break;
    case  79: Vout = bf.fp; break;
    case  80: fIntCur = bf.fp; break;
    case  81: Icontgain = bf.fp; break;
    case  82: Vout2 = bf.fp; break;
    case  83: fIntCur2 = bf.fp; break;
    case  84: Icontgain2 = bf.fp; break;
    case  85: sensCalVal1 = bf.fp; break;
    case  86: sensCalVal2 = bf.fp; break;
    case  87: sensCalVal3 = bf.fp; break;
    case  88: sensCalVal4 = bf.fp; break;
    case  89: sens1 = bf.fp; break;
    case  90: sens2 = bf.fp; break;
    case  91: sens3 = bf.fp; break;
    case  92: sens4 = bf.fp; break;
    case  93: sens1_lp = bf.fp; break;
    case  94: sens2_lp = bf.fp; break;
    case  95: sens3_lp = bf.fp; break;
    case  96: sens4_lp = bf.fp; break;
    case  97: sens1_calib = bf.fp; break;
    case  98: sens2_calib = bf.fp; break;
    case  99: sens3_calib = bf.fp; break;
    case 100: sens4_calib = bf.fp; break;
    case 101: sensBus = bf.fp; break;
    case 102: Jload = bf.fp; break;
    case 103: velFF = bf.fp; break;
    case 104: R = bf.fp; break;
    case 105: Jload2 = bf.fp; break;
    case 106: velFF2 = bf.fp; break;
    case 107: offsetVelTot = bf.fp; break;
    case 108: offsetVel = bf.fp; break;
    case 109: offsetVel_lp = bf.fp; break;
    case 110: acc = bf.fp; break;
    case 111: vel = bf.fp; break;
    case 112: dist = bf.fp; break;
    case 113: Ialpha = bf.fp; break;
    case 114: Ibeta = bf.fp; break;
    case 115: thetaPark = bf.fp; break;
    case 116: thetaParkPrev = bf.fp; break;
    case 117: edeltarad = bf.fp; break;
    case 118: eradpers_lp = bf.fp; break;
    case 119: erpm = bf.fp; break;
    case 120: thetaPark_enc = bf.fp; break;
    case 121: thetaPark_obs = bf.fp; break;
    case 122: thetaPark_vesc = bf.fp; break;
    case 123: co = bf.fp; break;
    case 124: si = bf.fp; break;
    case 125: D = bf.fp; break;
    case 126: Q = bf.fp; break;
    case 127: tA = bf.fp; break;
    case 128: tB = bf.fp; break;
    case 129: tC = bf.fp; break;
    case 130: Id_e = bf.fp; break;
    case 131: Id_SP = bf.fp; break;
    case 132: Iq_e = bf.fp; break;
    case 133: Iq_SP = bf.fp; break;
    case 134: ia = bf.fp; break;
    case 135: ib = bf.fp; break;
    case 136: ic = bf.fp; break;
    case 137: acc2 = bf.fp; break;
    case 138: vel2 = bf.fp; break;
    case 139: Ialpha2 = bf.fp; break;
    case 140: Ibeta2 = bf.fp; break;
    case 141: thetaPark2 = bf.fp; break;
    case 142: co2 = bf.fp; break;
    case 143: si2 = bf.fp; break;
    case 144: D2 = bf.fp; break;
    case 145: Q2 = bf.fp; break;
    case 146: tA2 = bf.fp; break;
    case 147: tB2 = bf.fp; break;
    case 148: tC2 = bf.fp; break;
    case 149: Id_e2 = bf.fp; break;
    case 150: Id_SP2 = bf.fp; break;
    case 151: Iq_e2 = bf.fp; break;
    case 152: Iq_SP2 = bf.fp; break;
    case 153: ia2 = bf.fp; break;
    case 154: ib2 = bf.fp; break;
    case 155: ic2 = bf.fp; break;
    case 156: Vq_distgain = bf.fp; break;
    case 157: Vd_distgain = bf.fp; break;
    case 158: Iq_distgain = bf.fp; break;
    case 159: Id_distgain = bf.fp; break;
    case 160: mechdistgain = bf.fp; break;
    case 161: Kt_Nm_Arms = bf.fp; break;
    case 162: Kt_Nm_Apeak = bf.fp; break;
    case 163: we = bf.fp; break;
    case 164: Ld = bf.fp; break;
    case 165: Lq = bf.fp; break;
    case 166: Lambda_m = bf.fp; break;
    case 167: observer_gain = bf.fp; break;
    case 168: x1 = bf.fp; break;
    case 169: x2 = bf.fp; break;
    case 170: Kt_Nm_Arms2 = bf.fp; break;
    case 171: Kt_Nm_Apeak2 = bf.fp; break;
    case 172: we2 = bf.fp; break;
    case 173: Ld2 = bf.fp; break;
    case 174: Lq2 = bf.fp; break;
    case 175: Lambda_m2 = bf.fp; break;
    case 176: hfi_V = bf.fp; break;
    case 177: hfi_dir = bf.fp; break;
    case 178: Valpha_offset_hfi = bf.fp; break;
    case 179: Vbeta_offset_hfi = bf.fp; break;
    case 180: hfi_curtot = bf.fp; break;
    case 181: hfi_curorttot = bf.fp; break;
    case 182: VqFF = bf.fp; break;
    case 183: VdFF = bf.fp; break;
    case 184: VqFF2 = bf.fp; break;
    case 185: VdFF2 = bf.fp; break;
    case 186: Id_offset_SP = bf.fp; break;
    case 187: Id_offset_SP2 = bf.fp; break;
    case 188: Valpha_offset = bf.fp; break;
    case 189: Vbeta_offset = bf.fp; break;
    case 190: Valpha2_offset = bf.fp; break;
    case 192: anglechoice = bf.sint; break;
    case 193: timeremain = bf.sint; break;
    case 194: spNgo = bf.sint; break;
    case 195: REFstatus = bf.sint; break;
    case 196: incomingByte = bf.sint; break;
    case 197: encoderPos1 = bf.sint; break;
    case 198: encoderPos2 = bf.sint; break;
    case 201: SP_input_status = bf.sint; break;
    case 202: spGO = bf.sint; break;
    case 203: hfi_cursample = bf.sint; break;
    case 204: hfi_maxsamples = bf.sint; break;
    case 205: ridethewave = bf.uint; break;
    case 206: ridethewave2 = bf.uint; break;
    case 207: sendall = bf.uint; break;
    case 208: curloop = bf.uint; break;
    case 209: Ndownsample = bf.uint; break;
    case 210: downsample = bf.uint; break;
    case 211: Novervolt = bf.uint; break;
    case 212: Novervolt2 = bf.uint; break;
    case 213: NdownsamplePRBS = bf.uint; break;
    case 214: downsamplePRBS = bf.uint; break;
    case 215: ss_n_aver = bf.uint; break;
    case 216: Nsend = bf.uint; break;
    case 217: timePrev = bf.uint; break;
    case 218: curtime = bf.uint; break;
    case 219: overloadcount = bf.uint; break;
    case 220: useIlowpass = bf.uint; break;
    case 221: ContSelect = bf.uint; break;
    case 222: firsterror = bf.uint; break;
    case 223: N_pp = bf.uint; break;
    case 224: N_pp2 = bf.uint; break;
    case 225: SPdir = bf.bl; break;
    case 226: is_v7 = bf.bl; break;
    case 227: haptic = bf.bl; break;
    case 228: revercommutation1 = bf.bl; break;
    case 229: IndexFound1 = bf.bl; break;
    case 230: IndexFound2 = bf.bl; break;
    case 231: OutputOn = bf.bl; break;
    case 232: hfi_on = bf.bl; break;
  }
}

void printSignals( unsigned int selected ) {
  char *signalNames[] = { "maxDutyCycle", "BEMFa", "BEMFb", "Ialpha_last", "Ibeta_last", "commutationoffset", "DQdisturbangle", "Vq", "Vd", "Valpha", "Vbeta", "thetawave", "Id_meas", "Iq_meas", "VSP", "commutationoffset2", "DQdisturbangle2", "Vq2", "Vd2", "Valpha2", "Vbeta2", "thetawave2", "Id_meas2", "Iq_meas2", "Va", "Vb", "Vc", "Va2", "Vb2", "Vc2", "adc2A1", "adc2A2", "one_by_sqrt3", "two_by_sqrt3", "sqrt_two_three", "sqrt3_by_2", "mechcontout", "Iout", "mechcontout2", "Iout2", "muziek_gain", "muziek_gain_V", "distval", "distoff", "ss_phase", "ss_fstart", "ss_fstep", "ss_fend", "ss_gain", "ss_offset", "ss_f", "ss_tstart", "ss_out", "T", "enc2rad", "enc2rad2", "I_max", "V_Bus", "rmech", "rdelay", "emech1", "ymech1", "rmech2", "emech2", "ymech2", "rmechoffset", "rmechoffset2", "Kp", "fBW", "alpha1", "alpha2", "fInt", "fLP", "Kp2", "fBW2", "alpha1_2", "alpha2_2", "fInt2", "fLP2", "Vout", "fIntCur", "Icontgain", "Vout2", "fIntCur2", "Icontgain2", "sensCalVal1", "sensCalVal2", "sensCalVal3", "sensCalVal4", "sens1", "sens2", "sens3", "sens4", "sens1_lp", "sens2_lp", "sens3_lp", "sens4_lp", "sens1_calib", "sens2_calib", "sens3_calib", "sens4_calib", "sensBus", "Jload", "velFF", "R", "Jload2", "velFF2", "offsetVelTot", "offsetVel", "offsetVel_lp", "acc", "vel", "dist", "Ialpha", "Ibeta", "thetaPark", "thetaParkPrev", "edeltarad", "eradpers_lp", "erpm", "thetaPark_enc", "thetaPark_obs", "thetaPark_vesc", "co", "si", "D", "Q", "tA", "tB", "tC", "Id_e", "Id_SP", "Iq_e", "Iq_SP", "ia", "ib", "ic", "acc2", "vel2", "Ialpha2", "Ibeta2", "thetaPark2", "co2", "si2", "D2", "Q2", "tA2", "tB2", "tC2", "Id_e2", "Id_SP2", "Iq_e2", "Iq_SP2", "ia2", "ib2", "ic2", "Vq_distgain", "Vd_distgain", "Iq_distgain", "Id_distgain", "mechdistgain", "Kt_Nm_Arms", "Kt_Nm_Apeak", "we", "Ld", "Lq", "Lambda_m", "observer_gain", "x1", "x2", "Kt_Nm_Arms2", "Kt_Nm_Apeak2", "we2", "Ld2", "Lq2", "Lambda_m2", "hfi_V", "hfi_dir", "Valpha_offset_hfi", "Vbeta_offset_hfi", "hfi_curtot", "hfi_curorttot", "VqFF", "VdFF", "VqFF2", "VdFF2", "Id_offset_SP", "Id_offset_SP2", "Valpha_offset", "Vbeta_offset", "Valpha2_offset", "Ts", "anglechoice", "timeremain", "spNgo", "REFstatus", "incomingByte", "encoderPos1", "encoderPos2", "enccountperrev", "enccountperrev2", "SP_input_status", "spGO", "hfi_cursample", "hfi_maxsamples", "ridethewave", "ridethewave2", "sendall", "curloop", "Ndownsample", "downsample", "Novervolt", "Novervolt2", "NdownsamplePRBS", "downsamplePRBS", "ss_n_aver", "Nsend", "timePrev", "curtime", "overloadcount", "useIlowpass", "ContSelect", "firsterror", "N_pp", "N_pp2", "SPdir", "is_v7", "haptic", "revercommutation1", "IndexFound1", "IndexFound2", "OutputOn", "hfi_on",  };
  char *signalTypes[] = { "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "b", "b", "b", "b", "b", "b", "b", "b",  };
  int imax = 10;
  switch(selected){
    case 0: imax = 233; break;
  }
  for ( int i = 0; i < imax; i++) {
    Serial.println( signalNames[i] );
    Serial.println( signalTypes[i] );
  }
}
