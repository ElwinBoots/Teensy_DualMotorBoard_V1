typedef union {
  float fp;
  byte bin[4];
  unsigned int uint;
  int sint;
  bool bl;
} binaryFloat;

#include <Math.h>
#include <arm_math.h>
#include <SPI.h>

#include "Biquad.h"
#include "ControlTools.h"
#include "muziek.c"
#include "QuadEncoder.h"
#include "MotionProfile.h"
#include "defines.h"

void setup() {
  Serial.begin(1);
  pinMode( engate , OUTPUT);
  digitalWrite( engate , 1);
 
  SPI_init();  // Disable this for DRV8302
  xbar_init();
  adc_init();
  adc_etc_init();
  flexpwm2_init();
  flexpwm4_init();
  syncflexpwm();
  Encoders_init();

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
}

void xbar_init() {
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);   //turn clock on for xbara1
  xbar_connect(XBARA1_IN_FLEXPWM2_PWM1_OUT_TRIG0, XBARA1_OUT_ADC_ETC_TRIG00); //FlexPWM to adc_etc
}

void adc_init() {
  //Tried many configurations, but this seems to be best:
   ADC1_CFG =   ADC_CFG_OVWREN       //Allow overwriting of the next converted Data onto the existing
              | ADC_CFG_ADICLK(0)    // input clock select - IPG clock
              | ADC_CFG_MODE(2)      // 12-bit conversion 0 8-bit conversion 1 10-bit conversion 2  12-bit conversion
              | ADC_CFG_ADIV(2)      // Input clock / 4
              | ADC_CFG_ADSTS(0)     // Sample period (ADC clocks) = 3 if ADLSMP=0b
              | ADC_CFG_ADHSC        // High speed operation
              | ADC_CFG_ADTRG;       // Hardware trigger selected
   ADC2_CFG = ADC1_CFG;
 
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
  ADC_ETC_CTRL |= 1;  // TRIG_ENABLE

  /* ADC channel, pin numbers
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
  
  //FLEXPWM2_SM2VAL4 = FLEXPWM2_SM0VAL4; // adc trigger 1 to show on digital output 9
  //FLEXPWM2_SM2VAL5 = FLEXPWM2_SM0VAL5; // adc trigger 2 to show on digital output 9

  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK( 7 );// Load Okay LDOK(SM) -> reload setting again

  //This triggers twice per PWM cycle:
  FLEXPWM2_SM0TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1 << 4) | FLEXPWM_SMTCTRL_OUT_TRIG_EN(1 << 5); //  val 4 of Flexpwm sm0 as trigger; #define FLEXPWM_SMTCTRL_OUT_TRIG_EN(n)   ((uint16_t)(((n) & 0x3F) << 0))
  //This triggers once per PWM cycle:
  //FLEXPWM2_SM0TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1 << 5); //  val 4 of Flexpwm sm0 as trigger; #define FLEXPWM_SMTCTRL_OUT_TRIG_EN(n)   ((uint16_t)(((n) & 0x3F) << 0))

  *(portConfigRegister(4)) = 1; //Set port 4 to the right mux value (found in pwm.c)
  *(portConfigRegister(5)) = 1; //Set port 5 to the right mux value (found in pwm.c)
  *(portConfigRegister(6)) = 2; //Set port 6 to the right mux value (found in pwm.c)

  //FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMB_EN( 4 ); // Activate B channel
  //*(portConfigRegister(9)) = 2; //Set port 9 to the right mux value (found in pwm.c)
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

//////////////////////////////////////////////////////////////////////////////////
////// Start of the real time code, everything runs in the adc interrupts.  //////
//////////////////////////////////////////////////////////////////////////////////

void loop() {
}

void adcetc0_isr() {
  ADC_ETC_DONE0_1_IRQ &= 1;   // clear
}

void adcetc1_isr() {
  ADC_ETC_DONE0_1_IRQ &= 1 << 20;   // clear
  curtime = micros();
  is_v7 = (FLEXPWM2_SM0STS & FLEXPWM_SMSTS_CMPF(2));  //is_v7 = True when in v7
  FLEXPWM2_SM0STS |= FLEXPWM_SMSTS_CMPF(2); //Reset flag
  sens1 = (ADC_ETC_TRIG0_RESULT_1_0 & 4095) * 0.0008058608; // 4095.0 * 3.3;
  sens1_lp = lowpassIsens1->process( sens1 );
  sens3 = ((ADC_ETC_TRIG0_RESULT_1_0 >> 16) & 4095) * 0.0008058608; // 4095.0 * 3.3;
  sens3_lp = lowpassIsens3->process( sens3 );
  sensBus = (ADC_ETC_TRIG0_RESULT_3_2 & 4095) * Busadc2Vbus;   // 4095.0 * 3.3 * ((68.3+5.05)/5.05);
  sensBus_lp = lowpass_sensbus->process( sensBus );
  
  if (sensBus > V_Bus + 1 ) {
    //    digitalWrite( chopperpin , HIGH);
  }
  else {
    //    digitalWrite( chopperpin , LOW);
  }
  if (sensBus > 45 ) {
    OutputOn = false;
    if (firsterror == 0) {
      firsterror = 41;
    }
  }
  sens2 = (ADC_ETC_TRIG4_RESULT_1_0 & 4095) * 0.0008058608; // 4095.0 * 3.3;
  sens2_lp = lowpassIsens2->process( sens2 );
  sens4 = ((ADC_ETC_TRIG4_RESULT_1_0 >> 16) & 4095) * 0.0008058608; // 4095.0 * 3.3;
  sens4_lp = lowpassIsens4->process( sens4 );
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
  if ( ss_phase >= 2 * M_PI) {
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
  vel = SPprofile->vref + offsetVelTot;
  we = vel * N_pp;  //Electrical speed [rad/s], based on setpoint
  
  acc2 = -acc;
  vel2 = -vel;

  //When no setpoint is running, always convert reference to nearest encoder count to avoid noise
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
  if (hfi_useforfeedback == 1) {
    ymech1 = hfi_abs_pos / N_pp;
  }
  emech1 = rmech - ymech1;
  emech2 = rmech2 - ymech2;
  if (haptic == 1) {
    emech1 = rmech - ymech1 - ymech2;
    emech2 = 0;
  }

  if ((abs(emech1) > 0.5) & (Kp > 0) )
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

  // Clipping to be improved...
  Iout = integrator->processclip( mechcontout , -I_max * (1.5 * N_pp * Lambda_m ) - mechcontout , I_max * (1.5 * N_pp * Lambda_m ) - mechcontout );
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
    vq_int_state = 0;
    vd_int_state = 0;
    integrator_Id2->setState(0);
    integrator_Iq2->setState(0);
  }

  Iq_SP = mechcontout / (1.5 * N_pp * Lambda_m );

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

  // Park transform, ride the wave option
  thetaPark_enc = N_pp * (encoderPos1 % enccountperrev) * enc2rad + commutationoffset; //Modulo on the encoder counts to keep the floating point 0 to 2pi for numerical accuracy
  if (revercommutation1) {
    thetaPark_enc *= -1;
  }
  while ( thetaPark_enc >= 2 * M_PI) {
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
  thetaPark_obs = atan2(BEMFb, BEMFa);

  while ( thetaPark_obs >= 2 * M_PI) {
    thetaPark_obs -= 2 * M_PI;
  }
  while ( thetaPark_obs < 0) {
    thetaPark_obs += 2 * M_PI;
  }
  //Check and remove nan
  if (thetaPark_obs != thetaPark_obs) {
    thetaPark_obs = thetaPark_obs_prev;
  }
  thetaPark_obs_prev = thetaPark_obs;



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
  thetaPark_vesc = atan2( x2 - L_ib, x1 - L_ia);
  while ( thetaPark_vesc >= 2 * M_PI) {
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
  else if (anglechoice == 3 ) {
    thetaPark = hfi_dir;
  }
  else if (anglechoice == 99) {
    utils_step_towards((float*)&i_vector_radpers_act, i_vector_radpers, i_vector_acc * T );
    thetaPark += T * i_vector_radpers_act;
  }
  else {
    thetaPark = 0;
  }

  // Phase advance
  thetaPark += eradpers_lp * T * advancefactor;

  while ( thetaPark >= 2 * M_PI) {
    thetaPark -= 2 * M_PI;
  }
  while ( thetaPark < 0) {
    thetaPark += 2 * M_PI;
  }

  // erpm estimator
  edeltarad = thetaPark - thetaParkPrev;
  if (edeltarad > M_PI) {
    edeltarad -= 2 * M_PI;
  }
  if (edeltarad < -M_PI) {
    edeltarad += 2 * M_PI;
  }
  //Limit change of thetaPark to 45 deg per cycle:
  if (edeltarad > max_edeltarad) {
    edeltarad = max_edeltarad;
    thetaPark = thetaParkPrev + edeltarad;
  }
  else if (edeltarad < -max_edeltarad) {
    edeltarad = -max_edeltarad;
    thetaPark = thetaParkPrev + edeltarad;
  }
  eradpers_lp = lowpass_eradpers->process( edeltarad / T );
  erpm = eradpers_lp * 60 / 2 / M_PI;
  thetaParkPrev = thetaPark;
  hfi_abs_pos += edeltarad;

  //  thetaPark2 = 8 * (encoderPos2 % enccountperrev2) * enc2rad2 + commutationoffset2; //Modulo on the encoder counts to keep the floating point 0 to 2pi for numerical accuracy
  //  while ( thetaPark2 >= 2 * M_PI) {
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

  Iq_SP += muziek_gain * muziek[ (curloop / (50 / (int)Ts)) % (sizeof(muziek) / 4) ];
  Iq_SP += dist * Iq_distgain;

  Iq_SP += Iq_offset_SP;

  Id_SP = Id_offset_SP;
  Id_SP += dist * Id_distgain;

  Iq_SP2 += muziek_gain * muziek[ (curloop / (50 / (int)Ts)) % (sizeof(muziek) / 4) ];
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



  // HFI
  if ( hfi_on ) {
    hfi_V_act = hfi_V;
    if (hfi_firstcycle) {
      hfi_V_act /= 2;
      hfi_firstcycle = false;
    }
    if (is_v7) {
      hfi_Id_meas_high = Id_meas;
      hfi_Iq_meas_high = Iq_meas;
    }
    else {
      hfi_V_act = -hfi_V_act;
      hfi_Id_meas_low = Id_meas;
      hfi_Iq_meas_low = Iq_meas;
    }
    delta_id = hfi_Id_meas_high - hfi_Id_meas_low;
    delta_iq = hfi_Iq_meas_high - hfi_Iq_meas_low;
    //hfi_curangleest = 0.25f * atan2( -delta_iq  , delta_id - 0.5 * hfi_V * T * ( 1 / Ld + 1 / Lq ) ); //Complete calculation (not needed because error is always small due to feedback). 0.25 comes from 0.5 because delta signals are used and 0.5 due to 2theta (not just theta) being in the sin and cos wave.
    if(hfi_method ==1 || hfi_method ==3 ){
      hfi_curangleest =  0.5f * delta_iq / (hfi_V * T * ( 1 / Lq - 1 / Ld ) ); //0.5 because delta_iq is twice the iq value
    }
    else if(hfi_method ==2 || hfi_method == 4){
      if (is_v7) {
        hfi_curangleest =  (Iq_meas - Iq_SP) / (hfi_V * T * ( 1 / Lq - 1 / Ld ) );
      }
      else {
        hfi_curangleest =  (Iq_meas - Iq_SP) / (-hfi_V * T * ( 1 / Lq - 1 / Ld ) );
      }
    }
    hfi_error = -hfi_curangleest; //Negative feedback
    if (hfi_use_lowpass){
      hfi_error = hfi_lowpass->process( hfi_error );
    }
    hfi_dir_int += T * hfi_error * hfi_gain_int2; //This the the double integrator

    float hfi_half_int = hfi_gain * 0.5f * T * hfi_error;
    hfi_contout += hfi_half_int + hfi_half_int_prev + hfi_dir_int; //This is the integrator and the double integrator
    if(hfi_method ==3 || hfi_method == 4){
      hfi_ffw = we * T;
      hfi_contout += hfi_ffw; //This is the feedforward
    }
    while ( hfi_contout >= 2 * M_PI) {
      hfi_contout -= 2 * M_PI;
    }
    while ( hfi_contout < 0) {
      hfi_contout += 2 * M_PI;
    }
    while ( hfi_contout >= 2 * M_PI) {
      hfi_contout -= 2 * M_PI;
    }
    while ( hfi_contout < 0) {
      hfi_contout += 2 * M_PI;
    }
    
    hfi_dir = hfi_contout + dist * hfi_distgain;
    
    while ( hfi_dir >= 2 * M_PI) {
      hfi_dir -= 2 * M_PI;
    }
    while ( hfi_dir < 0) {
      hfi_dir += 2 * M_PI;
    }
    while ( hfi_dir_int >= 2 * M_PI) {
      hfi_dir_int -= 2 * M_PI;
    }
    while ( hfi_dir_int < 0) {
      hfi_dir_int += 2 * M_PI;
    }
    hfi_half_int_prev = hfi_half_int;
  }
  else {
    hfi_dir = thetaPark_obs;
    hfi_contout = thetaPark_obs;
    hfi_dir_int = 0;
    hfi_half_int_prev = 0;
    hfi_firstcycle = true;
    hfi_Id_meas_low = 0;
    hfi_Iq_meas_low = 0;
    hfi_Id_meas_high = 0;
    hfi_Iq_meas_high = 0;
    hfi_V_act = 0;
  }
  hfi_prev = hfi_V;


  if (useIlowpass == 2)
  {
    Id_meas = lowpassId1->process( Id_meas );
    Iq_meas = lowpassIq1->process( Iq_meas );
    Id_meas2 = lowpassId2->process( Id_meas2 );
    Iq_meas2 = lowpassIq2->process( Iq_meas2 );
  }

  if (ridethewave != 1 ) {
    if (OutputOn == true) {
      Id_e = Id_SP - Id_meas;
      Iq_e = Iq_SP - Iq_meas;
    }
    else {
      Id_e = 0;
      Iq_e = 0;
    }

    Vq = Kp_iq * Iq_e;
    float vq_half_int = Ki_iq * T * 0.5f * Vq;
    vq_int_state += vq_half_int;
    Vq += vq_int_state;
    vq_int_state += vq_half_int;

    //Additional Vq
    Vq += VSP;
    Vq += dist * Vq_distgain;

    Vd = Kp_id * Id_e;
    float vd_half_int = Ki_id * T * 0.5f * Vd;
    vd_int_state += vd_half_int;
    Vd += vd_int_state;
    vd_int_state += vd_half_int;

    //Additional Vd
    Vd += dist * Vd_distgain;
    Vd += hfi_V_act;

    // PMSM decoupling control and BEMF FF
    VqFF = we * ( Ld * Id_meas + Lambda_m);

    // q axis induction FFW based on setpoint FFW
    VqFF += SPprofile->jref * Jload * Lq * Kt_Nm_Apeak * OutputOn;

    Vq += VqFF;
    VdFF = -we * Lq * Iq_meas;
    Vd += VdFF;
  }

  Vq += muziek_gain_V * muziek[ (curloop / (50 / (int)Ts)) % (sizeof(muziek) / 4) ];

  // Voltage clipping
  maxVolt = maxDutyCycle * sensBus_lp * one_by_sqrt3;
  Vtot = NORM2_f( Vd , Vq );
  if ( Vtot > maxVolt) {
    if ( abs( Vd ) > maxVolt) {
      if (Vd > 0) {
        Vd = maxVolt;
      }
      else {
        Vd = -maxVolt;
      }
      if (abs(vd_int_state) > abs(Vd)) {
        vd_int_state = Vd;
      }
    }
    if (sq(Vd) >= sq(maxVolt)) { //Vd cannot be larger than maxvolt, so no issue with sqrt of negative values. Still get nan Vq somehow. Fix:
      Vq = 0;
    }
    else {
      if ( Vq > 0 ) {
        Vq = sqrt(sq(maxVolt) - sq(Vd)) ;
      }
      else {
        Vq = -sqrt(sq(maxVolt) - sq(Vd)) ;
      }
    }
    if (abs(vq_int_state) > abs(Vq)) {
      vq_int_state = Vq;
    }
  }

  // Inverse park transform
  Valpha = co * Vd - si * Vq;
  Vbeta  = co * Vq + si * Vd;

  Valpha += Valpha_offset + Valpha_offset_hfi;
  Vbeta += Vbeta_offset + Vbeta_offset_hfi;

  if (Valpha > maxVolt) {
    Valpha = maxVolt;
  }
  if (Vbeta > maxVolt) {
    Vbeta = maxVolt;
  }
  if (Valpha < -maxVolt) {
    Valpha = -maxVolt;
  }
  if (Vbeta < -maxVolt) {
    Vbeta = -maxVolt;
  }

  // Inverse Power-variant Clarke transform
  Va = Valpha;
  Vb = -0.5 * Valpha + sqrt3_by_2 * Vbeta;
  Vc = -0.5 * Valpha - sqrt3_by_2 * Vbeta;

  //See https://microchipdeveloper.com/mct5001:start Zero Sequence Modulation Tutorial
  float Vcm = -(max(max(Va, Vb), Vc) + min(min(Va, Vb), Vc)) / 2;
  Va += Vcm + sensBus_lp / 2;
  Vb += Vcm + sensBus_lp / 2;
  Vc += Vcm + sensBus_lp / 2;

  //Calculate modulation times
  tA = Va / sensBus_lp;
  tB = Vb / sensBus_lp;
  tC = Vc / sensBus_lp;

  //Motor 2 (needs to be updated, can probably be done much nicer):
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

  Vq2 += muziek_gain_V * muziek[ (curloop / (50 / (int)Ts)) % (sizeof(muziek) / 4) ];

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
  Va2 += Vcm2 + sensBus_lp / 2;
  Vb2 += Vcm2 + sensBus_lp / 2;
  Vc2 += Vcm2 + sensBus_lp / 2;

  //Calculate modulation times
  tA2 = Va2 / sensBus_lp;
  tB2 = Vb2 / sensBus_lp;
  tC2 = Vc2 / sensBus_lp;

}

void changePWM() {
  if (tA > 1) {
    tA = 1;
  }
  if (tB > 1) {
    tB = 1;
  }
  if (tC > 1) {
    tC = 1;
  }

  if (tA < 0) {
    tA = 0;
  }
  if (tB < 0) {
    tB = 0;
  }
  if (tC < 0) {
    tC = 0;
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
  if ((sendall > 0 ) & (Nsend == 0)) {
    if (sendall == 1) {
      for (int i = 0; i < n_trace; ++i)
      {
        tracearray[i] = i;
      }
    }
    if (sendall > 1) {
      for (int i = 0; i < n_trace; ++i)
      {
        tracearray[i] += n_trace;
      }
    }
    trace( );
    sendall++;
    if ((sendall - 1) * n_trace >= 268 ) {
      sendall = 0;
    }
  }
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
      digitalWrite( engate , 1);
      SPI_init();
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
      vq_int_state = 0;
      vd_int_state = 0;

      integrator2->setState(0);
      integrator_Id2->setState(0);
      integrator_Iq2->setState(0);

      firsterror = 0;
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
      
      integrator = new Integrator( fInt , 1 / T);
      leadlag       = new LeadLag( fBW , alpha1 , alpha2 , 1 / T);
      lowpass        = new Biquad( bq_type_lowpass , fLP , 0.7, 1 / T);
      integrator2 = new Integrator( fInt2 , 1 / T);
      leadlag2       = new LeadLag( fBW2 , alpha1_2 , alpha2_2 , 1 / T);
      lowpass2        = new Biquad( bq_type_lowpass , fLP2 , 0.7 , 1 / T);
    }
  }
}

void utils_step_towards(float * value, float goal, float step) {
  if (*value < goal) {
    if ((*value + step) < goal) {
      *value += step;
    } else {
      *value = goal;
    }
  } else if (*value > goal) {
    if ((*value - step) > goal) {
      *value -= step;
    } else {
      *value = goal;
    }
  }
}



void trace( ) {
  for( int i = 0; i < n_trace; i++){
    int isignal = tracearray[i];
    switch( isignal ){
      case   0: bf.fp   = Ts; break;
      case   1: bf.fp   = advancefactor; break;
      case   2: bf.fp   = i_vector_radpers; break;
      case   3: bf.fp   = i_vector_radpers_act; break;
      case   4: bf.fp   = i_vector_acc; break;
      case   5: bf.fp   = maxDutyCycle; break;
      case   6: bf.fp   = BEMFa; break;
      case   7: bf.fp   = BEMFb; break;
      case   8: bf.fp   = Ialpha_last; break;
      case   9: bf.fp   = Ibeta_last; break;
      case  10: bf.fp   = commutationoffset; break;
      case  11: bf.fp   = DQdisturbangle; break;
      case  12: bf.fp   = Vq; break;
      case  13: bf.fp   = Vd; break;
      case  14: bf.fp   = Valpha; break;
      case  15: bf.fp   = Vbeta; break;
      case  16: bf.fp   = thetawave; break;
      case  17: bf.fp   = Id_meas; break;
      case  18: bf.fp   = Iq_meas; break;
      case  19: bf.fp   = VSP; break;
      case  20: bf.fp   = commutationoffset2; break;
      case  21: bf.fp   = DQdisturbangle2; break;
      case  22: bf.fp   = Vq2; break;
      case  23: bf.fp   = Vd2; break;
      case  24: bf.fp   = Valpha2; break;
      case  25: bf.fp   = Vbeta2; break;
      case  26: bf.fp   = thetawave2; break;
      case  27: bf.fp   = Id_meas2; break;
      case  28: bf.fp   = Iq_meas2; break;
      case  29: bf.fp   = Va; break;
      case  30: bf.fp   = Vb; break;
      case  31: bf.fp   = Vc; break;
      case  32: bf.fp   = Va2; break;
      case  33: bf.fp   = Vb2; break;
      case  34: bf.fp   = Vc2; break;
      case  35: bf.fp   = adc2A1; break;
      case  36: bf.fp   = adc2A2; break;
      case  37: bf.fp   = one_by_sqrt3; break;
      case  38: bf.fp   = two_by_sqrt3; break;
      case  39: bf.fp   = sqrt_two_three; break;
      case  40: bf.fp   = sqrt3_by_2; break;
      case  41: bf.fp   = mechcontout; break;
      case  42: bf.fp   = Iout; break;
      case  43: bf.fp   = mechcontout2; break;
      case  44: bf.fp   = Iout2; break;
      case  45: bf.fp   = muziek_gain; break;
      case  46: bf.fp   = muziek_gain_V; break;
      case  47: bf.fp   = distval; break;
      case  48: bf.fp   = distoff; break;
      case  49: bf.fp   = ss_phase; break;
      case  50: bf.fp   = ss_fstart; break;
      case  51: bf.fp   = ss_fstep; break;
      case  52: bf.fp   = ss_fend; break;
      case  53: bf.fp   = ss_gain; break;
      case  54: bf.fp   = ss_offset; break;
      case  55: bf.fp   = ss_f; break;
      case  56: bf.fp   = ss_tstart; break;
      case  57: bf.fp   = ss_out; break;
      case  58: bf.fp   = T; break;
      case  59: bf.fp   = enc2rad; break;
      case  60: bf.fp   = enc2rad2; break;
      case  61: bf.fp   = I_max; break;
      case  62: bf.fp   = V_Bus; break;
      case  63: bf.fp   = rmech; break;
      case  64: bf.fp   = rdelay; break;
      case  65: bf.fp   = emech1; break;
      case  66: bf.fp   = ymech1; break;
      case  67: bf.fp   = rmech2; break;
      case  68: bf.fp   = emech2; break;
      case  69: bf.fp   = ymech2; break;
      case  70: bf.fp   = rmechoffset; break;
      case  71: bf.fp   = rmechoffset2; break;
      case  72: bf.fp   = sensBus_lp; break;
      case  73: bf.fp   = Kp; break;
      case  74: bf.fp   = fBW; break;
      case  75: bf.fp   = alpha1; break;
      case  76: bf.fp   = alpha2; break;
      case  77: bf.fp   = fInt; break;
      case  78: bf.fp   = fLP; break;
      case  79: bf.fp   = Kp2; break;
      case  80: bf.fp   = fBW2; break;
      case  81: bf.fp   = alpha1_2; break;
      case  82: bf.fp   = alpha2_2; break;
      case  83: bf.fp   = fInt2; break;
      case  84: bf.fp   = fLP2; break;
      case  85: bf.fp   = Vout; break;
      case  86: bf.fp   = fIntCur; break;
      case  87: bf.fp   = Kp_iq; break;
      case  88: bf.fp   = Kp_id; break;
      case  89: bf.fp   = Ki_iq; break;
      case  90: bf.fp   = Ki_id; break;
      case  91: bf.fp   = vq_int_state; break;
      case  92: bf.fp   = vd_int_state; break;
      case  93: bf.fp   = Vout2; break;
      case  94: bf.fp   = fIntCur2; break;
      case  95: bf.fp   = Icontgain2; break;
      case  96: bf.fp   = sensCalVal1; break;
      case  97: bf.fp   = sensCalVal2; break;
      case  98: bf.fp   = sensCalVal3; break;
      case  99: bf.fp   = sensCalVal4; break;
      case 100: bf.fp   = sens1; break;
      case 101: bf.fp   = sens2; break;
      case 102: bf.fp   = sens3; break;
      case 103: bf.fp   = sens4; break;
      case 104: bf.fp   = sens1_lp; break;
      case 105: bf.fp   = sens2_lp; break;
      case 106: bf.fp   = sens3_lp; break;
      case 107: bf.fp   = sens4_lp; break;
      case 108: bf.fp   = sens1_calib; break;
      case 109: bf.fp   = sens2_calib; break;
      case 110: bf.fp   = sens3_calib; break;
      case 111: bf.fp   = sens4_calib; break;
      case 112: bf.fp   = sensBus; break;
      case 113: bf.fp   = Busadc2Vbus; break;
      case 114: bf.fp   = Jload; break;
      case 115: bf.fp   = velFF; break;
      case 116: bf.fp   = R; break;
      case 117: bf.fp   = Jload2; break;
      case 118: bf.fp   = velFF2; break;
      case 119: bf.fp   = offsetVelTot; break;
      case 120: bf.fp   = offsetVel; break;
      case 121: bf.fp   = offsetVel_lp; break;
      case 122: bf.fp   = acc; break;
      case 123: bf.fp   = vel; break;
      case 124: bf.fp   = dist; break;
      case 125: bf.fp   = Ialpha; break;
      case 126: bf.fp   = Ibeta; break;
      case 127: bf.fp   = thetaPark; break;
      case 128: bf.fp   = thetaParkPrev; break;
      case 129: bf.fp   = edeltarad; break;
      case 130: bf.fp   = eradpers_lp; break;
      case 131: bf.fp   = erpm; break;
      case 132: bf.fp   = thetaPark_enc; break;
      case 133: bf.fp   = thetaPark_obs; break;
      case 134: bf.fp   = thetaPark_obs_prev; break;
      case 135: bf.fp   = thetaPark_vesc; break;
      case 136: bf.fp   = co; break;
      case 137: bf.fp   = si; break;
      case 138: bf.fp   = D; break;
      case 139: bf.fp   = Q; break;
      case 140: bf.fp   = tA; break;
      case 141: bf.fp   = tB; break;
      case 142: bf.fp   = tC; break;
      case 143: bf.fp   = Id_e; break;
      case 144: bf.fp   = Id_SP; break;
      case 145: bf.fp   = Iq_e; break;
      case 146: bf.fp   = Iq_SP; break;
      case 147: bf.fp   = ia; break;
      case 148: bf.fp   = ib; break;
      case 149: bf.fp   = ic; break;
      case 150: bf.fp   = acc2; break;
      case 151: bf.fp   = vel2; break;
      case 152: bf.fp   = Ialpha2; break;
      case 153: bf.fp   = Ibeta2; break;
      case 154: bf.fp   = thetaPark2; break;
      case 155: bf.fp   = co2; break;
      case 156: bf.fp   = si2; break;
      case 157: bf.fp   = D2; break;
      case 158: bf.fp   = Q2; break;
      case 159: bf.fp   = tA2; break;
      case 160: bf.fp   = tB2; break;
      case 161: bf.fp   = tC2; break;
      case 162: bf.fp   = Id_e2; break;
      case 163: bf.fp   = Id_SP2; break;
      case 164: bf.fp   = Iq_e2; break;
      case 165: bf.fp   = Iq_SP2; break;
      case 166: bf.fp   = ia2; break;
      case 167: bf.fp   = ib2; break;
      case 168: bf.fp   = ic2; break;
      case 169: bf.fp   = Vq_distgain; break;
      case 170: bf.fp   = Vd_distgain; break;
      case 171: bf.fp   = Iq_distgain; break;
      case 172: bf.fp   = Id_distgain; break;
      case 173: bf.fp   = mechdistgain; break;
      case 174: bf.fp   = maxVolt; break;
      case 175: bf.fp   = Vtot; break;
      case 176: bf.fp   = max_edeltarad; break;
      case 177: bf.fp   = N_pp; break;
      case 178: bf.fp   = Kt_Nm_Arms; break;
      case 179: bf.fp   = Kt_Nm_Apeak; break;
      case 180: bf.fp   = we; break;
      case 181: bf.fp   = Ld; break;
      case 182: bf.fp   = Lq; break;
      case 183: bf.fp   = Lambda_m; break;
      case 184: bf.fp   = observer_gain; break;
      case 185: bf.fp   = x1; break;
      case 186: bf.fp   = x2; break;
      case 187: bf.fp   = Kt_Nm_Arms2; break;
      case 188: bf.fp   = Kt_Nm_Apeak2; break;
      case 189: bf.fp   = we2; break;
      case 190: bf.fp   = Ld2; break;
      case 191: bf.fp   = Lq2; break;
      case 192: bf.fp   = Lambda_m2; break;
      case 193: bf.fp   = hfi_V; break;
      case 194: bf.fp   = hfi_V_act; break;
      case 195: bf.fp   = hfi_dir; break;
      case 196: bf.fp   = hfi_dir_int; break;
      case 197: bf.fp   = Valpha_offset_hfi; break;
      case 198: bf.fp   = Vbeta_offset_hfi; break;
      case 199: bf.fp   = hfi_curtot; break;
      case 200: bf.fp   = hfi_curorttot; break;
      case 201: bf.fp   = hfi_curprev; break;
      case 202: bf.fp   = hfi_curortprev; break;
      case 203: bf.fp   = hfi_gain; break;
      case 204: bf.fp   = hfi_pgain; break;
      case 205: bf.fp   = hfi_curangleest; break;
      case 206: bf.fp   = hfi_dir_int2; break;
      case 207: bf.fp   = hfi_gain_int2; break;
      case 208: bf.fp   = hfi_Id_meas_low; break;
      case 209: bf.fp   = hfi_Iq_meas_low; break;
      case 210: bf.fp   = hfi_Id_meas_high; break;
      case 211: bf.fp   = hfi_Iq_meas_high; break;
      case 212: bf.fp   = delta_id; break;
      case 213: bf.fp   = delta_iq; break;
      case 214: bf.fp   = hfi_advance_factor; break;
      case 215: bf.fp   = hfi_abs_pos; break;
      case 216: bf.fp   = hfi_half_int_prev; break;
      case 217: bf.fp   = VqFF; break;
      case 218: bf.fp   = VdFF; break;
      case 219: bf.fp   = VqFF2; break;
      case 220: bf.fp   = VdFF2; break;
      case 221: bf.fp   = Iq_offset_SP; break;
      case 222: bf.fp   = Id_offset_SP; break;
      case 223: bf.fp   = Id_offset_SP2; break;
      case 224: bf.fp   = Valpha_offset; break;
      case 225: bf.fp   = Vbeta_offset; break;
      case 226: bf.fp   = Valpha2_offset; break;
      case 227: bf.sint = anglechoice; break;
      case 228: bf.sint = timeremain; break;
      case 229: bf.sint = spNgo; break;
      case 230: bf.sint = REFstatus; break;
      case 231: bf.sint = incomingByte; break;
      case 232: bf.sint = encoderPos1; break;
      case 233: bf.sint = encoderPos2; break;
      case 234: bf.sint = enccountperrev; break;
      case 235: bf.sint = enccountperrev2; break;
      case 236: bf.sint = n_senscalib; break;
      case 237: bf.sint = SP_input_status; break;
      case 238: bf.sint = spGO; break;
      case 239: bf.sint = hfi_cursample; break;
      case 240: bf.sint = hfi_maxsamples; break;
      case 241: bf.uint = ridethewave; break;
      case 242: bf.uint = ridethewave2; break;
      case 243: bf.uint = sendall; break;
      case 244: bf.uint = curloop; break;
      case 245: bf.uint = Ndownsample; break;
      case 246: bf.uint = downsample; break;
      case 247: bf.uint = Novervolt; break;
      case 248: bf.uint = Novervolt2; break;
      case 249: bf.uint = NdownsamplePRBS; break;
      case 250: bf.uint = downsamplePRBS; break;
      case 251: bf.uint = ss_n_aver; break;
      case 252: bf.uint = IndexFound1; break;
      case 253: bf.uint = IndexFound2; break;
      case 254: bf.uint = Nsend; break;
      case 255: bf.uint = timePrev; break;
      case 256: bf.uint = curtime; break;
      case 257: bf.uint = overloadcount; break;
      case 258: bf.uint = useIlowpass; break;
      case 259: bf.uint = ContSelect; break;
      case 260: bf.uint = firsterror; break;
      case 261: bf.uint = N_pp2; break;
      case 262: bf.bl   = SPdir; break;
      case 263: bf.bl   = is_v7; break;
      case 264: bf.bl   = haptic; break;
      case 265: bf.bl   = revercommutation1; break;
      case 266: bf.bl   = OutputOn; break;
      case 267: bf.bl   = setupready; break;
      case 268: bf.bl   = hfi_on; break;
      case 269: bf.bl   = hfi_firstcycle; break;
      case 270: bf.bl   = hfi_useforfeedback; break;
      case 271: bf.bl   = hfi_use_lowpass; break;
    }
    Serial.write( bf.bin , 4);
  }
}

void setpar( int isignal , binaryFloat bf ) {
  switch( isignal ){
    case   1: advancefactor = bf.fp; break;
    case   2: i_vector_radpers = bf.fp; break;
    case   3: i_vector_radpers_act = bf.fp; break;
    case   4: i_vector_acc = bf.fp; break;
    case   5: maxDutyCycle = bf.fp; break;
    case   6: BEMFa = bf.fp; break;
    case   7: BEMFb = bf.fp; break;
    case   8: Ialpha_last = bf.fp; break;
    case   9: Ibeta_last = bf.fp; break;
    case  10: commutationoffset = bf.fp; break;
    case  11: DQdisturbangle = bf.fp; break;
    case  12: Vq = bf.fp; break;
    case  13: Vd = bf.fp; break;
    case  14: Valpha = bf.fp; break;
    case  15: Vbeta = bf.fp; break;
    case  16: thetawave = bf.fp; break;
    case  17: Id_meas = bf.fp; break;
    case  18: Iq_meas = bf.fp; break;
    case  19: VSP = bf.fp; break;
    case  20: commutationoffset2 = bf.fp; break;
    case  21: DQdisturbangle2 = bf.fp; break;
    case  22: Vq2 = bf.fp; break;
    case  23: Vd2 = bf.fp; break;
    case  24: Valpha2 = bf.fp; break;
    case  25: Vbeta2 = bf.fp; break;
    case  26: thetawave2 = bf.fp; break;
    case  27: Id_meas2 = bf.fp; break;
    case  28: Iq_meas2 = bf.fp; break;
    case  29: Va = bf.fp; break;
    case  30: Vb = bf.fp; break;
    case  31: Vc = bf.fp; break;
    case  32: Va2 = bf.fp; break;
    case  33: Vb2 = bf.fp; break;
    case  34: Vc2 = bf.fp; break;
    case  41: mechcontout = bf.fp; break;
    case  42: Iout = bf.fp; break;
    case  43: mechcontout2 = bf.fp; break;
    case  44: Iout2 = bf.fp; break;
    case  45: muziek_gain = bf.fp; break;
    case  46: muziek_gain_V = bf.fp; break;
    case  47: distval = bf.fp; break;
    case  48: distoff = bf.fp; break;
    case  49: ss_phase = bf.fp; break;
    case  50: ss_fstart = bf.fp; break;
    case  51: ss_fstep = bf.fp; break;
    case  52: ss_fend = bf.fp; break;
    case  53: ss_gain = bf.fp; break;
    case  54: ss_offset = bf.fp; break;
    case  55: ss_f = bf.fp; break;
    case  56: ss_tstart = bf.fp; break;
    case  57: ss_out = bf.fp; break;
    case  61: I_max = bf.fp; break;
    case  62: V_Bus = bf.fp; break;
    case  63: rmech = bf.fp; break;
    case  64: rdelay = bf.fp; break;
    case  65: emech1 = bf.fp; break;
    case  66: ymech1 = bf.fp; break;
    case  67: rmech2 = bf.fp; break;
    case  68: emech2 = bf.fp; break;
    case  69: ymech2 = bf.fp; break;
    case  70: rmechoffset = bf.fp; break;
    case  71: rmechoffset2 = bf.fp; break;
    case  72: sensBus_lp = bf.fp; break;
    case  73: Kp = bf.fp; break;
    case  74: fBW = bf.fp; break;
    case  75: alpha1 = bf.fp; break;
    case  76: alpha2 = bf.fp; break;
    case  77: fInt = bf.fp; break;
    case  78: fLP = bf.fp; break;
    case  79: Kp2 = bf.fp; break;
    case  80: fBW2 = bf.fp; break;
    case  81: alpha1_2 = bf.fp; break;
    case  82: alpha2_2 = bf.fp; break;
    case  83: fInt2 = bf.fp; break;
    case  84: fLP2 = bf.fp; break;
    case  85: Vout = bf.fp; break;
    case  86: fIntCur = bf.fp; break;
    case  87: Kp_iq = bf.fp; break;
    case  88: Kp_id = bf.fp; break;
    case  89: Ki_iq = bf.fp; break;
    case  90: Ki_id = bf.fp; break;
    case  91: vq_int_state = bf.fp; break;
    case  92: vd_int_state = bf.fp; break;
    case  93: Vout2 = bf.fp; break;
    case  94: fIntCur2 = bf.fp; break;
    case  95: Icontgain2 = bf.fp; break;
    case  96: sensCalVal1 = bf.fp; break;
    case  97: sensCalVal2 = bf.fp; break;
    case  98: sensCalVal3 = bf.fp; break;
    case  99: sensCalVal4 = bf.fp; break;
    case 100: sens1 = bf.fp; break;
    case 101: sens2 = bf.fp; break;
    case 102: sens3 = bf.fp; break;
    case 103: sens4 = bf.fp; break;
    case 104: sens1_lp = bf.fp; break;
    case 105: sens2_lp = bf.fp; break;
    case 106: sens3_lp = bf.fp; break;
    case 107: sens4_lp = bf.fp; break;
    case 108: sens1_calib = bf.fp; break;
    case 109: sens2_calib = bf.fp; break;
    case 110: sens3_calib = bf.fp; break;
    case 111: sens4_calib = bf.fp; break;
    case 112: sensBus = bf.fp; break;
    case 113: Busadc2Vbus = bf.fp; break;
    case 114: Jload = bf.fp; break;
    case 115: velFF = bf.fp; break;
    case 116: R = bf.fp; break;
    case 117: Jload2 = bf.fp; break;
    case 118: velFF2 = bf.fp; break;
    case 119: offsetVelTot = bf.fp; break;
    case 120: offsetVel = bf.fp; break;
    case 121: offsetVel_lp = bf.fp; break;
    case 122: acc = bf.fp; break;
    case 123: vel = bf.fp; break;
    case 124: dist = bf.fp; break;
    case 125: Ialpha = bf.fp; break;
    case 126: Ibeta = bf.fp; break;
    case 127: thetaPark = bf.fp; break;
    case 128: thetaParkPrev = bf.fp; break;
    case 129: edeltarad = bf.fp; break;
    case 130: eradpers_lp = bf.fp; break;
    case 131: erpm = bf.fp; break;
    case 132: thetaPark_enc = bf.fp; break;
    case 133: thetaPark_obs = bf.fp; break;
    case 134: thetaPark_obs_prev = bf.fp; break;
    case 135: thetaPark_vesc = bf.fp; break;
    case 136: co = bf.fp; break;
    case 137: si = bf.fp; break;
    case 138: D = bf.fp; break;
    case 139: Q = bf.fp; break;
    case 140: tA = bf.fp; break;
    case 141: tB = bf.fp; break;
    case 142: tC = bf.fp; break;
    case 143: Id_e = bf.fp; break;
    case 144: Id_SP = bf.fp; break;
    case 145: Iq_e = bf.fp; break;
    case 146: Iq_SP = bf.fp; break;
    case 147: ia = bf.fp; break;
    case 148: ib = bf.fp; break;
    case 149: ic = bf.fp; break;
    case 150: acc2 = bf.fp; break;
    case 151: vel2 = bf.fp; break;
    case 152: Ialpha2 = bf.fp; break;
    case 153: Ibeta2 = bf.fp; break;
    case 154: thetaPark2 = bf.fp; break;
    case 155: co2 = bf.fp; break;
    case 156: si2 = bf.fp; break;
    case 157: D2 = bf.fp; break;
    case 158: Q2 = bf.fp; break;
    case 159: tA2 = bf.fp; break;
    case 160: tB2 = bf.fp; break;
    case 161: tC2 = bf.fp; break;
    case 162: Id_e2 = bf.fp; break;
    case 163: Id_SP2 = bf.fp; break;
    case 164: Iq_e2 = bf.fp; break;
    case 165: Iq_SP2 = bf.fp; break;
    case 166: ia2 = bf.fp; break;
    case 167: ib2 = bf.fp; break;
    case 168: ic2 = bf.fp; break;
    case 169: Vq_distgain = bf.fp; break;
    case 170: Vd_distgain = bf.fp; break;
    case 171: Iq_distgain = bf.fp; break;
    case 172: Id_distgain = bf.fp; break;
    case 173: mechdistgain = bf.fp; break;
    case 174: maxVolt = bf.fp; break;
    case 175: Vtot = bf.fp; break;
    case 176: max_edeltarad = bf.fp; break;
    case 177: N_pp = bf.fp; break;
    case 178: Kt_Nm_Arms = bf.fp; break;
    case 179: Kt_Nm_Apeak = bf.fp; break;
    case 180: we = bf.fp; break;
    case 181: Ld = bf.fp; break;
    case 182: Lq = bf.fp; break;
    case 183: Lambda_m = bf.fp; break;
    case 184: observer_gain = bf.fp; break;
    case 185: x1 = bf.fp; break;
    case 186: x2 = bf.fp; break;
    case 187: Kt_Nm_Arms2 = bf.fp; break;
    case 188: Kt_Nm_Apeak2 = bf.fp; break;
    case 189: we2 = bf.fp; break;
    case 190: Ld2 = bf.fp; break;
    case 191: Lq2 = bf.fp; break;
    case 192: Lambda_m2 = bf.fp; break;
    case 193: hfi_V = bf.fp; break;
    case 194: hfi_V_act = bf.fp; break;
    case 195: hfi_dir = bf.fp; break;
    case 196: hfi_dir_int = bf.fp; break;
    case 197: Valpha_offset_hfi = bf.fp; break;
    case 198: Vbeta_offset_hfi = bf.fp; break;
    case 199: hfi_curtot = bf.fp; break;
    case 200: hfi_curorttot = bf.fp; break;
    case 201: hfi_curprev = bf.fp; break;
    case 202: hfi_curortprev = bf.fp; break;
    case 203: hfi_gain = bf.fp; break;
    case 204: hfi_pgain = bf.fp; break;
    case 205: hfi_curangleest = bf.fp; break;
    case 206: hfi_dir_int2 = bf.fp; break;
    case 207: hfi_gain_int2 = bf.fp; break;
    case 208: hfi_Id_meas_low = bf.fp; break;
    case 209: hfi_Iq_meas_low = bf.fp; break;
    case 210: hfi_Id_meas_high = bf.fp; break;
    case 211: hfi_Iq_meas_high = bf.fp; break;
    case 212: delta_id = bf.fp; break;
    case 213: delta_iq = bf.fp; break;
    case 214: hfi_advance_factor = bf.fp; break;
    case 215: hfi_abs_pos = bf.fp; break;
    case 216: hfi_half_int_prev = bf.fp; break;
    case 217: VqFF = bf.fp; break;
    case 218: VdFF = bf.fp; break;
    case 219: VqFF2 = bf.fp; break;
    case 220: VdFF2 = bf.fp; break;
    case 221: Iq_offset_SP = bf.fp; break;
    case 222: Id_offset_SP = bf.fp; break;
    case 223: Id_offset_SP2 = bf.fp; break;
    case 224: Valpha_offset = bf.fp; break;
    case 225: Vbeta_offset = bf.fp; break;
    case 226: Valpha2_offset = bf.fp; break;
    case 227: anglechoice = bf.sint; break;
    case 228: timeremain = bf.sint; break;
    case 229: spNgo = bf.sint; break;
    case 230: REFstatus = bf.sint; break;
    case 231: incomingByte = bf.sint; break;
    case 232: encoderPos1 = bf.sint; break;
    case 233: encoderPos2 = bf.sint; break;
    case 236: n_senscalib = bf.sint; break;
    case 237: SP_input_status = bf.sint; break;
    case 238: spGO = bf.sint; break;
    case 239: hfi_cursample = bf.sint; break;
    case 240: hfi_maxsamples = bf.sint; break;
    case 241: ridethewave = bf.uint; break;
    case 242: ridethewave2 = bf.uint; break;
    case 243: sendall = bf.uint; break;
    case 244: curloop = bf.uint; break;
    case 245: Ndownsample = bf.uint; break;
    case 246: downsample = bf.uint; break;
    case 247: Novervolt = bf.uint; break;
    case 248: Novervolt2 = bf.uint; break;
    case 249: NdownsamplePRBS = bf.uint; break;
    case 250: downsamplePRBS = bf.uint; break;
    case 251: ss_n_aver = bf.uint; break;
    case 252: IndexFound1 = bf.uint; break;
    case 253: IndexFound2 = bf.uint; break;
    case 254: Nsend = bf.uint; break;
    case 255: timePrev = bf.uint; break;
    case 256: curtime = bf.uint; break;
    case 257: overloadcount = bf.uint; break;
    case 258: useIlowpass = bf.uint; break;
    case 259: ContSelect = bf.uint; break;
    case 260: firsterror = bf.uint; break;
    case 261: N_pp2 = bf.uint; break;
    case 262: SPdir = bf.bl; break;
    case 263: is_v7 = bf.bl; break;
    case 264: haptic = bf.bl; break;
    case 265: revercommutation1 = bf.bl; break;
    case 266: OutputOn = bf.bl; break;
    case 267: setupready = bf.bl; break;
    case 268: hfi_on = bf.bl; break;
    case 269: hfi_firstcycle = bf.bl; break;
    case 270: hfi_useforfeedback = bf.bl; break;
    case 271: hfi_use_lowpass = bf.bl; break;
  }
}

void printSignals( unsigned int selected ) {
  const char *signalNames[] = { "Ts", "advancefactor", "i_vector_radpers", "i_vector_radpers_act", "i_vector_acc", "maxDutyCycle", "BEMFa", "BEMFb", "Ialpha_last", "Ibeta_last", "commutationoffset", "DQdisturbangle", "Vq", "Vd", "Valpha", "Vbeta", "thetawave", "Id_meas", "Iq_meas", "VSP", "commutationoffset2", "DQdisturbangle2", "Vq2", "Vd2", "Valpha2", "Vbeta2", "thetawave2", "Id_meas2", "Iq_meas2", "Va", "Vb", "Vc", "Va2", "Vb2", "Vc2", "adc2A1", "adc2A2", "one_by_sqrt3", "two_by_sqrt3", "sqrt_two_three", "sqrt3_by_2", "mechcontout", "Iout", "mechcontout2", "Iout2", "muziek_gain", "muziek_gain_V", "distval", "distoff", "ss_phase", "ss_fstart", "ss_fstep", "ss_fend", "ss_gain", "ss_offset", "ss_f", "ss_tstart", "ss_out", "T", "enc2rad", "enc2rad2", "I_max", "V_Bus", "rmech", "rdelay", "emech1", "ymech1", "rmech2", "emech2", "ymech2", "rmechoffset", "rmechoffset2", "sensBus_lp", "Kp", "fBW", "alpha1", "alpha2", "fInt", "fLP", "Kp2", "fBW2", "alpha1_2", "alpha2_2", "fInt2", "fLP2", "Vout", "fIntCur", "Kp_iq", "Kp_id", "Ki_iq", "Ki_id", "vq_int_state", "vd_int_state", "Vout2", "fIntCur2", "Icontgain2", "sensCalVal1", "sensCalVal2", "sensCalVal3", "sensCalVal4", "sens1", "sens2", "sens3", "sens4", "sens1_lp", "sens2_lp", "sens3_lp", "sens4_lp", "sens1_calib", "sens2_calib", "sens3_calib", "sens4_calib", "sensBus", "Busadc2Vbus", "Jload", "velFF", "R", "Jload2", "velFF2", "offsetVelTot", "offsetVel", "offsetVel_lp", "acc", "vel", "dist", "Ialpha", "Ibeta", "thetaPark", "thetaParkPrev", "edeltarad", "eradpers_lp", "erpm", "thetaPark_enc", "thetaPark_obs", "thetaPark_obs_prev", "thetaPark_vesc", "co", "si", "D", "Q", "tA", "tB", "tC", "Id_e", "Id_SP", "Iq_e", "Iq_SP", "ia", "ib", "ic", "acc2", "vel2", "Ialpha2", "Ibeta2", "thetaPark2", "co2", "si2", "D2", "Q2", "tA2", "tB2", "tC2", "Id_e2", "Id_SP2", "Iq_e2", "Iq_SP2", "ia2", "ib2", "ic2", "Vq_distgain", "Vd_distgain", "Iq_distgain", "Id_distgain", "mechdistgain", "maxVolt", "Vtot", "max_edeltarad", "N_pp", "Kt_Nm_Arms", "Kt_Nm_Apeak", "we", "Ld", "Lq", "Lambda_m", "observer_gain", "x1", "x2", "Kt_Nm_Arms2", "Kt_Nm_Apeak2", "we2", "Ld2", "Lq2", "Lambda_m2", "hfi_V", "hfi_V_act", "hfi_dir", "hfi_dir_int", "Valpha_offset_hfi", "Vbeta_offset_hfi", "hfi_curtot", "hfi_curorttot", "hfi_curprev", "hfi_curortprev", "hfi_gain", "hfi_pgain", "hfi_curangleest", "hfi_dir_int2", "hfi_gain_int2", "hfi_Id_meas_low", "hfi_Iq_meas_low", "hfi_Id_meas_high", "hfi_Iq_meas_high", "delta_id", "delta_iq", "hfi_advance_factor", "hfi_abs_pos", "hfi_half_int_prev", "VqFF", "VdFF", "VqFF2", "VdFF2", "Iq_offset_SP", "Id_offset_SP", "Id_offset_SP2", "Valpha_offset", "Vbeta_offset", "Valpha2_offset", "anglechoice", "timeremain", "spNgo", "REFstatus", "incomingByte", "encoderPos1", "encoderPos2", "enccountperrev", "enccountperrev2", "n_senscalib", "SP_input_status", "spGO", "hfi_cursample", "hfi_maxsamples", "ridethewave", "ridethewave2", "sendall", "curloop", "Ndownsample", "downsample", "Novervolt", "Novervolt2", "NdownsamplePRBS", "downsamplePRBS", "ss_n_aver", "IndexFound1", "IndexFound2", "Nsend", "timePrev", "curtime", "overloadcount", "useIlowpass", "ContSelect", "firsterror", "N_pp2", "SPdir", "is_v7", "haptic", "revercommutation1", "OutputOn", "setupready", "hfi_on", "hfi_firstcycle", "hfi_useforfeedback", "hfi_use_lowpass",  };
  const char *signalTypes[] = { "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "b", "b", "b", "b", "b", "b", "b", "b", "b", "b",  };
  int imax = 10;
  switch(selected){
    case 0: imax = 272; break;
  }
  for ( int i = 0; i < imax; i++) {
    Serial.println( signalNames[i] );
    Serial.println( signalTypes[i] );
  }
}
