#include <Math.h>
#include <arm_math.h>
#include <SPI.h>

#include "Biquad.h"

//#include "ControlTools.h"
#include "muziek.c"
#include "QuadEncoder.h"
#include "MotionProfile.h"
#include "defines.h"
#include "trace.h"

Biquad *lowpass        = new Biquad( bq_type_lowpass , 50 , 0.7, 2 * F_PWM);
Biquad *lowpass_eradpers   = new Biquad( bq_type_lowpass , 50 , 0.7, 2 * F_PWM);
//Biquad *notch          = new Biquad( bq_type_notch , 2315.0, -20.0, 0.1 , 2 * F_PWM );

Biquad *Biquads1[6];
Biquad *Biquads2[6];

//Current lowpass (now used at sensor level, maybe better at id,iq level?). Doesn't seem to matter much.
Biquad *lowpassIsens1  = new Biquad( bq_type_lowpass , 10e3 , 0.7, 2 * F_PWM);
Biquad *lowpassIsens2  = new Biquad( bq_type_lowpass , 10e3 , 0.7, 2 * F_PWM);
Biquad *lowpassIsens3  = new Biquad( bq_type_lowpass , 10e3 , 0.7, 2 * F_PWM);
Biquad *lowpassIsens4  = new Biquad( bq_type_lowpass , 10e3 , 0.7, 2 * F_PWM);

//These are now used for power and bus current estimates
Biquad *lowpassId1  = new Biquad( bq_type_lowpass , 50 , 0.7, 2 * F_PWM);
Biquad *lowpassIq1  = new Biquad( bq_type_lowpass , 50 , 0.7, 2 * F_PWM);

Biquad *hfi_lowpass = new Biquad( bq_type_lowpass , 2000 , 0.707, 2 * F_PWM);

// For setpoint
Biquad *lowpassSP = new Biquad( bq_type_lowpass , 10 , 0.707, 2 * F_PWM);

//fast 180 deg:
MotionProfile *SPprofile1 = new MotionProfile( 0 , 0.000500000000000000 , 0.0193000000000000 , 0 , 3.14159265358979 , 157.079632679490 , 7853.98163397448 , 15632147.3532855 , 1 / (2 * F_PWM) );
MotionProfile *SPprofile2 = new MotionProfile( 0 , 0.000500000000000000 , 0.0193000000000000 , 0 , 3.14159265358979 , 157.079632679490 , 7853.98163397448 , 15632147.3532855 , 1 / (2 * F_PWM) );

Biquad *lowpass_ss_offset = new Biquad( bq_type_lowpass , 10 , 0.707, 2 * F_PWM);

//There are 4 hardware quadrature encoder channels available the Teensy 4.x.
//The Teensy 4.1 Encoders are supported on pins: 0, 1, 2, 3, 4, 5, 7, 30, 31, 33, 36 and 37.
//WARNING! Pins 0, 5 and 37 share the same internal crossbar connections and are as such exclusive...pick one or the other.
QuadEncoder Encoder1(1, 0, 1 , 0 , 3);   //Encoder 1 on pins 0 and 1, index on pin 3
QuadEncoder Encoder2(2, 30, 31 , 0 , 33);//Encoder 2 on pins 30 and 31, index on pin 33

void initparams( motor_total_t* m ) {
  m->conf.Ts = 1e6 / (2 * F_PWM);
  m->conf.T = m->conf.Ts / 1e6;
  m->conf.Busadc2Vbus = 1 / 4095.0 * 3.3 * ((68.3 + 5.05) / 5.05); //5.1 changed to 5.05 to improve accuracy. May differ board to board.
  m->conf.V_Bus = 24; //Bus Voltage
  m->conf.Ndownsample = 1;

  m->state1.OutputOn = true;
  m->state2.OutputOn = true;

  initmotor( &m->conf1 , &m->state1);
  initmotor( &m->conf2 , &m->state2);

  m->state.lfsr = 0xACE1u;
  m->state.ss_f = 1;
}

void initmotor( mot_conf_t* m , mot_state_t* state ) {
  m->maxDutyCycle = 0.99;
  m->adc2A = 1 / 0.09; //With linear hal current sensors ACS711 (31A or 15.5A): These allow for 2 measurements per PWM cycle

  m->enccountperrev = 20000;
  m->enc2rad = 2 * M_PI / m->enccountperrev;
  m->I_max = 15; //Max current (peak of the sine wave in a phase)
  m->max_edeltarad = 0.25f * M_PI;

  m->maxerror = 0.5;

  m->T_max = 1;

  //Motor parameters
  m->N_pp = 4; //Number of pole pairs
  m->Ld = 10e-3; //[Henry] Ld induction: phase-zero
  m->Lq = 10e-3; //[Henry] Lq induction: phase-zero
  m->Lambda_m = 0.01; //[Weber] Note: on the fly changes of Kt do not adjust this value!
  m->Kt_Nm_Apeak = 1.5 * m->N_pp * m->Lambda_m ;
  m->Kp = 0;
  m->fBW = 50.0;
  m->alpha1 = 3.0;
  m->alpha2 = 4.0;
  m->fInt = m->fBW / 6.0;
  m->fLP = m->fBW * 6.0;
}

void setup() {
  initparams( &motor );

  Serial.begin(1);

  for (int i = 0; i < 6; i++)
  {
    Biquads1[i] = new Biquad( bq_type_lowpass , 0 , 0.7, 2 * F_PWM);;
    Biquads2[i] = new Biquad( bq_type_lowpass , 0 , 0.7, 2 * F_PWM);;
  }

  pinMode( ENGATE , OUTPUT);
  digitalWrite( ENGATE , 1); // To be updated!
  SPI_init( SSPIN );  // Only for DRV8301.

  pinMode( ENGATE2 , OUTPUT);
  digitalWrite( ENGATE2 , 1); // To be updated!
  SPI_init( SSPIN2 );  // Only for DRV8301.

  //DRV8302_init( SSPIN2 , 13 ); // Note: pin 13 is also the SCLK pin for communication with DRV8301 and the LED.
  xbar_init();
  adc_init();
  adc_etc_init();
  flexpwm2_init();
  flexpwm4_init();
  syncflexpwm();
  Encoders_init();

  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMA_EN( 7 ); // Activate all A channels
  FLEXPWM4_OUTEN |= FLEXPWM_OUTEN_PWMA_EN( 7 ); // Activate all A channels

  delay(100); //Allow the lowpass filters in current measurement to settle before calibration
  motor.state.setupready = 1;
}

void SPI_init( int SSpin ) {
  SPI.begin();
  SPI.beginTransaction(SPISettings( 10e6 , MSBFIRST, SPI_MODE1)); //DRV8301 specsheet: 100 ns -> 10 Mhz. Set correct mode (fallingedgeof the clock).
  pinMode(SSpin, OUTPUT);
  int receivedVal16;
  while (receivedVal16 != 0x1038) {
    //while (receivedVal16 != 0x1008) {
    digitalWrite(SSpin, LOW);
    receivedVal16 = SPI.transfer16( (0 << 15) | (0x02 << 11) | (1 << 5) | (1 << 4) | (1 << 3) ); //Set 3 PWM inputs mode, disable OC mode
    //receivedVal16 = SPI.transfer16( (0 << 15) | (0x02 << 11)  | (1 << 3) ); //Set 3 PWM inputs mode
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

void DRV8302_init( int M_PWM_pin , int OC_ADJ_pin ) {
  pinMode( M_PWM_pin , OUTPUT);
  digitalWrite( M_PWM_pin , 1);
  pinMode( OC_ADJ_pin , OUTPUT);
  digitalWrite( OC_ADJ_pin , 1);
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

  CORE_PIN14_PADCONFIG &= ~ (1 << 12) ; // disable keeper pin for analog input, as per manual
  CORE_PIN15_PADCONFIG &= ~ (1 << 12) ; // disable keeper pin for analog input, as per manual
  CORE_PIN16_PADCONFIG &= ~ (1 << 12) ; // disable keeper pin for analog input, as per manual
  CORE_PIN17_PADCONFIG &= ~ (1 << 12) ; // disable keeper pin for analog input, as per manual
  CORE_PIN18_PADCONFIG &= ~ (1 << 12) ; // disable keeper pin for analog input, as per manual
  CORE_PIN19_PADCONFIG &= ~ (1 << 12) ; // disable keeper pin for analog input, as per manual
  CORE_PIN20_PADCONFIG &= ~ (1 << 12) ; // disable keeper pin for analog input, as per manual
  CORE_PIN21_PADCONFIG &= ~ (1 << 12) ; // disable keeper pin for analog input, as per manual

  CORE_PIN24_PADCONFIG &= ~ (1 << 12) ; // disable keeper pin for analog input, as per manual
  CORE_PIN25_PADCONFIG &= ~ (1 << 12) ; // disable keeper pin for analog input, as per manual
  CORE_PIN26_PADCONFIG &= ~ (1 << 12) ; // disable keeper pin for analog input, as per manual

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

  ADC_ETC_TRIG0_CTRL = ADC_ETC_TRIG_CTRL_TRIG_CHAIN(2); //TRIG chain length (0->1, 1->2, etc)

  ADC_ETC_TRIG0_CHAIN_1_0 =
    ADC_ETC_TRIG_CHAIN_IE1(0) |
    ADC_ETC_TRIG_CHAIN_B2B1 |
    ADC_ETC_TRIG_CHAIN_HWTS1(1) |
    ADC_ETC_TRIG_CHAIN_CSEL1(1) |
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
    ADC_ETC_TRIG_CHAIN_CSEL1(3) |
    ADC_ETC_TRIG_CHAIN_IE0(0) |
    ADC_ETC_TRIG_CHAIN_B2B0 |
    ADC_ETC_TRIG_CHAIN_HWTS0(1) |
    ADC_ETC_TRIG_CHAIN_CSEL0(8);

  ADC_ETC_TRIG4_CHAIN_3_2 =
    ADC_ETC_TRIG_CHAIN_IE0(2) |
    ADC_ETC_TRIG_CHAIN_B2B0 |
    ADC_ETC_TRIG_CHAIN_HWTS0(1) |
    ADC_ETC_TRIG_CHAIN_CSEL0(0);

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
  FLEXPWM2_SM0VAL1 = (uint32_t)((float)F_BUS_ACTUAL / F_PWM - 1) / 2; //Set the modulus value (dictates the frequency)
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
  FLEXPWM4_SM0VAL1 = (uint32_t)((float)F_BUS_ACTUAL / F_PWM - 1) / 2; //Set the modulus value (dictates the frequency)
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
  motor.state.curtime = micros();
  motor.state.is_v7 = (FLEXPWM2_SM0STS & FLEXPWM_SMSTS_CMPF(2));  //is_v7 = True when in v7
  FLEXPWM2_SM0STS |= FLEXPWM_SMSTS_CMPF(2); //Reset flag

  //ADC1:
  motor.state.sens1 = (ADC_ETC_TRIG0_RESULT_1_0 & 4095) * 0.0008058608; // 4095.0 * 3.3;
  motor.state.sens1_lp = lowpassIsens1->process( motor.state.sens1 );
  motor.state.sens3 = ((ADC_ETC_TRIG0_RESULT_1_0 >> 16) & 4095) * 0.0008058608; // 4095.0 * 3.3;
  motor.state.sens3_lp = lowpassIsens3->process( motor.state.sens3 );
  motor.state.sensBus = (ADC_ETC_TRIG0_RESULT_3_2 & 4095) * motor.conf.Busadc2Vbus;   // 4095.0 * 3.3 * ((68.3+5.05)/5.05);
  LOWPASS( motor.state.sensBus_lp , motor.state.sensBus, 0.1 ); //1000 Hz when running at 60 kHz

  //ADC2:
  motor.state.sens2 = (ADC_ETC_TRIG4_RESULT_1_0 & 4095) * 0.0008058608; // 4095.0 * 3.3;
  motor.state.sens2_lp = lowpassIsens2->process( motor.state.sens2 );
  motor.state.sens4 = ((ADC_ETC_TRIG4_RESULT_1_0 >> 16) & 4095) * 0.0008058608; // 4095.0 * 3.3;
  motor.state.sens4_lp = lowpassIsens4->process( motor.state.sens4 );
  motor.state.sensBus2 = (ADC_ETC_TRIG4_RESULT_3_2 & 4095) * motor.conf.Busadc2Vbus;   // 4095.0 * 3.3 * ((68.3+5.05)/5.05);

  // Calculate currents (links sensors to axes, to be improved)
  if (motor.conf1.useIlowpass == 1)
  {
    motor.state1.ia = motor.conf1.adc2A * (motor.state.sens1_lp - motor.state.sens1_calib);
    motor.state1.ib = motor.conf1.adc2A * (motor.state.sens2_lp - motor.state.sens2_calib);
  }
  else {
    motor.state1.ia = motor.conf1.adc2A * (motor.state.sens1 - motor.state.sens1_calib);
    motor.state1.ib = motor.conf1.adc2A * (motor.state.sens2 - motor.state.sens2_calib);
  }
  motor.state1.ic = -motor.state1.ia - motor.state1.ib;
  if (motor.conf2.useIlowpass == 1)
  {
    motor.state2.ia = motor.conf2.adc2A * (motor.state.sens3_lp - motor.state.sens3_calib);
    motor.state2.ib = motor.conf2.adc2A * (motor.state.sens4_lp - motor.state.sens4_calib);
  }
  else {
    motor.state2.ia = motor.conf2.adc2A * (motor.state.sens3 - motor.state.sens3_calib);
    motor.state2.ib = motor.conf2.adc2A * (motor.state.sens4 - motor.state.sens4_calib);
  }
  motor.state2.ic = -motor.state2.ia - motor.state2.ib;

  //
  //  if ( motor.state.sensBus > motor.conf.V_Bus + 1 ) {
  //    //    digitalWrite( CHOPPERPIN , HIGH);
  //  }
  //  else {
  //    //    digitalWrite( CHOPPERPIN , LOW);
  //  }
  if ( motor.state.sensBus > 45 or motor.state.sensBus2 > 45 ) {
    error(41 , &motor.state1);
  }
  if ( motor.state.sensBus2 > 45  ) {
    error(41 , &motor.state2);
  }

  if (motor.state.setupready == 1) {
    if (motor.state.n_senscalib < 1e4) {
      // Have to check this. Calibration not always ok.
      motor.state.n_senscalib++;
      motor.state.sens1_calib += motor.state.sens1_lp;
      motor.state.sens2_calib += motor.state.sens2_lp;
      motor.state.sens3_calib += motor.state.sens3_lp;
      motor.state.sens4_calib += motor.state.sens4_lp;
    }
    else if (motor.state.n_senscalib == 1e4) {
      motor.state.sens1_calib /= motor.state.n_senscalib;
      motor.state.sens2_calib /= motor.state.n_senscalib;
      motor.state.sens3_calib /= motor.state.n_senscalib;
      motor.state.sens4_calib /= motor.state.n_senscalib;
      motor.state.n_senscalib++;
    }
    else {
      updateDisturbance(); //Switched this before readENC to have a fresher encoder position
      GenSetpoint( &motor.conf1 , &motor.state1 , SPprofile1 );
      GenSetpoint( &motor.conf2 , &motor.state2 , SPprofile2 );
      readENC();
      Control( &motor.conf1 , &motor.state1 , Biquads1);
      Control( &motor.conf2 , &motor.state2 , Biquads2);
      Transforms( &motor.conf1 , &motor.state1 , Biquads1);
      Transforms( &motor.conf2 , &motor.state2 , Biquads2);

      current_and_duty_limts( &motor.conf1 , &motor.state1 );
      current_and_duty_limts( &motor.conf2 , &motor.state2 );
      changePWM();
      communicationProcess();
      processCommands( &motor.conf1 , &motor.state1);
      processCommands( &motor.conf2 , &motor.state2);
      motor.state.curloop++;
    }
  }
}


void updateDisturbance() {
  //PRBS
  motor.state.downsamplePRBS++;
  if ( motor.state.downsamplePRBS > motor.conf.NdownsamplePRBS) {
    motor.state.downsamplePRBS = 1;
    motor.state.noisebit  = ((motor.state.lfsr >> 5) ^ (motor.state.lfsr >> 7) ) & 1;  // taps: 11 9; feedback polynomial: x^11 + x^9 + 1
    motor.state.lfsr =  (motor.state.lfsr >> 1) | (motor.state.noisebit << 15);
  }

  //Single Sine
  if ( motor.state.curtime / 1e6 >= (motor.state.ss_tstart + motor.conf.ss_n_aver / motor.state.ss_f )) {
    motor.state.ss_f += motor.conf.ss_fstep;
    motor.state.ss_tstart = motor.state.curtime / 1e6;
    if (motor.state.ss_f > motor.conf.ss_fend)
    {
      motor.state.ss_f = 0;
      motor.state.ss_phase = 0;
      motor.state.ss_tstart = 1e8;
      motor.conf1.ss_gain = 0;
      motor.conf1.ss_offset = 0;
      motor.conf2.ss_gain = 0;
      motor.conf2.ss_offset = 0;
    }
  }
  motor.state.ss_phase += motor.state.ss_f * 2 * M_PI * motor.conf.T;
  if ( motor.state.ss_phase >= 2 * M_PI) {
    motor.state.ss_phase -= 2 * M_PI; //Required, because high value floats are inaccurate
  }
  float ss_offset_lp1 = lowpass_ss_offset->process( motor.conf1.ss_offset );
  float ss_offset_lp2 = lowpass_ss_offset->process( motor.conf2.ss_offset );
  //motor.state.ss_out = ss_offset_lp + motor.conf1.ss_gain * arm_sin_f32( motor.state.ss_phase ); //sin() measured to be faster then sinf(); arm_sin_f32() is way faster!
  float sin_phase = sin( motor.state.ss_phase );
  motor.state1.ss_out = ss_offset_lp1 + motor.conf1.ss_gain * sin_phase;
  motor.state2.ss_out = ss_offset_lp2 + motor.conf2.ss_gain * sin_phase;
  motor.state1.dist = motor.state1.distval * 1 * (motor.state.noisebit - 0.5) + motor.state1.distoff + motor.state1.ss_out;
  motor.state2.dist = motor.state2.distval * 1 * (motor.state.noisebit - 0.5) + motor.state2.distoff + motor.state2.ss_out;
}


void GenSetpoint( mot_conf_t* confX , mot_state_t* stateX , MotionProfile* SPprofileX ) {
  SPprofileX->REFidir = stateX->SPdir;
  SPprofileX->rdelay = stateX->rdelay;

  if (SPprofileX->REFstatus != 1) { // if not running SP
    SPprofileX->REFstatus = 0; //Make sure sp generator is ready for sp generation
    if (stateX->spNgo > 0) {
      stateX->spGO = 1;
      stateX->spNgo -= 1;
    }
    else {
      stateX->spGO = 0;
    }
  }
  stateX->REFstatus = SPprofileX->REFstatus;
  stateX->rmech = SPprofileX->stateCalculation( stateX->spGO );

  //  stateX->offsetVel_lp = lowpassSP->process( stateX->offsetVel );

  utils_step_towards( &stateX->offsetVel_lp , stateX->offsetVel, 20.0 * motor.conf.T );

  stateX->offsetVelTot += stateX->offsetVel_lp * motor.conf.T;
  stateX->rmech += stateX->offsetVelTot ;

  stateX->rmech += stateX->rmechoffset;

  stateX->acc = SPprofileX->aref;
  stateX->vel = SPprofileX->vref + stateX->offsetVel_lp;
  stateX->we = stateX->vel * confX->N_pp;  //Electrical speed [rad/s], based on setpoint

  //When no setpoint is running, always convert reference to nearest encoder count to avoid noise
  if (SPprofileX->REFstatus == 0 && stateX->offsetVel_lp == 0) {
    stateX->rmech = int((stateX->rmech / confX->enc2rad)) * confX->enc2rad;
  }

  stateX->rmech += stateX->dist * stateX->rdistgain;
}

void readENC() {
  // Linking between encoders and axes to be improved
  motor.state.encoderPos1 = Encoder1.read();
  motor.state.encoderPos2 = Encoder2.read();
  motor.state.IndexFound1 = Encoder1.indexfound();
  motor.state.IndexFound2 = Encoder2.indexfound();

  motor.state1.encoderPos1 = motor.state.encoderPos1;
  motor.state1.IndexFound1 = motor.state.IndexFound1;

  motor.state2.encoderPos1 = motor.state.encoderPos2;
  motor.state2.IndexFound1 = motor.state.IndexFound2;
}

void Control( mot_conf_t* confX , mot_state_t* stateX , Biquad **BiquadsX) {
  stateX->ymech = stateX->encoderPos1 * confX->enc2rad * confX->enc_transmission;

  if (stateX->hfi_useforfeedback == 1) {
    stateX->ymech = stateX->hfi_abs_pos / confX->N_pp;
  }
  stateX->emech = stateX->rmech - stateX->ymech;

  if ((abs(stateX->emech) > confX->maxerror) & (confX->Kp > 0) )
  {
    error(1 , stateX);
  }
  if (stateX->OutputOn == false) {
    stateX->emech = 0;
    stateX->Ki_sum = 0;
  }

  if (confX->Kp == 0) {
    stateX->Ki_sum = 0;
  }

  stateX->Kp_out = confX->Kp * stateX->emech;

  stateX->Kd_out = confX->Kd * (stateX->Kp_out - stateX->Kp_out_prev) + stateX->Kp_out;
  stateX->Kp_out_prev = stateX->Kp_out;

  stateX->Ki_sum += confX->Ki * stateX->Kd_out;
  truncate_number_abs(&stateX->Ki_sum, confX->T_max);

  stateX->Ki_out = stateX->Ki_sum + stateX->Kd_out;

  LOWPASS( stateX->lp_out, stateX->Ki_out, confX->lowpass_c);

  stateX->biquadout = stateX->lp_out;

  for ( int i = 0; i < 4; i++) {
    stateX->biquadout = BiquadsX[i]->process( stateX->biquadout );
  }

  stateX->a1 = BiquadsX[0]->a1;
  stateX->a2 = BiquadsX[0]->a2;
  stateX->b0 = BiquadsX[0]->b0;
  stateX->b1 = BiquadsX[0]->b1;
  stateX->b2 = BiquadsX[0]->b2;
  stateX->f0 = BiquadsX[0]->f0;
  stateX->damp = BiquadsX[0]->damp;
  stateX->fs = BiquadsX[0]->fs;
  stateX->z1 = BiquadsX[0]->z1;
  stateX->z2 = BiquadsX[0]->z2;

  stateX->mechcontout = stateX->biquadout + stateX->dist * stateX->mechdistgain;

  if (stateX->OutputOn) {
    stateX->mechcontout += stateX->acc * stateX->Jload;
    stateX->mechcontout += stateX->vel * stateX->velFF;
    stateX->vq_int_state = 0;
    stateX->vd_int_state = 0;
  }

  stateX->Iq_SP = stateX->mechcontout / (1.5 * confX->N_pp * confX->Lambda_m );
}

void Transforms( mot_conf_t* confX , mot_state_t* stateX , Biquad **BiquadsX)
{
  // For Park and Clarke see https://www.cypress.com/file/222111/download Power-variant Clarke transform. Asuming ia+ib+ic=0:
  stateX->Ialpha = stateX->ia;
  stateX->Ibeta = ONE_BY_SQRT3 * stateX->ia + TWO_BY_SQRT3 * stateX->ib;

  // Park transform, ride the wave option
  stateX->thetaPark_enc = confX->N_pp * (stateX->encoderPos1 % confX->enccountperrev) * confX->enc2rad + confX->commutationoffset; //Modulo on the encoder counts to keep the floating point 0 to 2pi for numerical accuracy
  if (confX->reversecommutation) {
    stateX->thetaPark_enc *= -1;
  }
  while ( stateX->thetaPark_enc >= 2 * M_PI) {
    stateX->thetaPark_enc -= 2 * M_PI;
  }
  while ( stateX->thetaPark_enc < 0) {
    stateX->thetaPark_enc += 2 * M_PI;
  }

  //Angle observer by mxlemming
  float L = (confX->Ld + confX->Lq) / 2;
  stateX->BEMFa = stateX->BEMFa + (stateX->Valpha - stateX->R * stateX->Ialpha) * motor.conf.T -
                  L * (stateX->Ialpha - stateX->Ialpha_last);
  stateX->BEMFb = stateX->BEMFb + (stateX->Vbeta - stateX->R * stateX->Ibeta) * motor.conf.T -
                  L * (stateX->Ibeta - stateX->Ibeta_last)  ;
  stateX->Ialpha_last = stateX->Ialpha;
  stateX->Ibeta_last = stateX->Ibeta;
  if (stateX->BEMFa > confX->Lambda_m  ) {
    stateX->BEMFa = confX->Lambda_m ;
  }
  if (stateX->BEMFa < -confX->Lambda_m ) {
    stateX->BEMFa = -confX->Lambda_m ;
  }
  if (stateX->BEMFb > confX->Lambda_m ) {
    stateX->BEMFb = confX->Lambda_m ;
  }
  if (stateX->BEMFb < -confX->Lambda_m ) {
    stateX->BEMFb = -confX->Lambda_m ;
  }
  stateX->thetaPark_obs = atan2(stateX->BEMFb, stateX->BEMFa);

  while ( stateX->thetaPark_obs >= 2 * M_PI) {
    stateX->thetaPark_obs -= 2 * M_PI;
  }
  while ( stateX->thetaPark_obs < 0) {
    stateX->thetaPark_obs += 2 * M_PI;
  }
  //Check and remove nan
  if (stateX->thetaPark_obs != stateX->thetaPark_obs) {
    stateX->thetaPark_obs = stateX->thetaPark_obs_prev;
  }
  stateX->thetaPark_obs_prev = stateX->thetaPark_obs;


  if (confX->anglechoice == 0) {
    stateX->thetaPark = stateX->thetaPark_enc;
  }
  else if (confX->anglechoice == 1) {
    stateX->thetaPark = stateX->thetaPark_obs;
  }
  else if (confX->anglechoice == 3 ) {
    if ( abs(stateX->vel) < stateX->hfi_maxvel ) {
      stateX->hfi_on = true;
      stateX->thetaPark = stateX->hfi_dir;
    }
    else {
      stateX->hfi_on = false;
      stateX->thetaPark = stateX->thetaPark_obs;
    }
  }
  else if (confX->anglechoice == 99) {
    utils_step_towards((float*)&stateX->i_vector_radpers_act, stateX->i_vector_radpers, stateX->i_vector_acc * motor.conf.T );
    stateX->thetaPark += motor.conf.T * stateX->i_vector_radpers_act;
  }
  else if (confX->anglechoice == 100) {
    //Empty such that thethaPark can be set from host.
  }
  else {
    stateX->thetaPark = 0;
  }

  // Phase advance
  stateX->thetaPark += stateX->eradpers_lp * motor.conf.T * confX->advancefactor;

  while ( stateX->thetaPark >= 2 * M_PI) {
    stateX->thetaPark -= 2 * M_PI;
  }
  while ( stateX->thetaPark < 0) {
    stateX->thetaPark += 2 * M_PI;
  }

  // erpm estimator
  stateX->edeltarad = stateX->thetaPark - stateX->thetaParkPrev;
  if (stateX->edeltarad > M_PI) {
    stateX->edeltarad -= 2 * M_PI;
  }
  if (stateX->edeltarad < -M_PI) {
    stateX->edeltarad += 2 * M_PI;
  }
  //Limit change of stateX->thetaPark to 45 deg per cycle:
  if (stateX->edeltarad > confX->max_edeltarad) {
    stateX->edeltarad = confX->max_edeltarad;
    stateX->thetaPark = stateX->thetaParkPrev + stateX->edeltarad;
  }
  else if (stateX->edeltarad < -confX->max_edeltarad) {
    stateX->edeltarad = -confX->max_edeltarad;
    stateX->thetaPark = stateX->thetaParkPrev + stateX->edeltarad;
  }
  stateX->eradpers_lp = lowpass_eradpers->process( stateX->edeltarad / motor.conf.T );
  stateX->erpm = stateX->eradpers_lp * 60 / (2 * M_PI);
  stateX->thetaParkPrev = stateX->thetaPark;
  stateX->hfi_abs_pos += stateX->edeltarad;

  if (confX->ridethewave == 1 ) {
    if ((stateX->IndexFound1) < 1 ) {
      stateX->thetaPark = stateX->thetawave;
      stateX->thetawave -= 10 * 2 * M_PI * motor.conf.T;
      stateX->Vq = 1.5;
    }
    else {
      stateX->Vq = 0;
      confX->ridethewave = 2;
      stateX->thetawave = 0;
    }
  }

  stateX->Iq_SP += stateX->muziek_gain * muziek[ (motor.state.curloop / (50 / (int)motor.conf.Ts)) % (sizeof(muziek) / 4) ];
  stateX->Iq_SP += stateX->dist * stateX->Iq_distgain;

  stateX->Iq_SP += stateX->Iq_offset_SP;

  stateX->Id_SP = stateX->Id_offset_SP;
  stateX->Id_SP += stateX->dist * stateX->Id_distgain;

  stateX->co = cos(stateX->thetaPark);
  stateX->si = sin(stateX->thetaPark);


  // Park transform
  stateX->Id_meas = stateX->co * stateX->Ialpha + stateX->si * stateX->Ibeta;
  stateX->Iq_meas = stateX->co * stateX->Ibeta  - stateX->si * stateX->Ialpha;

  //  stateX->Id_meas_lp = lowpassId1->process( stateX->Id_meas );
  //  stateX->Iq_meas_lp = lowpassIq1->process( stateX->Iq_meas );

  LOWPASS( stateX->Id_meas_lp, stateX->Id_meas, 0.005 ); //50 Hz when running at 60 kHz
  LOWPASS( stateX->Iq_meas_lp, stateX->Iq_meas, 0.005 );

  stateX->P_tot = 1.5 * ( stateX->Vq * stateX->Iq_meas_lp + stateX->Vd * stateX->Id_meas_lp);
  stateX->I_bus = stateX->P_tot / motor.state.sensBus_lp;

  // HFI
  if ( stateX->hfi_on ) {
    stateX->hfi_V_act = stateX->hfi_V;
    if (stateX->hfi_firstcycle) {
      stateX->hfi_V_act /= 2;
      stateX->hfi_firstcycle = false;
    }
    if (stateX->hfi_V != stateX->hfi_prev) {
      stateX->hfi_V_act = stateX->hfi_prev + (stateX->hfi_V - stateX->hfi_prev) / 2;
    }
    if (motor.state.is_v7) {
      stateX->hfi_Id_meas_high = stateX->Id_meas;
      stateX->hfi_Iq_meas_high = stateX->Iq_meas;
    }
    else {
      stateX->hfi_V_act = -stateX->hfi_V_act;
      stateX->hfi_Id_meas_low = stateX->Id_meas;
      stateX->hfi_Iq_meas_low = stateX->Iq_meas;
    }
    stateX->delta_id = stateX->hfi_Id_meas_high - stateX->hfi_Id_meas_low;
    stateX->delta_iq = stateX->hfi_Iq_meas_high - stateX->hfi_Iq_meas_low;

    if ( stateX->diq_compensation_on) {
      //stateX->delta_iq -= stateX->diq_compensation[ int(stateX->thetaPark * 180 / 2 / M_PI) ]; //Note: not adjusted yet for changing hfi_V
      stateX->compensation = stateX->diq_compensation[ int(stateX->thetaPark * 360 / 2 / M_PI) ];
    }
    else {
      stateX->compensation = 0;
    }

    //stateX->hfi_curangleest = 0.25f * atan2( -stateX->delta_iq  , stateX->delta_id - 0.5 * stateX->hfi_V * motor.conf.T * ( 1 / Ld + 1 / Lq ) ); //Complete calculation (not needed because error is always small due to feedback). 0.25 comes from 0.5 because delta signals are used and 0.5 due to 2theta (not just theta) being in the sin and cos wave.
    if (stateX->hfi_method == 1 || stateX->hfi_method == 3 ) {
      stateX->hfi_curangleest =  0.5f * stateX->delta_iq / (stateX->hfi_V * motor.conf.T * ( 1 / confX->Lq - 1 / confX->Ld ) ); //0.5 because delta_iq is twice the iq value
    }
    else if (stateX->hfi_method == 2 || stateX->hfi_method == 4) {
      if (motor.state.is_v7) {
        stateX->hfi_curangleest =  (stateX->Iq_meas - stateX->Iq_SP) / (stateX->hfi_V * motor.conf.T * ( 1 / confX->Lq - 1 / confX->Ld ) );
      }
      else {
        stateX->hfi_curangleest =  (stateX->Iq_meas - stateX->Iq_SP) / (-stateX->hfi_V * motor.conf.T * ( 1 / confX->Lq - 1 / confX->Ld ) );
      }
    }
    stateX->hfi_error = -stateX->hfi_curangleest; //Negative feedback
    if (stateX->hfi_use_lowpass) {
      stateX->hfi_error = hfi_lowpass->process( stateX->hfi_error );
    }
    stateX->hfi_dir_int += motor.conf.T * stateX->hfi_error * stateX->hfi_gain_int2; //This the the double integrator

    stateX->hfi_contout += stateX->hfi_gain * motor.conf.T * stateX->hfi_error + stateX->hfi_dir_int; //This is the integrator and the double integrator
    if (stateX->hfi_method == 3 || stateX->hfi_method == 4) {
      stateX->hfi_ffw = stateX->we * motor.conf.T;
      stateX->hfi_contout += stateX->hfi_ffw; //This is the feedforward
    }
    while ( stateX->hfi_contout >= 2 * M_PI) {
      stateX->hfi_contout -= 2 * M_PI;
    }
    while ( stateX->hfi_contout < 0) {
      stateX->hfi_contout += 2 * M_PI;
    }
    while ( stateX->hfi_contout >= 2 * M_PI) {
      stateX->hfi_contout -= 2 * M_PI;
    }
    while ( stateX->hfi_contout < 0) {
      stateX->hfi_contout += 2 * M_PI;
    }

    stateX->hfi_dir = stateX->hfi_contout + stateX->dist * stateX->hfi_distgain;

    while ( stateX->hfi_dir >= 2 * M_PI) {
      stateX->hfi_dir -= 2 * M_PI;
    }
    while ( stateX->hfi_dir < 0) {
      stateX->hfi_dir += 2 * M_PI;
    }
    while ( stateX->hfi_dir_int >= 2 * M_PI) {
      stateX->hfi_dir_int -= 2 * M_PI;
    }
    while ( stateX->hfi_dir_int < 0) {
      stateX->hfi_dir_int += 2 * M_PI;
    }
  }
  else {
    stateX->hfi_dir = stateX->thetaPark_obs;
    stateX->hfi_contout = stateX->thetaPark_obs;
    stateX->hfi_dir_int = 0;
    stateX->hfi_firstcycle = true;
    stateX->hfi_Id_meas_low = 0;
    stateX->hfi_Iq_meas_low = 0;
    stateX->hfi_Id_meas_high = 0;
    stateX->hfi_Iq_meas_high = 0;
    stateX->hfi_V_act = 0;
  }
  stateX->hfi_prev = stateX->hfi_V;

  if (confX->ridethewave != 1 ) {
    if (stateX->OutputOn == true) {
      stateX->Id_e = stateX->Id_SP - stateX->Id_meas;
      stateX->Iq_e = stateX->Iq_SP - stateX->Iq_meas;
    }
    else {
      stateX->Id_e = 0;
      stateX->Iq_e = 0;
    }

    stateX->Kp_iq_out = confX->Kp_iq * stateX->Iq_e;
    stateX->vq_int_state += confX->Ki_iq * motor.conf.T * stateX->Kp_iq_out;
    stateX->Ki_iq_out = stateX->Kp_iq_out + stateX->vq_int_state;
    LOWPASS( stateX->Vq_lp_out , stateX->Ki_iq_out, confX->lowpass_Vq_c);

    stateX->Vq_biquadout = BiquadsX[4]->process( stateX->Vq_lp_out );

    //Additional Vq
    stateX->Vq = stateX->Vq_biquadout;
    stateX->Vq += stateX->VSP;
    stateX->Vq += stateX->dist * stateX->Vq_distgain;
    stateX->Vq += stateX->hfi_V_act * stateX->compensation;

    //    stateX->Vd = confX->Kp_id * stateX->Id_e;
    //    stateX->vd_int_state += confX->Ki_id * motor.conf.T * stateX->Vd;;
    //    stateX->Vd += stateX->vd_int_state;
    //
    //    LOWPASS( stateX->Vd , stateX->Vd, confX->lowpass_Vd_c);

    stateX->Kp_id_out = confX->Kp_id * stateX->Id_e;
    stateX->vd_int_state += confX->Ki_id * motor.conf.T * stateX->Kp_id_out;
    stateX->Ki_id_out = stateX->Kp_id_out + stateX->vd_int_state;
    LOWPASS( stateX->Vd_lp_out , stateX->Ki_id_out, confX->lowpass_Vd_c);

    stateX->Vd_biquadout = BiquadsX[5]->process( stateX->Vd_lp_out );

    //Additional Vd
    stateX->Vd = stateX->Vd_biquadout;
    stateX->Vd += stateX->dist * stateX->Vd_distgain;
    stateX->Vd += stateX->hfi_V_act;

    // PMSM decoupling control and BEMF FF
    stateX->VqFF = stateX->we * ( confX->Ld * stateX->Id_meas + confX->Lambda_m);

    // q axis induction FFW based on setpoint FFW
    stateX->VqFF += SPprofile1->jref * stateX->Jload * confX->Lq * confX->Kt_Nm_Apeak * stateX->OutputOn;

    stateX->Vq += stateX->VqFF;

    stateX->VdFF = -stateX->we * confX->Lq * stateX->Iq_meas;
    stateX->Vd += stateX->VdFF;
  }

  stateX->Vq += stateX->muziek_gain_V * muziek[ (motor.state.curloop / (50 / (int)motor.conf.Ts)) % (sizeof(muziek) / 4) ];

  // Voltage clipping
  stateX->maxVolt = confX->maxDutyCycle * motor.state.sensBus_lp * ONE_BY_SQRT3;
  stateX->Vtot = NORM2_f( stateX->Vd , stateX->Vq );
  if ( stateX->Vtot > stateX->maxVolt) {
    if ( confX->clipMethod == 0 ) {
      if ( abs( stateX->Vd ) > stateX->maxVolt) {
        if (stateX->Vd > 0) {
          stateX->Vd = stateX->maxVolt;
        }
        else {
          stateX->Vd = -stateX->maxVolt;
        }
        if (abs(stateX->vd_int_state) > abs(stateX->Vd)) {
          stateX->vd_int_state = stateX->Vd;
        }
      }
      if (sq(stateX->Vd) >= sq(stateX->maxVolt)) { //Vd cannot be larger than maxvolt, so no issue with sqrt of negative values. Still get nan Vq somehow. Fix:
        stateX->Vq = 0;
      }
      else {
        if ( stateX->Vq > 0 ) {
          stateX->Vq = sqrt(sq(stateX->maxVolt) - sq(stateX->Vd)) ;
        }
        else {
          stateX->Vq = -sqrt(sq(stateX->maxVolt) - sq(stateX->Vd)) ;
        }
      }
      if (abs(stateX->vq_int_state) > abs(stateX->Vq)) {
        stateX->vq_int_state = stateX->Vq;
      }
    }
    else if ( confX->clipMethod == 1 ) {
      stateX->Vd *= (stateX->maxVolt / stateX->Vtot);
      stateX->Vq *= (stateX->maxVolt / stateX->Vtot);

      if (abs(stateX->vd_int_state) > abs(stateX->Vd)) {
        stateX->vd_int_state = stateX->Vd;
      }
      if (abs(stateX->vq_int_state) > abs(stateX->Vq)) {
        stateX->vq_int_state = stateX->Vq;
      }
    }
  }

  // Inverse park transform
  stateX->Valpha = stateX->co * stateX->Vd - stateX->si * stateX->Vq;
  stateX->Vbeta  = stateX->co * stateX->Vq + stateX->si * stateX->Vd;

  stateX->Valpha += stateX->Valpha_offset;
  stateX->Vbeta += stateX->Vbeta_offset;

  if (stateX->Valpha > stateX->maxVolt) {
    stateX->Valpha = stateX->maxVolt;
  }
  if (stateX->Vbeta > stateX->maxVolt) {
    stateX->Vbeta = stateX->maxVolt;
  }
  if (stateX->Valpha < -stateX->maxVolt) {
    stateX->Valpha = -stateX->maxVolt;
  }
  if (stateX->Vbeta < -stateX->maxVolt) {
    stateX->Vbeta = -stateX->maxVolt;
  }

  // Inverse Power-variant Clarke transform
  stateX->Va = stateX->Valpha;
  stateX->Vb = -0.5 * stateX->Valpha + SQRT3_by_2 * stateX->Vbeta;
  stateX->Vc = -0.5 * stateX->Valpha - SQRT3_by_2 * stateX->Vbeta;

  //See https://microchipdeveloper.com/mct5001:start Zero Sequence Modulation Tutorial
  float Vcm = -(max(max(stateX->Va, stateX->Vb), stateX->Vc) + min(min(stateX->Va, stateX->Vb), stateX->Vc)) / 2;
  stateX->Va += Vcm + motor.state.sensBus_lp / 2;
  stateX->Vb += Vcm + motor.state.sensBus_lp / 2;
  stateX->Vc += Vcm + motor.state.sensBus_lp / 2;

  //Calculate modulation times
  stateX->tA = stateX->Va / motor.state.sensBus_lp;
  stateX->tB = stateX->Vb / motor.state.sensBus_lp;
  stateX->tC = stateX->Vc / motor.state.sensBus_lp;
  // What is better, use lowpassed or raw bus voltage?
}


void current_and_duty_limts( mot_conf_t* confX , mot_state_t* stateX ) {
  truncate_number( &stateX->tA , 0.0 , 1.0);
  truncate_number( &stateX->tB , 0.0 , 1.0);
  truncate_number( &stateX->tC , 0.0 , 1.0);

  if ( (abs(stateX->Id_SP) > confX->I_max) || (abs(stateX->Iq_SP) > confX->I_max) ) {
    error(3 , stateX);
  }
  if ( (abs(stateX->Id_meas) > confX->I_max) || (abs(stateX->Iq_meas) > confX->I_max)  ) {
    error(4 , stateX);
  }
}


void changePWM() {
  //  //Motor 1, flexpwm4:
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_CLDOK( 7 );  //Enable changing of settings
  if (motor.state1.OutputOn == false) {
    //digitalWrite( ENGATE , 0);
    FLEXPWM4_SM0VAL3 = FLEXPWM4_SM0VAL1 / 2;
    FLEXPWM4_SM1VAL3 = FLEXPWM4_SM1VAL1 / 2;
    FLEXPWM4_SM2VAL3 = FLEXPWM4_SM2VAL1 / 2;
  }
  else {
    // Set duty cycles. FTM3_MOD = 100% (1800 for current settings, 20 kHz).
    //digitalWrite( ENGATE , 1);
    FLEXPWM4_SM0VAL3 = FLEXPWM4_SM0VAL1 * motor.state1.tB;
    FLEXPWM4_SM1VAL3 = FLEXPWM4_SM1VAL1 * motor.state1.tC;
    FLEXPWM4_SM2VAL3 = FLEXPWM4_SM2VAL1 * motor.state1.tA;
  }
  FLEXPWM4_SM0VAL2 = -FLEXPWM4_SM0VAL3;
  FLEXPWM4_SM1VAL2 = -FLEXPWM4_SM1VAL3;
  FLEXPWM4_SM2VAL2 = -FLEXPWM4_SM2VAL3;
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_LDOK( 7 ); //Activate settings


  //Motor 2, flexpwm2:
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK( 7 );  //Enable changing of settings
  if (motor.state2.OutputOn == false) {
    //digitalWrite( ENGATE2 , 0);
    FLEXPWM2_SM0VAL3 = FLEXPWM2_SM0VAL1 / 2;
    FLEXPWM2_SM1VAL3 = FLEXPWM2_SM1VAL1 / 2;
    FLEXPWM2_SM2VAL3 = FLEXPWM2_SM2VAL1 / 2;
  }
  else {
    // Set duty cycles. FTM3_MOD = 100% (1800 for current settings, 20 kHz).
    //digitalWrite( ENGATE2 , 1);
    FLEXPWM2_SM0VAL3 = FLEXPWM2_SM0VAL1 * motor.state2.tC;
    FLEXPWM2_SM1VAL3 = FLEXPWM2_SM1VAL1 * motor.state2.tB;
    FLEXPWM2_SM2VAL3 = FLEXPWM2_SM2VAL1 * motor.state2.tA;
  }
  FLEXPWM2_SM0VAL2 = -FLEXPWM2_SM0VAL3;
  FLEXPWM2_SM1VAL2 = -FLEXPWM2_SM1VAL3;
  FLEXPWM2_SM2VAL2 = -FLEXPWM2_SM2VAL3;
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK( 7 ); //Activate settings
}


void communicationProcess() {
  if (Serial.available() > 0) {
    processSerialIn();
  }
  if (trace.send_all) {
    static uint32_t i_total = 0;
    for ( uint32_t i = 0; i < 14; i++) {
      if (( trace.all_pointers[i_total] != NULL && i_total < sizeof(trace.all_pointers))) {
        Serial.write( trace.all_pointers[i_total] , trace.all_lengths[i_total]);
        i_total++;
      }
      else {
        i_total = 0;
        trace.send_all = false;
        break;
      }
    }
    delay(1);
  }

  if ( trace.n_to_send > 0) {
    if ( motor.state.downsample < 1) {
      motor.state.downsample = motor.conf.Ndownsample;
      for ( uint32_t i = 0; i < sizeof(trace.pointers) ; i++) {
        if ( trace.pointers[i] == NULL) {
          break;
        }
        Serial.write( trace.pointers[i] , trace.lengths[i]);
      }
      if ( trace.n_to_send < 1e6) {
        trace.n_to_send--;
      }
    }
    motor.state.downsample--;
  }

}

void processCommands( mot_conf_t* confX ,  mot_state_t* stateX ) {
  switch (confX->Command) {
    case UPDATE_CONTROLLER:
      {
        if (confX->Kp == 0) {
          stateX->rmechoffset -= stateX->emech;
          stateX->Kp_out_prev = 0;
          stateX->Ki_sum = 0;
          stateX->lp_out = 0;
        }
        confX->Kp = confX->Kp_prep;
        confX->Kd = confX->Kd_prep;
        confX->Ki = confX->Ki_prep;
        confX->lowpass_c = confX->lowpass_c_prep;
        confX->Command = NO_COMMAND;
        break;
      }
    case RESET_ERROR:
      {
        if (motor.state.firsterror > 0) {
          motor.state1.offsetVel_lp = 0;
          motor.state1.offsetVel = 0;
          motor.state1.offsetVelTot = 0;
          motor.state2.offsetVel_lp = 0;
          motor.state2.offsetVel = 0;
          motor.state2.offsetVelTot = 0;
          motor.state.firsterror = 0;
          motor.state1.firsterror = 0;
          motor.state2.firsterror = 0;
          motor.conf1.Kp = 0;
          motor.conf2.Kp = 0;
          digitalWrite( ENGATE , 1);
          SPI_init( SSPIN );  // Only for DRV8301.
          digitalWrite( ENGATE2 , 1);
          SPI_init( SSPIN2 );  // Only for DRV8301.
          motor.state1.OutputOn = true;
          motor.state2.OutputOn = true;
        }
        confX->Command = NO_COMMAND;
        break;
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
  char serialoption = Serial.read();
  uint32_t isignal;
  switch (serialoption) {
    case 'a':
      {
        trace.send_all = true;
        break;
      }
    case 'b':
      {
        Serial.readBytes( (char*)&trace.n_to_send , 4);
        break;
      }

    case 'g': //Get specific signal(s) from array
      {
        Serial.readBytes( (char*)&isignal , 4);
        uint32_t signallocation;
        Serial.readBytes( (char*)&signallocation , 4);
        uint32_t signallength;
        Serial.readBytes( (char*)&signallength , 4);
        uint8_t memberlength = signaltomemberlength( isignal );
        Serial.write( trace.all_pointers[isignal] + (signallocation * memberlength) , signallength );
        break;
      }
    case 'G': //Get signal
      {
        Serial.readBytes( (char*)&isignal , 4);
        Serial.write( trace.all_pointers[isignal] , trace.all_lengths[isignal]);
        break;
      }
    case 's': //Set specific paramter(s) in array
      {
        Serial.readBytes( (char*)&isignal , 4);
        uint32_t signallocation;
        Serial.readBytes( (char*)&signallocation , 4);
        uint32_t signallength;
        Serial.readBytes( (char*)&signallength , 4);
        uint8_t memberlength = signaltomemberlength( isignal );
        Serial.readBytes( (char*)trace.all_pointers[isignal] + (signallocation * memberlength) , signallength );
        break;
      }
    case 'S': //Set parameter
      {
        Serial.readBytes( (char*)&isignal , 4);
        Serial.readBytes( (char*)trace.all_pointers[isignal] , trace.all_lengths[isignal]);
        break;
      }
    case 't':
      {
        Serial.readBytes( (char*)&isignal , 4);
        uint8_t traceposition = Serial.read();
        if (isignal < sizeof( trace.all_types )) {
          trace.pointers[traceposition] = trace.all_pointers[isignal] ;
          trace.lengths[traceposition] = trace.all_lengths[isignal];
        }
        else {
          trace.pointers[traceposition] = NULL;
          trace.lengths[traceposition] = 0;
        }
        break;
      }
    case 'T':
      {
        for (uint32_t i = 0; i < sizeof( trace.all_types ) ; i ++) {
          if ( trace.names[i] == NULL) {
            break;
          }
          Serial.println( trace.names[i] );
          Serial.println( trace.all_types[i] );
          Serial.println( trace.all_lengths[i] / signaltomemberlength(i) );
        }
        break;
      }

    case  '1':
      {
        Serial.readBytes( (char*)&SPprofile1->t1 , 4);
        Serial.readBytes( (char*)&SPprofile1->t2 , 4);
        Serial.readBytes( (char*)&SPprofile1->t3 , 4);
        Serial.readBytes( (char*)&SPprofile1->p , 8);
        Serial.readBytes( (char*)&SPprofile1->v_max , 4);
        Serial.readBytes( (char*)&SPprofile1->a_max , 4);
        Serial.readBytes( (char*)&SPprofile1->j_max , 4);
        SPprofile1->init();
        break;
      }
    case  '2':
      {
        Serial.readBytes( (char*)&SPprofile2->t1 , 4);
        Serial.readBytes( (char*)&SPprofile2->t2 , 4);
        Serial.readBytes( (char*)&SPprofile2->t3 , 4);
        Serial.readBytes( (char*)&SPprofile2->p , 8);
        Serial.readBytes( (char*)&SPprofile2->v_max , 4);
        Serial.readBytes( (char*)&SPprofile2->a_max , 4);
        Serial.readBytes( (char*)&SPprofile2->j_max , 4);
        SPprofile2->init();
        break;
      }

    case  'N': //Notch
      {
        uint32_t axis;
        Serial.readBytes( (char*)&axis , 4);
        Serial.readBytes( (char*)&isignal , 4);
        float f0;
        Serial.readBytes( (char*)&f0 , 4);
        float debthdb;
        Serial.readBytes( (char*)&debthdb , 4);
        float notch_width;
        Serial.readBytes( (char*)&notch_width , 4);
        if (axis == 1) {
          Biquads1[isignal]->setNotch( f0, debthdb, notch_width, 2 * F_PWM);
        }
        else if (axis == 2) {
          Biquads2[isignal]->setNotch( f0, debthdb, notch_width, 2 * F_PWM);
        }
        break;
      }
      
    case  'L': //Lowpass
      {
        uint32_t axis;
        Serial.readBytes( (char*)&axis , 4);
        Serial.readBytes( (char*)&isignal , 4);
        float f0;
        Serial.readBytes( (char*)&f0 , 4);
        float damp;
        Serial.readBytes( (char*)&damp , 4);
        if (axis == 1) {
          Biquads1[isignal]->setBiquad( bq_type_lowpass , f0, damp, 2 * F_PWM);
        }
        else if (axis == 2) {
          Biquads2[isignal]->setBiquad( bq_type_lowpass , f0, damp, 2 * F_PWM);
        }
        break;
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


static inline void truncate_number(float *number, float min, float max) {
  if (*number > max) {
    *number = max;
  } else if (*number < min) {
    *number = min;
  }
}

static inline void truncate_number_int(int *number, int min, int max) {
  if (*number > max) {
    *number = max;
  } else if (*number < min) {
    *number = min;
  }
}

static inline void truncate_number_abs(float *number, float max) {
  if (*number > max) {
    *number = max;
  } else if (*number < -max) {
    *number = -max;
  }
}

void error( int ierror ,  mot_state_t* stateX ) {
  motor.state1.OutputOn = false;
  motor.state2.OutputOn = false;
  digitalWrite( ENGATE , 0);
  digitalWrite( ENGATE2 , 0);
  if (motor.state.firsterror == 0) {
    motor.state.firsterror = ierror;
    stateX->firsterror = ierror;
  }
}
