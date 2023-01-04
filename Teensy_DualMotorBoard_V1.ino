#include <Math.h>
#include <arm_math.h>
#include <SPI.h>

#include "Biquad.h"
#include "ControlTools.h"
#include "muziek.c"
#include "QuadEncoder.h"
#include "MotionProfile.h"
#include "defines.h"
#include "trace.h"

Biquad *lowpass_sensbus   = new Biquad( bq_type_lowpass , 500 , 0.7, 2 * f_pwm);
Integrator *integrator = new Integrator( 1 , 2 * f_pwm);
Integrator *integrator2 = new Integrator( 1 , 2 * f_pwm);

LeadLag *leadlag       = new LeadLag( 10 , 3 , 3 , 2 * f_pwm);
Biquad *lowpass        = new Biquad( bq_type_lowpass , 50 , 0.7, 2 * f_pwm);
Biquad *lowpass_eradpers   = new Biquad( bq_type_lowpass , 50 , 0.7, 2 * f_pwm);
//Biquad *notch          = new Biquad( bq_type_notch , 2315.0, -20.0, 0.1 , 2 * f_pwm );

LeadLag *leadlag2       = new LeadLag( 10 , 3 , 3 , 2 * f_pwm);
Biquad *lowpass2        = new Biquad( bq_type_lowpass , 50 , 0.7, 2 * f_pwm);


//Current lowpass (now used at sensor level, maybe better at id,iq level?). Doesn't seem to matter much.
Biquad *lowpassIsens1  = new Biquad( bq_type_lowpass , 10e3 , 0.7, 2 * f_pwm);
Biquad *lowpassIsens2  = new Biquad( bq_type_lowpass , 10e3 , 0.7, 2 * f_pwm);
Biquad *lowpassIsens3  = new Biquad( bq_type_lowpass , 10e3 , 0.7, 2 * f_pwm);
Biquad *lowpassIsens4  = new Biquad( bq_type_lowpass , 10e3 , 0.7, 2 * f_pwm);

//These are now used for power and bus current estimates
Biquad *lowpassId1  = new Biquad( bq_type_lowpass , 50 , 0.7, 2 * f_pwm);
Biquad *lowpassIq1  = new Biquad( bq_type_lowpass , 50 , 0.7, 2 * f_pwm);

Biquad *hfi_lowpass = new Biquad( bq_type_lowpass , 2000 , 0.707, 2 * f_pwm);

// For setpoint
Biquad *lowpassSP = new Biquad( bq_type_lowpass , 10 , 0.707, 2 * f_pwm);

//fast 180 deg:
MotionProfile *SPprofile = new MotionProfile( 0 , 0.000500000000000000 , 0.0193000000000000 , 0 , 3.14159265358979 , 157.079632679490 , 7853.98163397448 , 15632147.3532855 , 1 / (2 * f_pwm) );

Biquad *lowpass_ss_offset = new Biquad( bq_type_lowpass , 10 , 0.707, 2 * f_pwm);

//There are 4 hardware quadrature encoder channels available the Teensy 4.x.
//The Teensy 4.1 Encoders are supported on pins: 0, 1, 2, 3, 4, 5, 7, 30, 31, 33, 36 and 37.
//WARNING! Pins 0, 5 and 37 share the same internal crossbar connections and are as such exclusive...pick one or the other.
QuadEncoder Encoder1(1, 0, 1 , 0 , 3);   //Encoder 1 on pins 0 and 1, index on pin 3
QuadEncoder Encoder2(2, 30, 31 , 0 , 33);//Encoder 2 on pins 30 and 31, index on pin 33


void initparams( motor_total_t* m ) {
  m->conf.Ts = 1e6 / (2 * f_pwm);
  m->conf.T = m->conf.Ts / 1e6;
  m->conf.Busadc2Vbus = 1 / 4095.0 * 3.3 * ((68.3 + 5.05) / 5.05); //5.1 changed to 5.05 to improve accuracy. May differ board to board.
  m->conf.V_Bus = 24; //Bus Voltage
  m->conf.Ndownsample = 1;

  m->state1.OutputOn = true;

  initmotor( &m->conf1 , &m->state1);
  initmotor( &m->conf2 , &m->state2);
}

void initmotor( mot_conf_t* m , mot_state_t* state ) {
  m->maxDutyCycle = 0.99;
  m->adc2A = 1 / 0.09; //With linear hal current sensors ACS711 (31A or 15.5A): These allow for 2 measurements per PWM cycle

  m->enccountperrev = 20000;
  m->enc2rad = 2 * M_PI / m->enccountperrev;
  m->I_max = 15; //Max current (peak of the sine wave in a phase)
  m->max_edeltarad = 0.25f * M_PI;

  m->maxerror = 0.5;

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

  state->lfsr = 0xACE1u;
  state->ss_f = 1;
}

void setup() {
  initparams( &motor );

  Serial.begin(1);
  pinMode( ENGATE , OUTPUT);
  pinMode( ENGATE2 , OUTPUT);

  digitalWrite( ENGATE , 1); // To be updated!
  digitalWrite( ENGATE2 , 1); // To be updated!

  SPI_init( SSPIN );  // Only for DRV8301. Disable this for DRV8302
  //  SPI_init( SSPIN2 );  // Only for DRV8301. Disable this for DRV8302
  DRV8302_init( SSPIN2 , 13 ); // Note: pin 13 is also the SCLK pin for communication with DRV8301 and the LED.
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

//  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 &= ~ (1 << 12) ; // disable keeper pin 14, as per manual
//  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_03 &= ~ (1 << 12) ; // disable keeper pin 15, as per manual
//  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_07 &= ~ (1 << 12) ; // disable keeper pin 16, as per manual
//  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_06 &= ~ (1 << 12) ; // disable keeper pin 17, as per manual
//  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_01 &= ~ (1 << 12) ; // disable keeper pin 18, as per manual
//  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_00 &= ~ (1 << 12) ; // disable keeper pin 19, as per manual

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

  /* ADC channel, pin numbers
   
   Note: 0 to 13 is same as 14 to 27. Do not try to use these for analog in!
        7,      // 0/A0  AD_B1_02   ENC1_A             
        8,      // 1/A1  AD_B1_03   ENC1_B
        12,     // 2/A2  AD_B1_07   M1 out A INH-A
        11,     // 3/A3  AD_B1_06   ENC1_I
        6,      // 4/A4  AD_B1_01   M2 out C INH-C2
        5,      // 5/A5  AD_B1_00   M2 out B INH-B2
        15,     // 6/A6  AD_B1_10   M2 out A INH-A2
        0,      // 7/A7  AD_B1_11   M2 Ia              --> Doesn't work       
        13,     // 8/A8  AD_B1_08   M2 Ib              --> Doesn't work                       
        14,     // 9/A9  AD_B1_09   M2 Ic              --> Doesn't work       
        255,  // 10/A10 AD_B0_12   -
        255,  // 11/A11 AD_B0_13    SDI (SPI)
        3,      // 12/A12 AD_B1_14  SDO (SPI)
        4,      // 13/A13 AD_B1_15  SCLK (SPI)

        7,      // 14/A0  AD_B1_02  M1 Ic
        8,      // 15/A1  AD_B1_03  M1 Ib
        12,     // 16/A2  AD_B1_07  M1 Ia
        11,     // 17/A3  AD_B1_06  M1 Vbus
        6,      // 18/A4  AD_B1_01  M1 EMF-A
        5,      // 19/A5  AD_B1_00  M1 EMF-B
        15,     // 20/A6  AD_B1_10  M1 EMF-C
        0,      // 21/A7  AD_B1_11  M2 Vbus
        13,     // 22/A8  AD_B1_08  M1 out C INH-C
        14,     // 23/A9  AD_B1_09  M1 out B INH-B
        255,    // 24/A10 AD_B0_12  M2 EMF-A  --> Exchanged for M2 Ia 
        255,    // 25/A11 AD_B0_13  M2 EMF-B
        3,      // 26/A12 AD_B1_14  M2 EMF-C  --> Exchanged for M2 Ib 
        4,      // 27/A13 AD_B1_15  M2 nOCTW
        255,    // 28               M2 PWRGD
        255,    // 29
        255,    // 30               ENC2_A
        255,    // 31               ENC2_B
        255,    // 32
        255,    // 33               ENC2_I
        255,    // 34               M1 EN-GATE
        255,    // 35               M1 nSCS
        255,    // 36               M1 nFAULT
        255,    // 37               M1 nOCTW
        1,      // 38/A14 AD_B1_12  M1 PWRGD
        2,      // 39/A15 AD_B1_13  M2 EN-GATE
        9,      // 40/A16 AD_B1_04  M2 nSCS
        10,     // 41/A17 AD_B1_05  M2 nFAULT

    Actual chain:
        12,     // 16/A2  AD_B1_07  M1 Ia
        8,      // 15/A1  AD_B1_03  M1 Ib

        1,      // 24/A10 AD_B0_12  M2 EMF-A  --> Exchanged for M2 Ia  // Analog channel 1 input 1 NOTE: only on ADC1 !
        3,      // 26/A12 AD_B1_14  M2 EMF-C  --> Exchanged for M2 Ib  // Analog channel 2 input 3 NOTE: only on ADC2

        11,     // 17/A3  AD_B1_06  M1 Vbus
        0,      // 21/A7  AD_B1_11  M2 Vbus

  Removed due to not working (wrong pin, these do not have ADC connection):
        0,      // 7/A7  AD_B1_11   M2 Ia
        13,     // 8/A8  AD_B1_08   M2 Ib

  */

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
  motor.state.curtime = micros();
  motor.state.is_v7 = (FLEXPWM2_SM0STS & FLEXPWM_SMSTS_CMPF(2));  //is_v7 = True when in v7
  FLEXPWM2_SM0STS |= FLEXPWM_SMSTS_CMPF(2); //Reset flag

  //ADC1: 
  motor.state.sens1 = (ADC_ETC_TRIG0_RESULT_1_0 & 4095) * 0.0008058608; // 4095.0 * 3.3;
  motor.state.sens1_lp = lowpassIsens1->process( motor.state.sens1 );
  motor.state.sens3 = ((ADC_ETC_TRIG0_RESULT_1_0 >> 16) & 4095) * 0.0008058608; // 4095.0 * 3.3;
  motor.state.sens3_lp = lowpassIsens3->process( motor.state.sens3 );
  motor.state.sensBus = (ADC_ETC_TRIG0_RESULT_3_2 & 4095) * motor.conf.Busadc2Vbus;   // 4095.0 * 3.3 * ((68.3+5.05)/5.05);
  motor.state.sensBus_lp = lowpass_sensbus->process( motor.state.sensBus );

  //ADC2:
  motor.state.sens2 = (ADC_ETC_TRIG4_RESULT_1_0 & 4095) * 0.0008058608; // 4095.0 * 3.3;
  motor.state.sens2_lp = lowpassIsens2->process( motor.state.sens2 );
  motor.state.sens4 = ((ADC_ETC_TRIG4_RESULT_1_0 >> 16) & 4095) * 0.0008058608; // 4095.0 * 3.3;
  motor.state.sens4_lp = lowpassIsens4->process( motor.state.sens4 );
  motor.state.sensBus2 = (ADC_ETC_TRIG4_RESULT_3_2 & 4095) * motor.conf.Busadc2Vbus;   // 4095.0 * 3.3 * ((68.3+5.05)/5.05);

  if ( motor.state.sensBus > motor.conf.V_Bus + 1 ) {
    //    digitalWrite( chopperpin , HIGH);
  }
  else {
    //    digitalWrite( chopperpin , LOW);
  }
  if ( motor.state.sensBus > 45 or motor.state.sensBus2 > 45 ) {
    motor.state1.OutputOn = false;
    motor.state2.OutputOn = false;
    if (motor.state.firsterror == 0) {
      motor.state.firsterror = 41;
    }
  }
    
  if (motor.state.setupready == 1) {
    if (motor.state.n_senscalib < 1e4) {
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
      GenSetpoint();
      readENC();
      Control();
      Transforms();
      changePWM();
      communicationProcess();
      motor.state.curloop++;
    }
  }
}


void updateDisturbance() {
  //PRBS
  motor.state1.downsamplePRBS++;
  if ( motor.state1.downsamplePRBS > motor.conf1.NdownsamplePRBS) {
    motor.state1.downsamplePRBS = 1;
    motor.state1.noisebit  = ((motor.state1.lfsr >> 5) ^ (motor.state1.lfsr >> 7) ) & 1;  // taps: 11 9; feedback polynomial: x^11 + x^9 + 1
    motor.state1.lfsr =  (motor.state1.lfsr >> 1) | (motor.state1.noisebit << 15);
  }

  //Single Sine
  if ( motor.state.curtime / 1e6 >= (motor.state1.ss_tstart + motor.conf1.ss_n_aver / motor.state1.ss_f )) {
    motor.state1.ss_f += motor.conf1.ss_fstep;
    motor.state1.ss_tstart = motor.state.curtime / 1e6;
    if (motor.state1.ss_f > motor.conf1.ss_fend)
    {
      motor.state1.ss_f = 0;
      motor.state1.ss_phase = 0;
      motor.state1.ss_tstart = 1e8;
      motor.conf1.ss_gain = 0;
      motor.conf1.ss_offset = 0;
    }
  }
  motor.state1.ss_phase += motor.state1.ss_f * 2 * M_PI * motor.conf.T;
  if ( motor.state1.ss_phase >= 2 * M_PI) {
    motor.state1.ss_phase -= 2 * M_PI; //Required, because high value floats are inaccurate
  }
  float ss_offset_lp = lowpass_ss_offset->process( motor.conf1.ss_offset );
  //motor.state1.ss_out = ss_offset_lp + motor.conf1.ss_gain * arm_sin_f32( motor.state1.ss_phase ); //sin() measured to be faster then sinf(); arm_sin_f32() is way faster!
  motor.state1.ss_out = ss_offset_lp + motor.conf1.ss_gain * sin( motor.state1.ss_phase );
  motor.state1.dist = motor.state1.distval * 1 * (motor.state1.noisebit - 0.5) + motor.state1.distoff + motor.state1.ss_out;
}


void GenSetpoint() {
  SPprofile->REFidir = motor.setpoint.SPdir;
  SPprofile->rdelay = motor.state1.rdelay;

  if (SPprofile->REFstatus != 1) { // if not running SP
    SPprofile->REFstatus = 0; //Make sure sp generator is ready for sp generation
    if (motor.setpoint.spNgo > 0) {
      motor.state1.spGO = 1;
      motor.setpoint.spNgo -= 1;
    }
    else {
      motor.state1.spGO = 0;
    }
  }
  motor.state1.REFstatus = SPprofile->REFstatus;
  motor.state1.rmech = SPprofile->stateCalculation( motor.state1.spGO );

  motor.state1.offsetVel_lp = lowpassSP->process( motor.state1.offsetVel );
  motor.state1.offsetVelTot += motor.state1.offsetVel_lp * motor.conf.T;
  motor.state1.rmech += motor.state1.offsetVelTot ;

  motor.state2.rmech = -motor.state1.rmech;
  motor.state1.rmech += motor.state1.rmechoffset;
  motor.state2.rmech += motor.state2.rmechoffset;

  motor.state1.acc = SPprofile->aref;
  motor.state1.vel = SPprofile->vref + motor.state1.offsetVelTot;
  motor.state1.we = motor.state1.vel * motor.conf1.N_pp;  //Electrical speed [rad/s], based on setpoint

  motor.state2.acc = -motor.state1.acc;
  motor.state2.vel = -motor.state1.vel;
  motor.state2.we  = motor.state2.vel * motor.conf2.N_pp;  //Electrical speed [rad/s], based on setpoint

  //When no setpoint is running, always convert reference to nearest encoder count to avoid noise
  if (SPprofile->REFstatus == 0 && motor.state1.offsetVel_lp == 0) {
    motor.state1.rmech = int((motor.state1.rmech / motor.conf1.enc2rad)) * motor.conf1.enc2rad;
  }
  if (SPprofile->REFstatus == 0 && motor.state1.offsetVel_lp == 0) { //Note: refers to state1.
    motor.state2.rmech = int((motor.state2.rmech / motor.conf2.enc2rad)) * motor.conf2.enc2rad;
  }

}

void readENC() {
  motor.state1.encoderPos1 = Encoder1.read();
  motor.state1.encoderPos2 = Encoder2.read();
  motor.state1.IndexFound1 = Encoder1.indexfound();
  motor.state1.IndexFound2 = Encoder2.indexfound();

  //Hack
//  motor.state1.encoderPos2 = Encoder1.read();
//  motor.state1.encoderPos1 = Encoder2.read();
//  motor.state1.IndexFound2 = Encoder1.indexfound();
//  motor.state1.IndexFound1 = Encoder2.indexfound();
}

void Control() {
  motor.state1.ymech = motor.state1.encoderPos1 * motor.conf1.enc2rad;
  motor.state2.ymech = motor.state1.encoderPos2 * motor.conf2.enc2rad;
  if (motor.hfi1.hfi_useforfeedback == 1) {
    motor.state1.ymech = motor.hfi1.hfi_abs_pos / motor.conf1.N_pp;
  }
  motor.state1.emech = motor.state1.rmech - motor.state1.ymech;
  motor.state2.emech = motor.state2.rmech - motor.state2.ymech;
  if (motor.conf1.haptic == 1) {
    motor.state1.emech = motor.state1.rmech - motor.state1.ymech - motor.state2.ymech;
    motor.state2.emech = 0;
  }

  if ((abs(motor.state1.emech) > motor.conf1.maxerror) & (motor.conf1.Kp > 0) )
  {
    motor.state1.OutputOn = false;
    if (motor.state.firsterror == 0) {
      motor.state.firsterror = 1;
    }
  }
  if (motor.state1.OutputOn == false) {
    motor.state1.emech = 0;
    integrator->setState(0);
    lowpass->InitStates(0);
  }
  //  if (abs(motor.state2.emech) > 0.5 & motor.conf2.Kp > 0 )
  //  {
  //    motor.state1.OutputOn = false;
  //    if (motor.state.firsterror == 0) {
  //      motor.state.firsterror = 21;
  //    }
  //  }
  if (motor.state1.OutputOn == false) {
    motor.state2.emech = 0;
    integrator2->setState(0);
    lowpass2->InitStates(0);
  }
  if (motor.conf1.Kp == 0) {
    integrator->setState(0);
  }
  if (motor.conf2.Kp == 0) {
    integrator2->setState(0);
  }

  motor.state1.mechcontout = motor.conf1.Kp * leadlag->process( motor.state1.emech );
  motor.state1.mechcontout = lowpass->process( motor.state1.mechcontout );

  motor.state2.mechcontout = motor.conf2.Kp * leadlag2->process( motor.state2.emech );
  motor.state2.mechcontout = lowpass2->process( motor.state2.mechcontout );

  // Clipping to be improved...
  motor.state1.Iout = integrator->processclip( motor.state1.mechcontout , -motor.conf1.I_max * (1.5 * motor.conf1.N_pp * motor.conf1.Lambda_m ) - motor.state1.mechcontout , motor.conf1.I_max * (1.5 * motor.conf1.N_pp * motor.conf1.Lambda_m ) - motor.state1.mechcontout );
  //motor.state2.Iout = integrator->processclip( motor.state1.mechcontout , -motor.conf1.I_max * (1.5 * motor.conf1.N_pp * motor.conf1.Lambda_m ) - motor.state1.mechcontout , motor.conf1.I_max * (1.5 * motor.conf1.N_pp * motor.conf1.Lambda_m ) - motor.state1.mechcontout );

  motor.state1.mechcontout += motor.state1.Iout;
  if (motor.state1.OutputOn) {
    motor.state1.mechcontout += motor.state1.acc * motor.state1.Jload;
    motor.state1.mechcontout += motor.state1.vel * motor.state1.velFF;
  }
  motor.state1.mechcontout += motor.state1.dist * motor.state1.mechdistgain;

  motor.state2.mechcontout += motor.state2.Iout;
  if (motor.state1.OutputOn) {
    motor.state2.mechcontout += motor.state2.acc * motor.state2.Jload;
    motor.state2.mechcontout += motor.state2.vel * motor.state2.velFF;
  }
  motor.state2.mechcontout += motor.state1.dist * motor.state2.mechdistgain;

  if (motor.conf1.haptic == 1) {
    motor.state2.mechcontout = motor.state1.mechcontout;
  }


  if (motor.state1.OutputOn == false) {
    motor.state1.vq_int_state = 0;
    motor.state1.vd_int_state = 0;
    motor.state2.vq_int_state = 0;
    motor.state2.vd_int_state = 0;
  }

  motor.state1.Iq_SP = motor.state1.mechcontout / (1.5 * motor.conf1.N_pp * motor.conf1.Lambda_m );
  motor.state2.Iq_SP = motor.state2.mechcontout / (1.5 * motor.conf2.N_pp * motor.conf2.Lambda_m );
}

void Transforms()
{
  // Calculate currents
  if (motor.conf.useIlowpass == 1)
  {
    motor.state1.ia = motor.conf1.adc2A * (motor.state.sens1_lp - motor.state.sens1_calib);
    motor.state1.ib = motor.conf1.adc2A * (motor.state.sens2_lp - motor.state.sens2_calib);
    motor.state2.ia = motor.conf2.adc2A * (motor.state.sens3_lp - motor.state.sens3_calib);
    motor.state2.ib = motor.conf2.adc2A * (motor.state.sens4_lp - motor.state.sens4_calib);
  }
  else {
    motor.state1.ia = motor.conf1.adc2A * (motor.state.sens1 - motor.state.sens1_calib);
    motor.state1.ib = motor.conf1.adc2A * (motor.state.sens2 - motor.state.sens2_calib);
    motor.state2.ia = motor.conf2.adc2A * (motor.state.sens3 - motor.state.sens3_calib);
    motor.state2.ib = motor.conf2.adc2A * (motor.state.sens4 - motor.state.sens4_calib);
  }
  motor.state1.ic = -motor.state1.ia - motor.state1.ib;
  motor.state2.ic = -motor.state2.ia - motor.state2.ib;

  //HACK 
//  motor.state1.ia = motor.state2.ia;
//  motor.state1.ib = motor.state2.ib;
//  motor.state1.ic = motor.state2.ic;

  // For Park and Clarke see https://www.cypress.com/file/222111/download
  // Power-variant Clarke transform. Asuming ia+ib+ic=0:
  motor.state1.Ialpha = motor.state1.ia;
  motor.state1.Ibeta = one_by_sqrt3 * motor.state1.ia + two_by_sqrt3 * motor.state1.ib;

  //  motor.state2.Ialpha = motor.state2.ia;
  //  motor.state2.Ibeta = one_by_sqrt3 * motor.state2.ia + two_by_sqrt3 * motor.state2.ib;

  // Park transform, ride the wave option
  motor.state1.thetaPark_enc = motor.conf1.N_pp * (motor.state1.encoderPos1 % motor.conf1.enccountperrev) * motor.conf1.enc2rad + motor.conf1.commutationoffset; //Modulo on the encoder counts to keep the floating point 0 to 2pi for numerical accuracy
  if (motor.conf1.reversecommutation) {
    motor.state1.thetaPark_enc *= -1;
  }
  while ( motor.state1.thetaPark_enc >= 2 * M_PI) {
    motor.state1.thetaPark_enc -= 2 * M_PI;
  }
  while ( motor.state1.thetaPark_enc < 0) {
    motor.state1.thetaPark_enc += 2 * M_PI;
  }

  //Angle observer by mxlemming
  float L = (motor.conf1.Ld + motor.conf1.Lq) / 2;
  motor.state1.BEMFa = motor.state1.BEMFa + (motor.state1.Valpha - motor.state1.R * motor.state1.Ialpha) * motor.conf.T -
                       L * (motor.state1.Ialpha - motor.state1.Ialpha_last);
  motor.state1.BEMFb = motor.state1.BEMFb + (motor.state1.Vbeta - motor.state1.R * motor.state1.Ibeta) * motor.conf.T -
                       L * (motor.state1.Ibeta - motor.state1.Ibeta_last)  ;
  motor.state1.Ialpha_last = motor.state1.Ialpha;
  motor.state1.Ibeta_last = motor.state1.Ibeta;
  if (motor.state1.BEMFa > motor.conf1.Lambda_m  ) {
    motor.state1.BEMFa = motor.conf1.Lambda_m ;
  }
  if (motor.state1.BEMFa < -motor.conf1.Lambda_m ) {
    motor.state1.BEMFa = -motor.conf1.Lambda_m ;
  }
  if (motor.state1.BEMFb > motor.conf1.Lambda_m ) {
    motor.state1.BEMFb = motor.conf1.Lambda_m ;
  }
  if (motor.state1.BEMFb < -motor.conf1.Lambda_m ) {
    motor.state1.BEMFb = -motor.conf1.Lambda_m ;
  }
  motor.state1.thetaPark_obs = atan2(motor.state1.BEMFb, motor.state1.BEMFa);

  while ( motor.state1.thetaPark_obs >= 2 * M_PI) {
    motor.state1.thetaPark_obs -= 2 * M_PI;
  }
  while ( motor.state1.thetaPark_obs < 0) {
    motor.state1.thetaPark_obs += 2 * M_PI;
  }
  //Check and remove nan
  if (motor.state1.thetaPark_obs != motor.state1.thetaPark_obs) {
    motor.state1.thetaPark_obs = motor.state1.thetaPark_obs_prev;
  }
  motor.state1.thetaPark_obs_prev = motor.state1.thetaPark_obs;


  if (motor.conf1.anglechoice == 0) {
    motor.state1.thetaPark = motor.state1.thetaPark_enc;
  }
  else if (motor.conf1.anglechoice == 1) {
    motor.state1.thetaPark = motor.state1.thetaPark_obs;
  }
  else if (motor.conf1.anglechoice == 3 ) {
    if ( abs(motor.state1.vel) < motor.hfi1.hfi_maxvel ) {
      motor.hfi1.hfi_on = true;
      motor.state1.thetaPark = motor.hfi1.hfi_dir;
    }
    else {
      motor.hfi1.hfi_on = false;
      motor.state1.thetaPark = motor.state1.thetaPark_obs;
    }
  }
  else if (motor.conf1.anglechoice == 99) {
    utils_step_towards((float*)&motor.state1.i_vector_radpers_act, motor.state1.i_vector_radpers, motor.state1.i_vector_acc * motor.conf.T );
    motor.state1.thetaPark += motor.conf.T * motor.state1.i_vector_radpers_act;
  }
  else if (motor.conf1.anglechoice == 100) {
    //Empty such that thethaPark can be set from host.
  }
  else {
    motor.state1.thetaPark = 0;
  }

  // Phase advance
  motor.state1.thetaPark += motor.state1.eradpers_lp * motor.conf.T * motor.conf1.advancefactor;

  while ( motor.state1.thetaPark >= 2 * M_PI) {
    motor.state1.thetaPark -= 2 * M_PI;
  }
  while ( motor.state1.thetaPark < 0) {
    motor.state1.thetaPark += 2 * M_PI;
  }

  // erpm estimator
  motor.state1.edeltarad = motor.state1.thetaPark - motor.state1.thetaParkPrev;
  if (motor.state1.edeltarad > M_PI) {
    motor.state1.edeltarad -= 2 * M_PI;
  }
  if (motor.state1.edeltarad < -M_PI) {
    motor.state1.edeltarad += 2 * M_PI;
  }
  //Limit change of motor.state1.thetaPark to 45 deg per cycle:
  if (motor.state1.edeltarad > motor.conf1.max_edeltarad) {
    motor.state1.edeltarad = motor.conf1.max_edeltarad;
    motor.state1.thetaPark = motor.state1.thetaParkPrev + motor.state1.edeltarad;
  }
  else if (motor.state1.edeltarad < -motor.conf1.max_edeltarad) {
    motor.state1.edeltarad = -motor.conf1.max_edeltarad;
    motor.state1.thetaPark = motor.state1.thetaParkPrev + motor.state1.edeltarad;
  }
  motor.state1.eradpers_lp = lowpass_eradpers->process( motor.state1.edeltarad / motor.conf.T );
  motor.state1.erpm = motor.state1.eradpers_lp * 60 / (2 * M_PI);
  motor.state1.thetaParkPrev = motor.state1.thetaPark;
  motor.hfi1.hfi_abs_pos += motor.state1.edeltarad;

  //  thetaPark2 = 8 * (motor.state1.encoderPos2 % enccountperrev2) * motor.conf2.enc2rad + commutationoffset2; //Modulo on the encoder counts to keep the floating point 0 to 2pi for numerical accuracy
  //  while ( thetaPark2 >= 2 * M_PI) {
  //    thetaPark2 -= 2 * M_PI;
  //  }
  //  while ( thetaPark2 < 0) {
  //    thetaPark2 += 2 * M_PI;
  //  }

  if (motor.conf1.ridethewave == 1 ) {
    if ((motor.state1.IndexFound1) < 1 ) {
      motor.state1.thetaPark = motor.state1.thetawave;
      motor.state1.thetawave -= 10 * 2 * M_PI * motor.conf.T;
      motor.state1.Vq = 1.5;
    }
    else {
      motor.state1.Vq = 0;
      motor.conf1.ridethewave = 2;
      motor.state1.thetawave = 0;
    }
  }

  /*   if (ridethewave2 == 1 ) {
      if ((motor.state1.IndexFound2) < 1 ) {
        thetaPark2 = thetawave2;
        thetawave2 -= 10 * 2 * M_PI * motor.conf.T;
        Vq2 = 1.5;
      }
      else {
        Vq2 = 0;
        ridethewave2 = 2;
        thetawave2 = 0;
      }
    } */

  motor.state1.Iq_SP += motor.state1.muziek_gain * muziek[ (motor.state.curloop / (50 / (int)motor.conf.Ts)) % (sizeof(muziek) / 4) ];
  motor.state1.Iq_SP += motor.state1.dist * motor.state1.Iq_distgain;

  motor.state1.Iq_SP += motor.state1.Iq_offset_SP;

  motor.state1.Id_SP = motor.state1.Id_offset_SP;
  motor.state1.Id_SP += motor.state1.dist * motor.state1.Id_distgain;

  // motor.state2.Iq_SP += motor.state2.muziek_gain * muziek[ (motor.state.curloop / (50 / (int)motor.conf.Ts)) % (sizeof(muziek) / 4) ];
  // motor.state2.Iq_SP += motor.state2.dist * motor.state2.Iq_distgain;

  // Id_SP2 = Id_offset_SP2;
  // Id_SP2 += motor.state1.dist * motor.state1.Id_distgain;

  motor.state1.co = cos(motor.state1.thetaPark);
  motor.state1.si = sin(motor.state1.thetaPark);

  // co2 = cos(thetaPark2);
  // si2 = sin(thetaPark2);


  // Park transform
  motor.state1.Id_meas = motor.state1.co * motor.state1.Ialpha + motor.state1.si * motor.state1.Ibeta;
  motor.state1.Iq_meas = motor.state1.co * motor.state1.Ibeta  - motor.state1.si * motor.state1.Ialpha;

  motor.state1.Id_meas_lp = lowpassId1->process( motor.state1.Id_meas );
  motor.state1.Iq_meas_lp = lowpassIq1->process( motor.state1.Iq_meas );

  // Id_meas2 = co2 * Ialpha2 + si2 * Ibeta2;
  // Iq_meas2 = co2 * Ibeta2  - si2 * Ialpha2;
  // Id_meas2_lp = lowpassId2->process( Id_meas2 );
  // Iq_meas2_lp = lowpassIq2->process( Iq_meas2 );


  motor.state1.P_tot = 1.5 * ( motor.state1.Vq * motor.state1.Iq_meas_lp + motor.state1.Vd * motor.state1.Id_meas_lp);
  motor.state1.I_bus = motor.state1.P_tot / motor.state.sensBus_lp;

  // HFI
  if ( motor.hfi1.hfi_on ) {
    motor.hfi1.hfi_V_act = motor.hfi1.hfi_V;
    if (motor.hfi1.hfi_firstcycle) {
      motor.hfi1.hfi_V_act /= 2;
      motor.hfi1.hfi_firstcycle = false;
    }
    if (motor.hfi1.hfi_V != motor.hfi1.hfi_prev) {
      motor.hfi1.hfi_V_act = motor.hfi1.hfi_prev + (motor.hfi1.hfi_V - motor.hfi1.hfi_prev) / 2;
    }
    if (motor.state.is_v7) {
      motor.hfi1.hfi_Id_meas_high = motor.state1.Id_meas;
      motor.hfi1.hfi_Iq_meas_high = motor.state1.Iq_meas;
    }
    else {
      motor.hfi1.hfi_V_act = -motor.hfi1.hfi_V_act;
      motor.hfi1.hfi_Id_meas_low = motor.state1.Id_meas;
      motor.hfi1.hfi_Iq_meas_low = motor.state1.Iq_meas;
    }
    motor.hfi1.delta_id = motor.hfi1.hfi_Id_meas_high - motor.hfi1.hfi_Id_meas_low;
    motor.hfi1.delta_iq = motor.hfi1.hfi_Iq_meas_high - motor.hfi1.hfi_Iq_meas_low;

    if ( motor.hfi1.diq_compensation_on) {
      //motor.hfi1.delta_iq -= motor.hfi1.diq_compensation[ int(motor.state1.thetaPark * 180 / 2 / M_PI) ]; //Note: not adjusted yet for changing hfi_V
      motor.hfi1.compensation = motor.hfi1.diq_compensation[ int(motor.state1.thetaPark * 360 / 2 / M_PI) ];
    }
    else {
      motor.hfi1.compensation = 0;
    }

    //motor.hfi1.hfi_curangleest = 0.25f * atan2( -motor.hfi1.delta_iq  , motor.hfi1.delta_id - 0.5 * motor.hfi1.hfi_V * motor.conf.T * ( 1 / Ld + 1 / Lq ) ); //Complete calculation (not needed because error is always small due to feedback). 0.25 comes from 0.5 because delta signals are used and 0.5 due to 2theta (not just theta) being in the sin and cos wave.
    if (motor.hfi1.hfi_method == 1 || motor.hfi1.hfi_method == 3 ) {
      motor.hfi1.hfi_curangleest =  0.5f * motor.hfi1.delta_iq / (motor.hfi1.hfi_V * motor.conf.T * ( 1 / motor.conf1.Lq - 1 / motor.conf1.Ld ) ); //0.5 because delta_iq is twice the iq value
    }
    else if (motor.hfi1.hfi_method == 2 || motor.hfi1.hfi_method == 4) {
      if (motor.state.is_v7) {
        motor.hfi1.hfi_curangleest =  (motor.state1.Iq_meas - motor.state1.Iq_SP) / (motor.hfi1.hfi_V * motor.conf.T * ( 1 / motor.conf1.Lq - 1 / motor.conf1.Ld ) );
      }
      else {
        motor.hfi1.hfi_curangleest =  (motor.state1.Iq_meas - motor.state1.Iq_SP) / (-motor.hfi1.hfi_V * motor.conf.T * ( 1 / motor.conf1.Lq - 1 / motor.conf1.Ld ) );
      }
    }
    motor.hfi1.hfi_error = -motor.hfi1.hfi_curangleest; //Negative feedback
    if (motor.hfi1.hfi_use_lowpass) {
      motor.hfi1.hfi_error = hfi_lowpass->process( motor.hfi1.hfi_error );
    }
    motor.hfi1.hfi_dir_int += motor.conf.T * motor.hfi1.hfi_error * motor.hfi1.hfi_gain_int2; //This the the double integrator

    float hfi_half_int = motor.hfi1.hfi_gain * 0.5f * motor.conf.T * motor.hfi1.hfi_error;
    motor.hfi1.hfi_contout += hfi_half_int + motor.hfi1.hfi_half_int_prev + motor.hfi1.hfi_dir_int; //This is the integrator and the double integrator
    if (motor.hfi1.hfi_method == 3 || motor.hfi1.hfi_method == 4) {
      motor.hfi1.hfi_ffw = motor.state1.we * motor.conf.T;
      motor.hfi1.hfi_contout += motor.hfi1.hfi_ffw; //This is the feedforward
    }
    while ( motor.hfi1.hfi_contout >= 2 * M_PI) {
      motor.hfi1.hfi_contout -= 2 * M_PI;
    }
    while ( motor.hfi1.hfi_contout < 0) {
      motor.hfi1.hfi_contout += 2 * M_PI;
    }
    while ( motor.hfi1.hfi_contout >= 2 * M_PI) {
      motor.hfi1.hfi_contout -= 2 * M_PI;
    }
    while ( motor.hfi1.hfi_contout < 0) {
      motor.hfi1.hfi_contout += 2 * M_PI;
    }

    motor.hfi1.hfi_dir = motor.hfi1.hfi_contout + motor.state1.dist * motor.hfi1.hfi_distgain;

    while ( motor.hfi1.hfi_dir >= 2 * M_PI) {
      motor.hfi1.hfi_dir -= 2 * M_PI;
    }
    while ( motor.hfi1.hfi_dir < 0) {
      motor.hfi1.hfi_dir += 2 * M_PI;
    }
    while ( motor.hfi1.hfi_dir_int >= 2 * M_PI) {
      motor.hfi1.hfi_dir_int -= 2 * M_PI;
    }
    while ( motor.hfi1.hfi_dir_int < 0) {
      motor.hfi1.hfi_dir_int += 2 * M_PI;
    }
    motor.hfi1.hfi_half_int_prev = hfi_half_int;
  }
  else {
    motor.hfi1.hfi_dir = motor.state1.thetaPark_obs;
    motor.hfi1.hfi_contout = motor.state1.thetaPark_obs;
    motor.hfi1.hfi_dir_int = 0;
    motor.hfi1.hfi_half_int_prev = 0;
    motor.hfi1.hfi_firstcycle = true;
    motor.hfi1.hfi_Id_meas_low = 0;
    motor.hfi1.hfi_Iq_meas_low = 0;
    motor.hfi1.hfi_Id_meas_high = 0;
    motor.hfi1.hfi_Iq_meas_high = 0;
    motor.hfi1.hfi_V_act = 0;
  }
  motor.hfi1.hfi_prev = motor.hfi1.hfi_V;

  if (motor.conf1.ridethewave != 1 ) {
    if (motor.state1.OutputOn == true) {
      motor.state1.Id_e = motor.state1.Id_SP - motor.state1.Id_meas;
      motor.state1.Iq_e = motor.state1.Iq_SP - motor.state1.Iq_meas;
    }
    else {
      motor.state1.Id_e = 0;
      motor.state1.Iq_e = 0;
    }

    motor.state1.Vq = motor.conf1.Kp_iq * motor.state1.Iq_e;
    float vq_half_int = motor.conf1.Ki_iq * motor.conf.T * 0.5f * motor.state1.Vq;
    motor.state1.vq_int_state += vq_half_int;
    motor.state1.Vq += motor.state1.vq_int_state;
    motor.state1.vq_int_state += vq_half_int;

    //Additional Vq
    motor.state1.Vq += motor.state1.VSP;
    motor.state1.Vq += motor.state1.dist * motor.state1.Vq_distgain;
    motor.state1.Vq += motor.hfi1.hfi_V_act * motor.hfi1.compensation;

    motor.state1.Vd = motor.conf1.Kp_id * motor.state1.Id_e;
    float vd_half_int = motor.conf1.Ki_id * motor.conf.T * 0.5f * motor.state1.Vd;
    motor.state1.vd_int_state += vd_half_int;
    motor.state1.Vd += motor.state1.vd_int_state;
    motor.state1.vd_int_state += vd_half_int;

    //Additional Vd
    motor.state1.Vd += motor.state1.dist * motor.state1.Vd_distgain;
    motor.state1.Vd += motor.hfi1.hfi_V_act;

    // PMSM decoupling control and BEMF FF
    motor.state1.VqFF = motor.state1.we * ( motor.conf1.Ld * motor.state1.Id_meas + motor.conf1.Lambda_m);

    // q axis induction FFW based on setpoint FFW
    motor.state1.VqFF += SPprofile->jref * motor.state1.Jload * motor.conf1.Lq * motor.conf1.Kt_Nm_Apeak * motor.state1.OutputOn;

    motor.state1.Vq += motor.state1.VqFF;

    motor.state1.VdFF = -motor.state1.we * motor.conf1.Lq * motor.state1.Iq_meas;
    motor.state1.Vd += motor.state1.VdFF;
  }

  motor.state1.Vq += motor.state1.muziek_gain_V * muziek[ (motor.state.curloop / (50 / (int)motor.conf.Ts)) % (sizeof(muziek) / 4) ];

  // Voltage clipping
  motor.state1.maxVolt = motor.conf1.maxDutyCycle * motor.state.sensBus_lp * one_by_sqrt3;
  motor.state1.Vtot = NORM2_f( motor.state1.Vd , motor.state1.Vq );
  if ( motor.state1.Vtot > motor.state1.maxVolt) {
    if ( motor.conf1.clipMethod == 0 ) {
      if ( abs( motor.state1.Vd ) > motor.state1.maxVolt) {
        if (motor.state1.Vd > 0) {
          motor.state1.Vd = motor.state1.maxVolt;
        }
        else {
          motor.state1.Vd = -motor.state1.maxVolt;
        }
        if (abs(motor.state1.vd_int_state) > abs(motor.state1.Vd)) {
          motor.state1.vd_int_state = motor.state1.Vd;
        }
      }
      if (sq(motor.state1.Vd) >= sq(motor.state1.maxVolt)) { //Vd cannot be larger than maxvolt, so no issue with sqrt of negative values. Still get nan Vq somehow. Fix:
        motor.state1.Vq = 0;
      }
      else {
        if ( motor.state1.Vq > 0 ) {
          motor.state1.Vq = sqrt(sq(motor.state1.maxVolt) - sq(motor.state1.Vd)) ;
        }
        else {
          motor.state1.Vq = -sqrt(sq(motor.state1.maxVolt) - sq(motor.state1.Vd)) ;
        }
      }
      if (abs(motor.state1.vq_int_state) > abs(motor.state1.Vq)) {
        motor.state1.vq_int_state = motor.state1.Vq;
      }
    }
    else if ( motor.conf1.clipMethod == 1 ) {
      motor.state1.Vd *= (motor.state1.maxVolt / motor.state1.Vtot);
      motor.state1.Vq *= (motor.state1.maxVolt / motor.state1.Vtot);

      if (abs(motor.state1.vd_int_state) > abs(motor.state1.Vd)) {
        motor.state1.vd_int_state = motor.state1.Vd;
      }
      if (abs(motor.state1.vq_int_state) > abs(motor.state1.Vq)) {
        motor.state1.vq_int_state = motor.state1.Vq;
      }
    }
  }





  // Inverse park transform
  motor.state1.Valpha = motor.state1.co * motor.state1.Vd - motor.state1.si * motor.state1.Vq;
  motor.state1.Vbeta  = motor.state1.co * motor.state1.Vq + motor.state1.si * motor.state1.Vd;

  motor.state1.Valpha += motor.state1.Valpha_offset;
  motor.state1.Vbeta += motor.state1.Vbeta_offset;

  if (motor.state1.Valpha > motor.state1.maxVolt) {
    motor.state1.Valpha = motor.state1.maxVolt;
  }
  if (motor.state1.Vbeta > motor.state1.maxVolt) {
    motor.state1.Vbeta = motor.state1.maxVolt;
  }
  if (motor.state1.Valpha < -motor.state1.maxVolt) {
    motor.state1.Valpha = -motor.state1.maxVolt;
  }
  if (motor.state1.Vbeta < -motor.state1.maxVolt) {
    motor.state1.Vbeta = -motor.state1.maxVolt;
  }

  // Inverse Power-variant Clarke transform
  motor.state1.Va = motor.state1.Valpha;
  motor.state1.Vb = -0.5 * motor.state1.Valpha + sqrt3_by_2 * motor.state1.Vbeta;
  motor.state1.Vc = -0.5 * motor.state1.Valpha - sqrt3_by_2 * motor.state1.Vbeta;

  //See https://microchipdeveloper.com/mct5001:start Zero Sequence Modulation Tutorial
  float Vcm = -(max(max(motor.state1.Va, motor.state1.Vb), motor.state1.Vc) + min(min(motor.state1.Va, motor.state1.Vb), motor.state1.Vc)) / 2;
  motor.state1.Va += Vcm + motor.state.sensBus_lp / 2;
  motor.state1.Vb += Vcm + motor.state.sensBus_lp / 2;
  motor.state1.Vc += Vcm + motor.state.sensBus_lp / 2;

  //Calculate modulation times
  motor.state1.tA = motor.state1.Va / motor.state.sensBus_lp;
  motor.state1.tB = motor.state1.Vb / motor.state.sensBus_lp;
  motor.state1.tC = motor.state1.Vc / motor.state.sensBus_lp;

  /*Motor 2 (needs to be updated, can probably be done much nicer):
    if (ridethewave2 != 1 ) {
    if (motor.state1.OutputOn == true) {
      Id_e2 = Id_SP2 - Id_meas2;
      Iq_e2 = motor.state2.Iq_SP - Iq_meas2;
    }
    else {
      Id_e2 = 0;
      Iq_e2 = 0;
    }
    Vq2 = Icontgain2 * Iq_e2;
    Vq2 += integrator_Iq2->processclip( Vq2 , -V_Bus - Vq2 , V_Bus - Vq2 );

    //Additional Vq
    Vq2 += VSP;
    Vq2 += motor.state1.dist * motor.state1.Vq_distgain;

    Vd2 = Icontgain2 * Id_e2;
    Vd2 += integrator_Id2->processclip( Vd2 , -V_Bus - Vd2 , V_Bus - Vd2 );

    //Additional Vd
    Vd2 += motor.state1.dist * motor.state1.Vd_distgain;

    we2 = motor.state2.vel * 8;  //Electrical speed [rad/s], based on setpoint

    // PMSM decoupling control and BEMF FF
    VqFF2 = we2 * ( Ld2 * Id_meas2 + Lambda_m2);

    // q axis induction FFW based on setpoint FFW
    VqFF2 += SPprofile->jref * motor.state2.Jload * Lq2 * Kt_Nm_Apeak2 * motor.state1.OutputOn;

    Vq2 += VqFF2;
    VdFF2 = -we2 * Lq2 * Iq_meas2;
    Vd2 += VdFF2;
    }

    Vq2 += motor.state1.muziek_gain_V * muziek[ (motor.state.curloop / (50 / (int)motor.conf.Ts)) % (sizeof(muziek) / 4) ];

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
    Va2 += Vcm2 + motor.state.sensBus_lp / 2;
    Vb2 += Vcm2 + motor.state.sensBus_lp / 2;
    Vc2 += Vcm2 + motor.state.sensBus_lp / 2;

    //Calculate modulation times
    tA2 = Va2 / motor.state.sensBus_lp;
    tB2 = Vb2 / motor.state.sensBus_lp;
    tC2 = Vc2 / motor.state.sensBus_lp;
  */
}

void changePWM() {
  if (motor.state1.tA > 1) {
    motor.state1.tA = 1;
  }
  if (motor.state1.tB > 1) {
    motor.state1.tB = 1;
  }
  if (motor.state1.tC > 1) {
    motor.state1.tC = 1;
  }

  if (motor.state1.tA < 0) {
    motor.state1.tA = 0;
  }
  if (motor.state1.tB < 0) {
    motor.state1.tB = 0;
  }
  if (motor.state1.tC < 0) {
    motor.state1.tC = 0;
  }

  if ( (abs(motor.state1.Id_SP) > motor.conf1.I_max) || (abs(motor.state1.Iq_SP) > motor.conf1.I_max) ) {
    motor.state1.OutputOn = false;
    if (motor.state.firsterror == 0) {
      motor.state.firsterror = 3;
    }
  }
  if ( (abs(motor.state1.Id_meas) > motor.conf1.I_max) || (abs(motor.state1.Iq_meas) > motor.conf1.I_max)  ) {
    motor.state1.OutputOn = false;
    if (motor.state.firsterror == 0) {
      motor.state.firsterror = 4;
    }
  }

  //  if ( (abs(Id_SP2) > motor.conf1.I_max) || (abs(motor.state2.Iq_SP) > motor.conf1.I_max) ) {
  //    motor.state1.OutputOn = false;
  //    if (motor.state.firsterror == 0) {
  //      motor.state.firsterror = 23;
  //    }
  //  }
  //  if ( (abs(Id_meas2) > motor.conf1.I_max) || (abs(Iq_meas2) > motor.conf1.I_max)  ) {
  //    motor.state1.OutputOn = false;
  //    if (motor.state.firsterror == 0) {
  //      motor.state.firsterror = 24;
  //    }
  //  }



  //  //Motor 1, flexpwm4:
    FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_CLDOK( 7 );  //Enable changing of settings
    if (motor.state1.OutputOn == false) {
      digitalWrite( ENGATE , 0);
      digitalWrite( ENGATE2 , 0);
      FLEXPWM4_SM0VAL3 = FLEXPWM4_SM0VAL1 / 2;
      FLEXPWM4_SM1VAL3 = FLEXPWM4_SM1VAL1 / 2;
      FLEXPWM4_SM2VAL3 = FLEXPWM4_SM2VAL1 / 2;
    }
    else {
      // Set duty cycles. FTM3_MOD = 100% (1800 for current settings, 20 kHz).
      digitalWrite( ENGATE , 1);
      digitalWrite( ENGATE2 , 1);
      FLEXPWM4_SM0VAL3 = FLEXPWM4_SM0VAL1 * motor.state1.tB;
      FLEXPWM4_SM1VAL3 = FLEXPWM4_SM1VAL1 * motor.state1.tC;
      FLEXPWM4_SM2VAL3 = FLEXPWM4_SM2VAL1 * motor.state1.tA;
    }
    FLEXPWM4_SM0VAL2 = -FLEXPWM4_SM0VAL3;
    FLEXPWM4_SM1VAL2 = -FLEXPWM4_SM1VAL3;
    FLEXPWM4_SM2VAL2 = -FLEXPWM4_SM2VAL3;
    FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_LDOK( 7 ); //Activate settings


  //Motor 2, flexpwm2:
//  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK( 7 );  //Enable changing of settings
//  if (motor.state1.OutputOn == false) {
//    digitalWrite( ENGATE , 0);
//    digitalWrite( ENGATE2 , 0);
//    FLEXPWM2_SM0VAL3 = FLEXPWM2_SM0VAL1 / 2;
//    FLEXPWM2_SM1VAL3 = FLEXPWM2_SM1VAL1 / 2;
//    FLEXPWM2_SM2VAL3 = FLEXPWM2_SM2VAL1 / 2;
//  }
//  else {
//    // Set duty cycles. FTM3_MOD = 100% (1800 for current settings, 20 kHz).
//    digitalWrite( ENGATE , 1);
//    digitalWrite( ENGATE2 , 1);
//    FLEXPWM2_SM0VAL3 = FLEXPWM2_SM0VAL1 * motor.state1.tC;
//    FLEXPWM2_SM1VAL3 = FLEXPWM2_SM1VAL1 * motor.state1.tB;
//    FLEXPWM2_SM2VAL3 = FLEXPWM2_SM2VAL1 * motor.state1.tA;
//  }
//  FLEXPWM2_SM0VAL2 = -FLEXPWM2_SM0VAL3;
//  FLEXPWM2_SM1VAL2 = -FLEXPWM2_SM1VAL3;
//  FLEXPWM2_SM2VAL2 = -FLEXPWM2_SM2VAL3;
//  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK( 7 ); //Activate settings

  //  M1:
  //  FLEXPWM4 22 23 2
  //  {1, M(4, 0), 1, 1},  // FlexPWM4_0_A  22  // AD_B1_08    --> M1 Phase B
  //  {1, M(4, 1), 1, 1},  // FlexPWM4_1_A  23  // AD_B1_09    --> M1 Phase C
  //  {1, M(4, 2), 1, 1},  // FlexPWM4_2_A   2  // EMC_04      --> M1 Phase A
  //
  //  M2:
  //  FLEXPWM2 4 5 6
  //  {1, M(2, 0), 1, 1},  // FlexPWM2_0_A   4  // EMC_06      --> M2 Phase C
  //  {1, M(2, 1), 1, 1},  // FlexPWM2_1_A   5  // EMC_08      --> M2 Phase B
  //  {1, M(2, 2), 1, 2},  // FlexPWM2_2_A   6  // B0_10       --> M2 Phase A

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

    case 'C':
      {
        integrator = new Integrator( motor.conf1.fInt , 2 * f_pwm);
        leadlag       = new LeadLag( motor.conf1.fBW , motor.conf1.alpha1 , motor.conf1.alpha2 , 2 * f_pwm);
        lowpass        = new Biquad( bq_type_lowpass , motor.conf1.fLP , 0.7, 2 * f_pwm);
        //        integrator2 = new Integrator( fInt2 , 2 * f_pwm);
        //        leadlag2       = new LeadLag( fBW2 , alpha1_2 , alpha2_2 , 2 * f_pwm);
        //        lowpass2        = new Biquad( bq_type_lowpass , fLP2 , 0.7 , 2 * f_pwm);
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
        Serial.readBytes( (char*)&SPprofile->t1 , 4);
        Serial.readBytes( (char*)&SPprofile->t2 , 4);
        Serial.readBytes( (char*)&SPprofile->t3 , 4);
        Serial.readBytes( (char*)&SPprofile->p , 4);
        Serial.readBytes( (char*)&SPprofile->v_max , 4);
        Serial.readBytes( (char*)&SPprofile->a_max , 4);
        Serial.readBytes( (char*)&SPprofile->j_max , 4);
        SPprofile->init();
        break;
      }
  }
}


/*
  if (Serial.available() > 4) {
  char settingByte = Serial.read();
  for ( int i = 0; i < 4; i++) {
    ser_in.bin[i] = Serial.read();
  }
  if (settingByte == 's') {
    motor.conf1.ss_gain = ser_in.fp;
    motor.state1.ss_f = ss_fstart;
    motor.state1.ss_phase = 0;
    motor.state1.ss_tstart = (motor.state.curtime + Ts) / 1e6; //timePrev gebruik ik niet meer?!!
  }
  if (settingByte == 'o') {
    digitalWrite( ENGATE , 1);
    SPI_init();
    motor.state1.OutputOn = true;
    motor.state1.offsetVelTot = 0;
    //      motor.state1.encoderPos1 = 0;
    //      motor.state1.encoderPos2 = 0;
    // Reset positions to zero, as the floating point number is most accurate here
    SPprofile->REFqmem = 0;
    if (motor.conf1.haptic == 1) {
      rmechoffset = motor.state1.ymech + motor.state2.ymech;
    }
    else {
      rmechoffset = motor.state1.ymech;
    }
    rmechoffset2 = motor.state2.ymech;

    integrator->setState(0);
    motor.state1.vq_int_state = 0;
    motor.state1.vd_int_state = 0;

    integrator2->setState(0);
    integrator_Id2->setState(0);
    integrator_Iq2->setState(0);

    motor.state.firsterror = 0;
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
    integrator = new Integrator( fInt , 2 * f_pwm);
    leadlag       = new LeadLag( fBW , alpha1 , alpha2 , 2 * f_pwm);
    lowpass        = new Biquad( bq_type_lowpass , fLP , 0.7, 2 * f_pwm);
    integrator2 = new Integrator( fInt2 , 2 * f_pwm);
    leadlag2       = new LeadLag( fBW2 , alpha1_2 , alpha2_2 , 2 * f_pwm);
    lowpass2        = new Biquad( bq_type_lowpass , fLP2 , 0.7 , 2 * f_pwm);
  }
  }
*/

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
