/*
  See https://nl.mathworks.com/help/mcb/ref/mtpacontrolreference.html for a PMSM model

  vd = id*R + dλd/dt − ωe * Lq * iq
  vq = iq*R + dλq/dt + ωe * Ld * id + ωe * λpm
  λd = Ld * id + λpm
  λq = Lq * iq
  Te = 1.5* N_pp * (λpm * iq+(Ld− Lq) * id * iq)

  vd is the d-axis voltage (Volts).
  vq is the q-axis voltage (Volts).
  id is the d-axis current (Amperes).
  iq is the q-axis current (Amperes).
  R is the stator phase winding resistance (Ohms).
  λpm is the permanent magnet flux linkage (Weber).
  λd is the d-axis flux linkage (Weber).
  λq is the q-axis flux linkage (Weber).
  ωe is the electrical speed corresponding to frequency of stator voltages (Radians/ sec).
  Ld is the d-axis winding inductance (Henry).
  Lq is the q-axis winding inductance (Henry).
  Te is the electromechanical torque produced by the PMSM (Nm).
  N_pp is the number of motor pole pairs.


  Te = 1.5* N_pp * (λpm * iq+(Ld− Lq) * id * iq)
  id = 0 -> Te = 1.5 * N_pp * λpm * iq [Nm]

  Kt_Nm_Apeak = 1.5 * N_pp * λpm                                            [Nm/Apeak]
  Kt_Nm_Arms = sqrt(2) * Kt_Nm_Apeak                                        [Nm/Arms]

  λpm = Kt_Nm_Apeak / (1.5 * N_pp) = Kt_Nm_Arms / (sqrt(2) * 1.5 * N_pp )   [Weber]

  Ke_phase_0_peak = N_pp * λpm
*/



/*
  With 1 motor:

  M:
  FLEXPWM2 4 5 6
  {1, M(2, 0), 1, 1},  // FlexPWM2_0_A   4  // EMC_06
  {1, M(2, 1), 1, 1},  // FlexPWM2_1_A   5  // EMC_08
  {1, M(2, 2), 1, 2},  // FlexPWM2_2_A   6  // B0_10

  Encoders:
  E1:
  0 1 3

  E2:
  30 31 33



  For two motors:

  M1:
  FLEXPWM4 22 23 2
  {1, M(4, 0), 1, 1},  // FlexPWM4_0_A  22  // AD_B1_08    --> M1 Phase B
  {1, M(4, 1), 1, 1},  // FlexPWM4_1_A  23  // AD_B1_09    --> M1 Phase C 
  {1, M(4, 2), 1, 1},  // FlexPWM4_2_A   2  // EMC_04      --> M1 Phase A

  M2:
  FLEXPWM2 4 5 6
  {1, M(2, 0), 1, 1},  // FlexPWM2_0_A   4  // EMC_06      --> M2 Phase C
  {1, M(2, 1), 1, 1},  // FlexPWM2_1_A   5  // EMC_08      --> M2 Phase B
  {1, M(2, 2), 1, 2},  // FlexPWM2_2_A   6  // B0_10       --> M2 Phase A

  Encoders:
  E1:
  0 1 3

  E2:
  30 31 33

  teensy 4.1 has 0..41 full size pins.

  Encoder possible on:
  0, 1, 2, 3, 4, 5, 7, 30, 31, 33, 36 and 37.
  WARNING! Pins 0, 5 and 37 share the same internal crossbar connections and are as such exclusive...pick one or the other.


//There are 4 hardware quadrature encoder channels available the Teensy 4.x.
//The Teensy 4.1 Encoders are supported on pins: 0, 1, 2, 3, 4, 5, 7, 30, 31, 33, 36 and 37.
//WARNING! Pins 0, 5 and 37 share the same internal crossbar connections and are as such exclusive...pick one or the other.
//Same thing applies to pins 1 / 36 and 5 / 37.




  ADC channel, pin numbers

    Note: 0 to 13 is same as 14 to 27. Do not try to use these for analog in!
        7,      // 0/A0  AD_B1_02   ENC1_A
        8,      // 1/A1  AD_B1_03   ENC1_B
        12,     // 2/A2  AD_B1_07   M1 out A INH-A
        11,     // 3/A3  AD_B1_06   ENC1_I
        6,      // 4/A4  AD_B1_01   M2 out C INH-C2
        5,      // 5/A5  AD_B1_00   M2 out B INH-B2
        15,     // 6/A6  AD_B1_10   M2 out A INH-A2
        0,      // 7/A7  AD_B1_11   M2 Ia              --> Orignial design --> Doesn't work
        13,     // 8/A8  AD_B1_08   M2 Ib              --> Orignial design --> Doesn't work
        14,     // 9/A9  AD_B1_09   M2 Ic              --> Orignial design --> Doesn't work
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

        1,      // 24/A10 AD_B0_12  M2 EMF-A  --> Exchanged for M2 Ia  // Analog channel 1 input 1 NOTE: only on ADC1
        3,      // 26/A12 AD_B1_14  M2 EMF-C  --> Exchanged for M2 Ib  // Analog channel 2 input 3 NOTE: only on ADC2

        11,     // 17/A3  AD_B1_06  M1 Vbus
        0,      // 21/A7  AD_B1_11  M2 Vbus
  */
