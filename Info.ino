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
  FLEXPWM2 4 5 6
  {1, M(2, 0), 1, 1},  // FlexPWM2_0_A   4  // EMC_06
  {1, M(2, 1), 1, 1},  // FlexPWM2_1_A   5  // EMC_08
  {1, M(2, 2), 1, 2},  // FlexPWM2_2_A   6  // B0_10

  M2:
  FLEXPWM4 22 23 2
  {1, M(4, 0), 1, 1},  // FlexPWM4_0_A  22  // AD_B1_08
  {1, M(4, 1), 1, 1},  // FlexPWM4_1_A  23  // AD_B1_09
  {1, M(4, 2), 1, 1},  // FlexPWM4_2_A   2  // EMC_04

  Encoders:
  E1:
  0 1 3

  E2:
  30 31 33

  teensy 4.1 has 0..41 full size pins.

  Encoder possible on:
  0, 1, 2, 3, 4, 5, 7, 30, 31, 33, 36 and 37.
  WARNING! Pins 0, 5 and 37 share the same internal crossbar connections and are as such exclusive...pick one or the other.

  FlexPWM pins:
  {1, M(1, 1), 0, 4},  // FlexPWM1_1_X   0  // AD_B0_03
  {1, M(1, 0), 0, 4},  // FlexPWM1_0_X   1  // AD_B0_02
  {1, M(4, 2), 1, 1},  // FlexPWM4_2_A   2  // EMC_04
  {1, M(4, 2), 2, 1},  // FlexPWM4_2_B   3  // EMC_05
  {1, M(2, 0), 1, 1},  // FlexPWM2_0_A   4  // EMC_06
  {1, M(2, 1), 1, 1},  // FlexPWM2_1_A   5  // EMC_08
  {1, M(2, 2), 1, 2},  // FlexPWM2_2_A   6  // B0_10
  {1, M(1, 3), 2, 6},  // FlexPWM1_3_B   7  // B1_01
  {1, M(1, 3), 1, 6},  // FlexPWM1_3_A   8  // B1_00
  {1, M(2, 2), 2, 2},  // FlexPWM2_2_B   9  // B0_11
  {2, M(1, 0), 0, 1},  // QuadTimer1_0  10  // B0_00
  {2, M(1, 2), 0, 1},  // QuadTimer1_2  11  // B0_02
  {2, M(1, 1), 0, 1},  // QuadTimer1_1  12  // B0_01
  {2, M(2, 0), 0, 1},  // QuadTimer2_0  13  // B0_03
  {2, M(3, 2), 0, 1},  // QuadTimer3_2  14  // AD_B1_02
  {2, M(3, 3), 0, 1},  // QuadTimer3_3  15  // AD_B1_03
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {2, M(3, 1), 0, 1},  // QuadTimer3_1  18  // AD_B1_01
  {2, M(3, 0), 0, 1},  // QuadTimer3_0  19  // AD_B1_00
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {1, M(4, 0), 1, 1},  // FlexPWM4_0_A  22  // AD_B1_08
  {1, M(4, 1), 1, 1},  // FlexPWM4_1_A  23  // AD_B1_09
  {1, M(1, 2), 0, 4},  // FlexPWM1_2_X  24  // AD_B0_12
  {1, M(1, 3), 0, 4},  // FlexPWM1_3_X  25  // AD_B0_13
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {1, M(3, 1), 2, 1},  // FlexPWM3_1_B  28  // EMC_32
  {1, M(3, 1), 1, 1},  // FlexPWM3_1_A  29  // EMC_31
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {1, M(2, 0), 2, 1},  // FlexPWM2_0_B  33  // EMC_07
  #ifdef ARDUINO_TEENSY40
  {1, M(1, 1), 2, 1},  // FlexPWM1_1_B  34  // SD_B0_03
  {1, M(1, 1), 1, 1},  // FlexPWM1_1_A  35  // SD_B0_02
  {1, M(1, 0), 2, 1},  // FlexPWM1_0_B  36  // SD_B0_01
  {1, M(1, 0), 1, 1},  // FlexPWM1_0_A  37  // SD_B0_00
  {1, M(1, 2), 2, 1},  // FlexPWM1_2_B  38  // SD_B0_05
  {1, M(1, 2), 1, 1},  // FlexPWM1_2_A  39  // SD_B0_04
  #endif
  #ifdef ARDUINO_TEENSY41
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {1, M(2, 3), 1, 6},  // FlexPWM2_3_A  36  // B1_00
  {1, M(2, 3), 2, 6},  // FlexPWM2_3_B  37  // B1_01
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {1, M(1, 1), 2, 1},  // FlexPWM1_1_B  42  // SD_B0_03
  {1, M(1, 1), 1, 1},  // FlexPWM1_1_A  43  // SD_B0_02
  {1, M(1, 0), 2, 1},  // FlexPWM1_0_B  44  // SD_B0_01
  {1, M(1, 0), 1, 1},  // FlexPWM1_0_A  45  // SD_B0_00
  {1, M(1, 2), 2, 1},  // FlexPWM1_2_B  46  // SD_B0_05
  {1, M(1, 2), 1, 1},  // FlexPWM1_2_A  47  // SD_B0_04
  {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_0_B
  {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_2_A
  {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_2_B
  {1, M(3, 3), 2, 1},  // FlexPWM3_3_B  51  // EMC_22
  {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_1_B
  {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_1_A
  {1, M(3, 0), 1, 1},  // FlexPWM3_0_A  53  // EMC_29

*/
