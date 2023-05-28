#include <stdbool.h>
#include <stdint.h>

#define NORM2_f(x,y)    (sqrtf(sq(x) + sq(y)))
#define UTILS_IS_INF(x)    ((x) == (1.0 / 0.0) || (x) == (-1.0 / 0.0))
#define UTILS_IS_NAN(x)   ((x) != (x))
#define UTILS_NAN_ZERO(x) (x = UTILS_IS_NAN(x) ? 0.0 : x)
#define PRREG(x) Serial.print(#x" 0x"); Serial.println(x,HEX)

#define ONE_BY_SQRT3    0.57735026919
#define TWO_BY_SQRT3    1.15470053838
#define SQRT_TWO_THREE  0.81649658092
#define SQRT3_by_2      0.86602540378

#define F_PWM 60e3
#define CHOPPERPIN 33 //Digital output for chopper resistor
#define DEBUGPIN 32
#define ENGATE  34
#define ENGATE2 39
#define SSPIN 35
#define SSPIN2 40 

// First order lowpass filter.
// c_lowpass = 1 - exp( -f_lowpass * 2 * pi * Ts );
// Where f_lowpass is the lowpass frequency in [Hz] and Ts is the sample time in [s]

#define LOWPASS( output, input, c_lowpass)  (output += (c_lowpass) * ((input) - (output)))


enum commands {
  NO_COMMAND = 0,
  UPDATE_CONTROLLER = 1
};

typedef struct setpoint_t {
  bool SPdir;
  int spNgo;
} setpoint_t;

typedef struct hfi_t {
  bool hfi_on;
  float hfi_V;
  float hfi_V_act;
  float hfi_dir;
  float hfi_dir_int;
  float hfi_gain;
  float hfi_curangleest;
  float hfi_gain_int2;
  float hfi_Id_meas_low;
  float hfi_Iq_meas_low;
  float hfi_Id_meas_high;
  float hfi_Iq_meas_high;
  float delta_id;
  float delta_iq;
  bool hfi_firstcycle;
  float hfi_abs_pos;
  bool hfi_useforfeedback;
  bool hfi_use_lowpass;
  float hfi_prev;
  float hfi_distgain;
  float hfi_contout;
  float hfi_error;
  unsigned int hfi_method;
  float hfi_ffw;
  float hfi_maxvel;
  float diq_compensation[361];
  bool diq_compensation_on;
  float compensation;
} hfi_t;

typedef struct mot_conf_t {
  int anglechoice;
  float advancefactor;
  bool haptic;
  bool reversecommutation;
  float maxDutyCycle;
  unsigned int ridethewave;
  float commutationoffset;
  float adc2A;
  unsigned int NdownsamplePRBS;
  float ss_fstart;
  float ss_fstep;
  float ss_fend;
  unsigned int ss_n_aver;
  float ss_gain;
  float ss_offset;
  unsigned int Ndownsample;
  int enccountperrev;
  float enc2rad;
  float I_max;
  float T_max;
  float Kp_iq;
  float Kp_id;
  float Ki_iq; //Series PI controller. Ki = w0. Choose this to be R / Lq.
  float Ki_id; //Series PI controller. Ki = w0. Choose this to be R / Ld.
  float max_edeltarad;
  float N_pp; //Number of pole pairs
  unsigned int clipMethod;
  float maxerror;
  unsigned int Command;
  
  //Motor parameters
  float Kt_Nm_Apeak;
  float Ld; //[Henry] Ld induction: phase-zero
  float Lq; //[Henry] Lq induction: phase-zero
  float Lambda_m; //[Weber] Note: on the fly changes of Kt do not adjust this value!
  float Kp;
  float Ki;
  float Kd;
  float lowpass_c;
  float Kp_prep;
  float Ki_prep;
  float Kd_prep;
  float lowpass_c_prep;
  float enc_transmission;
  
  float fBW;
  float alpha1;
  float alpha2;
  float fInt;
  float fLP;
} mot_conf_t;

typedef struct mot_state_t {
  float i_vector_radpers_act;
  float BEMFa;
  float BEMFb;
  float Ialpha_last;
  float Ibeta_last;
  float Vq;
  float Vd;
  float Valpha;
  float Vbeta;
  float thetawave ;
  float Id_meas;
  float Iq_meas;
  float Id_meas_lp;
  float Iq_meas_lp;
  float Va;
  float Vb;
  float Vc;
  float VSP;
  float i_vector_radpers;
  float i_vector_acc;
  float Kp_out;
  float Kp_out_prev;
  float Kd_out;
  float Ki_sum;
  float Ki_out;
  float lp_out;
  float mechcontout;
  float Iout;
  float muziek_gain;
  float muziek_gain_V;
  float distval;
  float distoff;
  unsigned int downsample;
  int REFstatus;
  int incomingByte;
  int encoderPos1;
  int encoderPos2;
  unsigned int IndexFound1;
  unsigned int IndexFound2;
  bool OutputOn;
  float ss_phase;
  uint16_t lfsr;
  float ss_out;
  uint16_t noisebit; /* Must be 16bit to allow bit<<15 later in the code */
  float ss_f;
  float ss_tstart;
  unsigned int downsamplePRBS;
  int SP_input_status;
  int spGO;
  float Jload;
  float velFF;
  float R;

  float Jload2;
  float velFF2;

  float offsetVelTot;
  float offsetVel;
  float offsetVel_lp;
  float acc;
  float vel;
  float dist;
  float Ialpha;
  float Ibeta;
  float thetaPark;
  float thetaParkPrev;
  float edeltarad;
  float eradpers_lp;
  float erpm;
  float thetaPark_enc;
  float thetaPark_obs;
  float thetaPark_obs_prev;
  float co;
  float si;
  float D;
  float Q;
  float tA;
  float tB;
  float tC;
  float Id_e;
  float Id_SP;
  float Iq_e;
  float Iq_SP;
  float ia;
  float ib;
  float ic;

  float Vq_distgain;
  float Vd_distgain;
  float Iq_distgain;
  float Id_distgain;
  float mechdistgain;
  float rdistgain;
  
  float maxVolt;
  float Vtot;
  float rmech;
  float rdelay; //Delays the reference relative to the feedforward signal. e.g. rdelay of 0.5 gives 0.5 Ts delay.
  float emech;
  float ymech;
  float rmechoffset;

  float we;
  float VqFF;
  float VdFF;
  float Iq_offset_SP;
  float Id_offset_SP;
  float Valpha_offset;
  float Vbeta_offset;
  float vq_int_state;
  float vd_int_state;
  float I_bus;
  float P_tot;
} mot_state_t;

typedef struct globalstate_t {
  float sens1;
  float sens2;
  float sens3;
  float sens4;
  float sens1_lp;
  float sens2_lp;
  float sens3_lp;
  float sens4_lp;
  float sens1_calib;
  float sens2_calib;
  float sens3_calib;
  float sens4_calib;
  float sensBus_lp;
  float sensCalVal1;
  float sensCalVal2;
  float sensCalVal3;
  float sensCalVal4;
  float sensBus;
  float sensBus2;
  int n_senscalib;
  bool setupready;
  unsigned int curtime;
  bool is_v7;
  unsigned int firsterror;
  unsigned int curloop;
  unsigned int downsample;
} globalstate_t;

typedef struct conf_t {
  float Ts; //Ts in microseconds
  float T; //Ts in seconds
  unsigned int useIlowpass;
  float Busadc2Vbus;
  float V_Bus;
  unsigned int Ndownsample;
} conf_t;

typedef struct motor_total_t {
  mot_conf_t conf1;
  mot_conf_t conf2;
  hfi_t hfi1;
  hfi_t hfi2;
  conf_t conf;
  globalstate_t state;
  mot_state_t state1;
  mot_state_t state2;
  setpoint_t setpoint;
} motor_total_t;

motor_total_t motor;
