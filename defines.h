#define f_pwm 30e3
#define n_trace 14
const float Ts = 1e6/(2*f_pwm); //Ts in microseconds

#define NORM2_f(x,y)    (sqrtf(sq(x) + sq(y)))
#define UTILS_IS_INF(x)    ((x) == (1.0 / 0.0) || (x) == (-1.0 / 0.0))
#define UTILS_IS_NAN(x)   ((x) != (x))
#define UTILS_NAN_ZERO(x) (x = UTILS_IS_NAN(x) ? 0.0 : x)

#define chopperpin 33 //Digital output for chopper resistor
#define debugpin 32
#define engate 34
//#define PRREG(x) Serial.print(#x" 0x"); Serial.println(x,HEX)
#define SSpin 35


int anglechoice = 0;
float advancefactor;

float i_vector_radpers;
float i_vector_radpers_act;
float i_vector_acc;

bool SPdir = 1;

bool is_v7;

int tracearray[] = {1, 2, 3, 4, 5, 6, 7, 8, 9 , 10 , 1, 2, 3, 4, 5, 6, 7, 8, 9 , 10, 1, 2, 3, 4, 5, 6, 7, 8, 9 , 10};

bool haptic = 0;
bool revercommutation1 = false;

unsigned int ridethewave = 0;
unsigned int ridethewave2 = 0;
unsigned int sendall = 0;

float maxDutyCycle = 0.99;

float BEMFa;
float BEMFb;
float Ialpha_last;
float Ibeta_last;

float commutationoffset = 0;
float DQdisturbangle = 0;
float Vq = 0;
float Vd = 0;
float Valpha = 0;
float Vbeta = 0;
float thetawave = 0 ;
float Id_meas;
float Iq_meas;
float Id_meas_lp;
float Iq_meas_lp;
float VSP = 0;

float commutationoffset2 = 0;
float DQdisturbangle2 = 0;
float Vq2 = 0;
float Vd2 = 0;
float Valpha2 = 0;
float Vbeta2 = 0;
float thetawave2 = 0 ;
float Id_meas2;
float Iq_meas2;
float Id_meas2_lp;
float Iq_meas2_lp;

float Va = 0;
float Vb = 0;
float Vc = 0;

float Va2 = 0;
float Vb2 = 0;
float Vc2 = 0;

const float adc2A1 = 1 / 0.09;  //With linear hal current sensors ACS711 31A: These allow for 2 measurements per PWM cycle
const float adc2A2 = 1 / 0.09;  //With linear hal current sensors ACS711 15.5A: These allow for 2 measurements per PWM cycle
//cons/t fl/oat adc2A = -12.22; //With on driver board: Only 1 measurement per PWM cycle (low side current shunts) 

binaryFloat bf;
binaryFloat ser_in;

const float one_by_sqrt3 = 0.57735026919;
const float two_by_sqrt3 = 1.15470053838;
const float sqrt_two_three = 0.81649658092;
const float sqrt3_by_2 = 0.86602540378;

int timeremain;

float mechcontout = 0;
float Iout = 0;

float mechcontout2 = 0;
float Iout2 = 0;

float muziek_gain = 0;
float muziek_gain_V = 0;

float distval = 0;
float distoff = 0;
unsigned int curloop = 0;

unsigned int Ndownsample = 1;
unsigned int downsample = 1;

unsigned int Novervolt;
unsigned int Novervolt2;

// prbs
unsigned int NdownsamplePRBS = 1;
unsigned int downsamplePRBS  = 1;
uint16_t lfsr = 0xACE1u; /* Any nonzero start state will work. */
uint16_t noisebit;       /* Must be 16bit to allow bit<<15 later in the code */
//

// SS
float ss_phase = 0;
float ss_fstart = 1730;
float ss_fstep = 0.2;
float ss_fend = 1830;
unsigned int ss_n_aver = 200;
float ss_gain = 0;
float ss_offset = 0;
float ss_f = ss_fstart;
float ss_tstart;
float ss_out = 0;
//

int spNgo;
int REFstatus;

int incomingByte;

int encoderPos1 = 0;
int encoderPos2 = 0;

unsigned int IndexFound1 = 0;
unsigned int IndexFound2 = 0;

bool OutputOn = true;

unsigned int Nsend = 0;
unsigned int timePrev = 0;
unsigned int curtime = 0;
unsigned int overloadcount = 0;
const float T = Ts / 1e6; //Ts in seconds

const int enccountperrev = 20000;
const float enc2rad = 2 * M_PI / enccountperrev;

const int enccountperrev2 = 8192;
const float enc2rad2 = 2 * M_PI / enccountperrev2;

float I_max = 15; //Max current (peak of the sine wave in a phase)
float V_Bus = 24; //Bus Voltage

float rmech;
float rdelay; //Delays the reference relative to the feedforward signal. e.g. rdelay of 0.5 gives 0.5 Ts delay.
float emech1;
float ymech1;

float rmech2;
float emech2;
float ymech2;
float rmechoffset;
float rmechoffset2;

Biquad *lowpass_sensbus   = new Biquad( bq_type_lowpass , 500 , 0.7, 1 / T);
float sensBus_lp;

// Controller 1
float Kp = 0;
float fBW = 50.0;
float alpha1 = 3.0;
float alpha2 = 4.0;
float fInt = fBW / 6.0;
float fLP = fBW * 6.0;

Integrator *integrator = new Integrator( fInt , 1 / T);
LeadLag *leadlag       = new LeadLag( fBW , alpha1 , alpha2 , 1 / T);
Biquad *lowpass        = new Biquad( bq_type_lowpass , fLP , 0.7, 1 / T);

Biquad *lowpass_eradpers   = new Biquad( bq_type_lowpass , 50 , 0.7, 1 / T);

//Biquad *notch          = new Biquad( bq_type_notch , 2315.0, -20.0, 0.1 , 1 / T );

// Controller 2
float Kp2 = 0;
float fBW2 = 50.0;
float alpha1_2 = 3.0;
float alpha2_2 = 4.0;
float fInt2 = fBW2 / 6.0;
float fLP2 = fBW2 * 6.0;
Integrator *integrator2 = new Integrator( fInt2 , 1 / T);
LeadLag *leadlag2       = new LeadLag( fBW2 , alpha1_2 , alpha2_2 , 1 / T);
Biquad *lowpass2        = new Biquad( bq_type_lowpass , fLP2 , 0.7 , 1 / T);

//Current lowpass (now used at sensor level, maybe better at id,iq level?). Doesn't seem to matter much.
Biquad *lowpassIsens1  = new Biquad( bq_type_lowpass , 10e3 , 0.7, 1 / T);
Biquad *lowpassIsens2  = new Biquad( bq_type_lowpass , 10e3 , 0.7, 1 / T);
Biquad *lowpassIsens3  = new Biquad( bq_type_lowpass , 10e3 , 0.7, 1 / T);
Biquad *lowpassIsens4  = new Biquad( bq_type_lowpass , 10e3 , 0.7, 1 / T);

//These are now used for power and bus current estimates
Biquad *lowpassId1  = new Biquad( bq_type_lowpass , 50 , 0.7, 1 / T);
Biquad *lowpassId2  = new Biquad( bq_type_lowpass , 50 , 0.7, 1 / T);
Biquad *lowpassIq1  = new Biquad( bq_type_lowpass , 50 , 0.7, 1 / T);
Biquad *lowpassIq2  = new Biquad( bq_type_lowpass , 50 , 0.7, 1 / T);

unsigned int useIlowpass = 0;

float Vout;
float fIntCur = 700;

float Kp_iq = 0;
float Kp_id = 0;
float Ki_iq = 0; //Series PI controller. Ki = w0 of zero. Choose this to be R / Lq.
float Ki_id = 0; //Series PI controller. Ki = w0 of zero. Choose this to be R / Ld.
float vq_int_state;
float vd_int_state;

float Vout2;
float fIntCur2 = 700;
float Icontgain2 = 0;
Integrator *integrator_Id2 = new Integrator( fIntCur , 1 / T);
Integrator *integrator_Iq2 = new Integrator( fIntCur , 1 / T);


// For setpoint
Biquad *lowpassSP = new Biquad( bq_type_lowpass , 10 , 0.707, 1 / T);


float sensCalVal1;
float sensCalVal2;
float sensCalVal3;
float sensCalVal4;

float sens1 = 0; 
float sens2 = 0;
float sens3 = 0; 
float sens4 = 0;
float sens1_lp = 0; 
float sens2_lp = 0;
float sens3_lp = 0; 
float sens4_lp = 0;
float sens1_calib;
float sens2_calib;
float sens3_calib;
float sens4_calib;

float sensBus;
float I_bus; 
float P_tot;

float Busadc2Vbus = 1 / 4095.0 * 3.3 * ((68.3 + 5.05) / 5.05); //5.1 changed to 5.05 to improve accuracy. May differ board to board.
int n_senscalib;
bool setupready;

//fast 180 deg:
MotionProfile *SPprofile = new MotionProfile( 0 , 0.000500000000000000 , 0.0193000000000000 , 0 , 3.14159265358979 , 157.079632679490 , 7853.98163397448 , 15632147.3532855 , T );

int SP_input_status = 0;
int spGO = 0;
float Jload = 0;
float velFF = 0;
float R = 0;

float Jload2 = 0;
float velFF2 = 0;

float offsetVelTot = 0;
float offsetVel = 0;
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
float thetaPark_vesc;
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

float acc2;
float vel2;
float Ialpha2;
float Ibeta2;
float thetaPark2;
float co2;
float si2;
float D2;
float Q2;
float tA2;
float tB2;
float tC2;
float Id_e2;
float Id_SP2;
float Iq_e2;
float Iq_SP2;
float ia2;
float ib2;
float ic2;

float Vq_distgain;
float Vd_distgain;
float Iq_distgain;
float Id_distgain;
float mechdistgain;

float maxVolt;
float Vtot;

float max_edeltarad = 0.25f * M_PI;

unsigned int ContSelect = 1; 
unsigned int firsterror = 0;
float N_pp = 4; //Number of pole pairs 

//Motor parameters
float Kt_Nm_Arms = 0.103; //Motor Torque constant [Nm/Arms]
float Kt_Nm_Apeak = Kt_Nm_Arms/sqrt(2);
float we;
float Ld = 10e-3; //[Henry] Ld induction: phase-zero
float Lq = 10e-3; //[Henry] Lq induction: phase-zero
float Lambda_m = Kt_Nm_Arms / (sqrt(2) * 1.5 * N_pp ); //[Weber] Note: on the fly changes of Kt do not adjust this value!

float observer_gain = 1 / (Lambda_m * Lambda_m);
float x1;
float x2;

unsigned int N_pp2 = 8; //Number of pole pairs

float Kt_Nm_Arms2 = 0.0474; //Motor Torque constant [Nm/Arms]
float Kt_Nm_Apeak2 = Kt_Nm_Arms2/sqrt(2);
float we2;
float Ld2 = 0; //[Henry] Ld induction: phase-zero
float Lq2 = 0; //[Henry] Lq induction: phase-zero
float Lambda_m2 = Kt_Nm_Arms2 / (sqrt(2) * 1.5 * N_pp2 ); //[Weber] Note: on the fly changes of Kt do not adjust this value!

bool hfi_on = false;
float hfi_V = 0;
float hfi_V_act;
float hfi_dir = 0;
float hfi_dir_int;
float Valpha_offset_hfi;
float Vbeta_offset_hfi;
int hfi_cursample;
int hfi_maxsamples = 1e4;
float hfi_curtot;
float hfi_curorttot;
float hfi_curprev;
float hfi_curortprev;
float hfi_gain = 100 * 2 * M_PI;
float hfi_pgain = 0;
float hfi_curangleest;
float hfi_dir_int2;
float hfi_gain_int2;
float hfi_Id_meas_low = 0;
float hfi_Iq_meas_low = 0;
float hfi_Id_meas_high = 0;
float hfi_Iq_meas_high = 0;
float delta_id;
float delta_iq;
float hfi_advance_factor;
bool hfi_firstcycle = true;
float hfi_abs_pos;
bool hfi_useforfeedback = false;
float hfi_half_int_prev;
bool hfi_use_lowpass;
Biquad *hfi_lowpass = new Biquad( bq_type_lowpass , 2000 , 0.707, 1 / T);
float hfi_prev;
float hfi_distgain;
float hfi_contout;
float hfi_error;
unsigned int hfi_method = 1;
float hfi_ffw;
float hfi_maxvel = 10;

float VqFF;
float VdFF;

float VqFF2;
float VdFF2;

float Iq_offset_SP;
float Id_offset_SP;
float Id_offset_SP2;

float Valpha_offset;
float Vbeta_offset;
float Valpha2_offset;

Biquad *lowpass_ss_offset = new Biquad( bq_type_lowpass , 10 , 0.707, 1 / T);

//There are 4 hardware quadrature encoder channels available the Teensy 4.x. 
//The Teensy 4.1 Encoders are supported on pins: 0, 1, 2, 3, 4, 5, 7, 30, 31, 33, 36 and 37. 
//WARNING! Pins 0, 5 and 37 share the same internal crossbar connections and are as such exclusive...pick one or the other.
QuadEncoder Encoder1(1, 0, 1 , 0 , 3);   //Encoder 1 on pins 0 and 1, index on pin 3
QuadEncoder Encoder2(2, 30, 31 , 0 , 33);//Encoder 2 on pins 30 and 31, index on pin 33
