#ifndef __MOTIONPROFILE2_H__
#define __MOTIONPROFILE2_H__

class MotionProfile{		
public:	
	MotionProfile(float tstart, float t1, float t2, float t3, float p, float v_max, float a_max, float j_max, float Ts);	 
	void init();
	float stateCalculation( int input_status);
	float p2p_get_b( double t );
	float qref, vref, aref, jref;

	long REFstatus; 
	bool REFidir;
	float p, v, v_max, a_max, j_max, tstart, t1, t2, t3, Ts, rdelay;
	float REFiref, REFqmem;
	double t, REFt0, REFt1, REFt2, REFt3, REFt4, REFt5, REFt6, REFt7, reltim;
	double REFs0, REFs1, REFs2, REFs3, REFs4, REFs5, REFs6, REFs7;
	double REFv0, REFv1, REFv2, REFv3, REFv4, REFv5, REFv6, REFv7;
	double REFa0, REFa1, REFa2, REFa3, REFa4, REFa5, REFa6, REFa7;
	double REFdelta, REFgamma, REFjerk, REFxstrt, j, jd;
	
protected:

	
	

};
#endif
