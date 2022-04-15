//
//  Biquad.h
//
//  Created by Nigel Redmon on 11/24/12
//  EarLevel Engineering: earlevel.com
//  Copyright 2012 Nigel Redmon
//
//  For a complete explanation of the Biquad code:
//  http://www.earlevel.com/main/2012/11/25/Biquad-c-source-code/
//
//  License:
//
//  This source code is provided as is, without warranty.
//  You may copy and distribute verbatim copies of this document.
//  You may modify and use this source code to create binary code
//  for your own purposes, free or commercial.
//

#ifndef ControlTools_h
#define ControlTools_h

class Integrator {
public:

    //Integrator();
    Integrator( float f0, float fs);
    //~Integrator();
    void setState( float z1 ) ;
    // void setdamp(float damp);
    // void setf0(float f0);
    // void setfs(float fs);
    // void setBiquad(int type, float f0, float damp, float fs);
    float process(float in);
    float processclip(float in , float cliplow , float cliphigh);
    

protected:
	float G;
    float f0, fs;
    float z1;
};

inline float Integrator::process(float in) {
	float A = G * in;
    float out = A + z1;
    z1 = out + A;
    return out;
}

inline float Integrator::processclip(float in , float cliplow , float cliphigh) {
    float A = G * in;
    float out = A + z1;
    z1 = out + A;
	if( out > cliphigh){
		out = cliphigh;
		z1 = out;
	}
	else if( out < cliplow ){
		out = cliplow;
		z1 = out;
	}
    return out;
}





// 

class LeadLag {
public:

    //Integrator();
    LeadLag( float fBW, float alpha1, float alpha2, float fs);
    //~Integrator();
    //void setState( float z1 ) ;
    // void setdamp(float damp);
    // void setf0(float f0);
    // void setfs(float fs);
    // void setBiquad(int type, float f0, float damp, float fs);
    float process(float in);

protected:
	float alpha1, alpha2;
    float a1, b0, b1;
    float fBW, fs;
    float z1;
};

inline float LeadLag::process(float in) {
    float out = b0 * in + z1;
    z1 = b1 * in -a1 * out;
    return out;
}


#endif  //ControlTools_h
