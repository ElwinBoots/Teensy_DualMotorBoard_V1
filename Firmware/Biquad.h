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

#ifndef Biquad_h
#define Biquad_h

enum {
  bq_type_lowpass = 0,
  bq_type_highpass,
  bq_type_notch,
};

class Biquad {
  public:
    Biquad();
    Biquad(int type, float f0, float damp, float fs);
    Biquad(int type, float f0, float debthdb, float notch_width, float fs);
    ~Biquad();
    void InitStates( );
    void setType(int type);
    void setdamp(float damp);
    void setf0(float f0);
    void setfs(float fs);
    void setBiquad(int type, float f0, float damp, float fs);
    void setNotch( float f0, float debthdb, float notch_width, float fs);
    float process(float in);



    void calcBiquad(void);

    int type;
    float b0, a1, a2, b1, b2;
    float f0, damp, fs;
    float z1, z2, in;
  protected:
};

inline float Biquad::process(float in) {
  this->in = in;
  float out = in * b0 + z1;
  z1 = in * b1 + z2 - a1 * out;
  z2 = in * b2 - a2 * out;
  return out;
}

#endif // Biquad_h
