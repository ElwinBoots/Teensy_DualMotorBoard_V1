//
//  Biquad.cpp
//
//  Created by Nigel Redmon on 11/24/12
//  EarLevel Engineering: earlevel.com
//  Copyright 2012 Nigel Redmon
//
//  For a complete explanation of the Biquad code:
//  http://www.earlevel.com/main/2012/11/26/biquad-c-source-code/
//
//  License:
//
//  This source code is provided as is, without warranty.
//  You may copy and distribute verbatim copies of this document.
//  You may modify and use this source code to create binary code
//  for your own purposes, free or commercial.
//



// First, given a biquad transfer function defined as:

//         b0 + b1*z^-1 + b2*z^-2
// H(z) = ------------------------                                  (Eq 1)
//         a0 + a1*z^-1 + a2*z^-2

// This shows 6 coefficients instead of 5 so, depending on your architechture,
// you will likely normalize a0 to be 1 and perhaps also b0 to 1 (and collect
// that into an overall gain coefficient).  Then your transfer function would
// look like:

// (b0/a0) + (b1/a0)*z^-1 + (b2/a0)*z^-2
// H(z) = ---------------------------------------                   (Eq 2)
// 1 + (a1/a0)*z^-1 + (a2/a0)*z^-2

// or

// 1 + (b1/b0)*z^-1 + (b2/b0)*z^-2
// H(z) = (b0/a0) * ---------------------------------               (Eq 3)
// 1 + (a1/a0)*z^-1 + (a2/a0)*z^-2


// The most straight forward implementation would be the "Direct Form 1"
// (Eq 2):

// y[n] = (b0/a0)*x[n] + (b1/a0)*x[n-1] + (b2/a0)*x[n-2]
// - (a1/a0)*y[n-1] - (a2/a0)*y[n-2]            (Eq 4)

// This is probably both the best and the easiest method to implement in the
// 56K and other fixed-point or floating-point architechtures with a float
// wide accumulator.



#include <math.h>
#include "Biquad.h"

Biquad::Biquad() {
  type = bq_type_lowpass;
  b0 = 1.0;
  a1 = a2 = b1 = b2 = 0.0;
  f0 = 10.0;
  damp = 0.707;
  fs = 0.0;
  z1 = z2 = 0.0;
}

Biquad::Biquad(int type, float f0, float damp, float fs) {
  setBiquad(type, f0, damp, fs);
  z1 = z2 = 0.0;
}

Biquad::Biquad(int type, float f0, float debthdb, float notch_width, float fs) {
  if (debthdb == 0) {
    b0 = 1;
    b1 = 0;
    b2 = 0;
    //a0 = 1;
    a1 = 0;
    a2 = 0;
    z1 = z2 = 0.0;
  }
  else {
    float w0 = 2 * M_PI * f0 / fs;
    float alpha;
    float alpha1;
    if (debthdb < 0) {
      alpha = notch_width * sin(w0);
      alpha1 = alpha * pow ( 10 , debthdb / 20);
    }
    else {
      alpha = notch_width * sin(w0) * pow ( 10 , -debthdb / 20);
      alpha1 = notch_width * sin(w0);
    }

    b0 =   (1 + alpha1) / (1 + alpha);
    b1 =  -2 * cos(w0) / (1 + alpha);
    b2 =   (1 - alpha1) / (1 + alpha);
    //a0 =   1;
    a1 =  -2 * cos(w0) / (1 + alpha);
    a2 =   (1 - alpha) / (1 + alpha);
    z1 = z2 = 0.0;
  }
}

Biquad::~Biquad() {
}

void Biquad::setBiquad(int type, float f0, float damp, float fs) {
  this->type = type;
  this->f0 = f0;
  this->damp = damp;
  this->fs = fs;
  if (f0 == 0) {
    b0 = 1;
    b1 = 0;
    b2 = 0;
    //a0 = 1;
    a1 = 0;
    a2 = 0;
    InitStates();
  }
  else {
    calcBiquad();
  }
}

void Biquad::setNotch( float f0, float debthdb, float notch_width, float fs) {
  if (debthdb == 0) {
    b0 = 1;
    b1 = 0;
    b2 = 0;
    //a0 = 1;
    a1 = 0;
    a2 = 0;
    InitStates();
  }
  else {

    this->type = bq_type_notch;
    this->f0 = f0;
    this->fs = fs;

    float w0 = 2 * M_PI * f0 / fs;
    float alpha;
    float alpha1;
    if (debthdb < 0) {
      alpha = notch_width * sin(w0);
      alpha1 = alpha * pow ( 10 , debthdb / 20);
    }
    else {
      alpha = notch_width * sin(w0) * pow ( 10 , -debthdb / 20);
      alpha1 = notch_width * sin(w0);
    }

    b0 =   (1 + alpha1) / (1 + alpha);
    b1 =  -2 * cos(w0) / (1 + alpha);
    b2 =   (1 - alpha1) / (1 + alpha);
    //a0 =   1;
    a1 =  -2 * cos(w0) / (1 + alpha);
    a2 =   (1 - alpha) / (1 + alpha);
    InitStates();
  }
}




void Biquad::InitStates( ) {
  z1 = in * (1 - b0);
  z2 = in * (b2 - a2);
}


void Biquad::calcBiquad(void) {
  float w0 = 2 * M_PI * f0 / fs;
  float alpha = sin(w0) * damp;
  switch (this->type) {
    case bq_type_lowpass:

      b0 = 0.5 * (1 - cos(w0)) / (1 + alpha) ;
      b1 = 2 * b0;
      b2 = b0 ;

      //a0 = 1
      a1 = (-2 * cos(w0)) / (1 + alpha);
      a2 = (1 - alpha) / (1 + alpha);
      break;

    case bq_type_highpass:

      b0 =   (1 + cos(w0)) / 2 / (1 + alpha);
      b1 =   -2 * b0;
      b2 =   b0;
      //a0 =   1;
      a1 =  -2 * cos(w0) / (1 + alpha);
      a2 =   (1 - alpha) / (1 + alpha);

      break;

    case bq_type_notch:



      break;

  }
  InitStates();
  return;
}
