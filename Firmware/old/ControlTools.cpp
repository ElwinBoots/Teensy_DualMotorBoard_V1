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

// b0 + b1*z^-1 + b2*z^-2
// H(z) = ------------------------                                  (Eq 1)
// a0 + a1*z^-1 + a2*z^-2

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
#include "ControlTools.h"

// Integrator::Integrator() {
// }

Integrator::Integrator( float f0, float fs) {
  G = M_PI * f0 / fs; // G = w_i * Ts / 2 = 2 * pi * f_i / fs /2 = pi * f_i / fs
  this->f0 = f0;
  this->fs = fs;
  z1 = 0.0;
}

// Integrator::~Integrator() {
// }

void Integrator::setState( float z1 ) {
	this->z1 = z1;
}

LeadLag::LeadLag( float fBW, float alpha1, float alpha2, float fs) {
  float alpha = sqrt( 1/ pow(alpha2,2) + 1 ) / sqrt( pow(alpha1,2) + 1 );
  float w0 = fBW * 2 * M_PI / alpha1; 
  float wp = fBW * 2 * M_PI * alpha2; 
  a1 = (wp -2*fs) / (wp + 2*fs);
  b0 = alpha * ( ( 1 + 2*fs/w0 ) / ( 1 + 2*fs/wp ));
  b1 = alpha * ( ( 1 - 2*fs/w0 ) / ( 1 + 2*fs/wp ));
  this->fBW = fBW;
  this->alpha1 = alpha1;
  this->alpha2 = alpha2;
  this->fs = fs;
  z1 = 0.0;
}

// void Biquad::setQ(float Q) {
// this->Q = Q;
// calcBiquad();
// }

// void Biquad::setFc(float Fc) {
// this->Fc = Fc;
// calcBiquad();
// }

// void Biquad::setPeakGain(float peakGainDB) {
// this->peakGain = peakGainDB;
// calcBiquad();
// }

// void Biquad::setBiquad(int type, float f0, float damp, float fs) {
  // this->type = type;
  // this->f0 = f0;
  // this->damp = damp;
  // this->fs = fs;
  // this->bla = fs;
  // calcBiquad();
// }

// void ControlTools::calcBiquad(void) {
  // float w0 = 2 * 3.14159265359 * f0 / fs;
  // float alpha = sin(w0) * damp;
  // switch (this->type) {
    // case bq_type_lowpass:



      // b0 = 0.5 * (1 - cos(w0)) / (1 + alpha) ;
      // b1 = 2 * b0;
      // b2 = b0 ;

      //a0 = 1
      // a1 = (-2 * cos(w0)) / (1 + alpha);
      // a2 = (1 - alpha) / (1 + alpha);
      // break;

    // case bq_type_highpass:

      // b0 =   (1 + cos(w0)) / 2 / (1 + alpha);
      // b1 =   -2 * b0;
      // b2 =   b0;
      //a0 =   1;
      // a1 =  -2 * cos(w0) / (1 + alpha);
      // a2 =   (1 - alpha) / (1 + alpha);

      // break;

    // case bq_type_notch:



      // break;

  // }

  // return;
// }
