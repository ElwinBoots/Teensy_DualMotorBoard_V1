#include "MotionProfile.h"
	
MotionProfile::MotionProfile(float tstart, float t1, float t2, float t3, float p, float v_max, float a_max, float j_max, float Ts ){
    this->tstart = tstart;
    this->t1 = t1;
    this->t2 = t2;
    this->t3 = t3;
    this->p = p;
    this->v_max = v_max;
    this->a_max = a_max;
    this->j_max = j_max;
	this->Ts = Ts;
    rdelay = 0;
	REFqmem = 0;
	REFstatus = 0;
	reltim = 0;
	t = 0.0;
	init();
}
 
void MotionProfile::init() {
    if (p < 0.0) {
        REFidir= 0;   //Direction of movement negative
        p = -p;
    }
    else {
        REFidir= 1;    //Direction of movement positive
    }
	v = v_max;
    if (v >= 0.0) {     //v = |v| (Do not use abs() because it is not exact)
    } else {
        v = -v;
    }
    if (j_max >= 0.0) {  //j = |maxjerk|
        j = j_max;
    } else {
        j = -j_max;
    }
    
//  Compute switching times
    REFt0 = tstart;
    REFt1 = REFt0 + t1;
    REFt2 = REFt1 + t2;
    REFt3 = REFt2 + t1;
    REFt4 = REFt3 + t3;
    REFt5 = REFt4 + t1;
    REFt6 = REFt5 + t2;
    REFt7 = REFt6 + t1;
    
//	Compute reference values at switching times
    REFxstrt = REFqmem;
    REFs0 = 0.0;
    
    REFa1 = j*(REFt1-REFt0);
    REFv1 = 0.5*j*(REFt1-REFt0)*(REFt1-REFt0);
    REFs1 = REFs0+j*(REFt1-REFt0)*(REFt1-REFt0)*(REFt1-REFt0)/6.0;
    
    REFa2=REFa1;
    REFv2=REFv1+REFa1*(REFt2-REFt1);
    REFs2=REFs1+REFv1*(REFt2-REFt1)+0.5*REFa1*(REFt2-REFt1)*(REFt2-REFt1);
    
    REFa3=REFa2-j*(REFt3-REFt2);
    REFv3=REFv2+REFa2*(REFt3-REFt2)-0.5*j*(REFt3-REFt2)*(REFt3-REFt2);
    REFs3=REFs2+REFv2*(REFt3-REFt2)+0.5*REFa2*(REFt3-REFt2)*(REFt3-REFt2)
    -j*(REFt3-REFt2)*(REFt3-REFt2)*(REFt3-REFt2)/6.0;
    
    REFa4=0.0;
    REFv4=REFv3;
    REFs4=REFs3+REFv3*(REFt4-REFt3);
    
    REFa5=-j*(REFt5-REFt4);
    REFv5=REFv4-0.5*j*(REFt5-REFt4)*(REFt5-REFt4);
    REFs5=REFs4+REFv4*(REFt5-REFt4)-j*(REFt5-REFt4)*(REFt5-REFt4)*(REFt5-REFt4)/6.0;
    
    REFa6=REFa5;
    REFv6=REFv5+REFa5*(REFt6-REFt5);
    REFs6=REFs5+REFv5*(REFt6-REFt5)+0.5*REFa5*(REFt6-REFt5)*(REFt6-REFt5);
    
    REFa7=REFa6+j*(REFt7-REFt6);
    REFv7=REFv6+REFa6*(REFt7-REFt6)+0.5*j*(REFt7-REFt6)*(REFt7-REFt6);
    //REFs7=REFs6+REFv6*(REFt7-REFt6)+0.5*REFa6*(REFt7-REFt6)*(REFt7-REFt6)
    //+j*(REFt7-REFt6)*(REFt7-REFt6)*(REFt7-REFt6)/6.0;
	REFs7 = p;
}


float MotionProfile::p2p_get_b( double t ){
    if (t <= REFt0) {
        jref = 0;
        aref = 0;
        vref = 0;
    }
    else if (t <= REFt1) {
        jref = j;
        aref = j*(t-REFt0);
        vref = 0.5*j*(t-REFt0)*(t-REFt0);
    }
    else if (t <= REFt2) {
        jref = 0;
        aref = REFa1;
        vref = REFv1+REFa1*(t-REFt1);
    }
    else if (t <= REFt3) {
        jref = -j;
        aref = REFa2-j*(t-REFt2);
        vref = REFv2+REFa2*(t-REFt2)-0.5*j*(t-REFt2)*(t-REFt2);
    }
    else if (t <= REFt4) {
        jref = 0;
        aref = 0;
        vref = REFv3;
    }
    else if (t <= REFt5) {
        jref = -j;
        aref = -j*(t-REFt4);
        vref = REFv4-0.5*j*(t-REFt4)*(t-REFt4);
    }
    else if (t <= REFt6) {
        jref = 0;
        aref = REFa5;
        vref = REFv5+REFa5*(t-REFt5);
    }
    else if (t <= REFt7) {
        jref = j;
        aref = REFa6+j*(t-REFt6);
        vref = REFv6+REFa6*(t-REFt6)+0.5*j*(t-REFt6)*(t-REFt6);
    }
    else {
        jref = 0.0;
        aref = 0.0;
        vref = 0.0;
    }

    t -= rdelay*Ts; 

    if (t <= REFt0) {
        qref = REFs0;
    }
    else if (t <= REFt1) {
        qref = REFs0+j*(t-REFt0)*(t-REFt0)*(t-REFt0)/6.0;
    }
    else if (t <= REFt2) {
        qref = REFs1+REFv1*(t-REFt1)+0.5*REFa1*(t-REFt1)*(t-REFt1);
    }
    else if (t <= REFt3) {
        qref = REFs2+REFv2*(t-REFt2)+0.5*REFa2*(t-REFt2)*(t-REFt2)
        -j*(t-REFt2)*(t-REFt2)*(t-REFt2)/6.0;
    }
    else if (t <= REFt4) {
        qref = REFs3+REFv3*(t-REFt3);
    }
    else if (t <= REFt5) {
        qref = REFs4+REFv4*(t-REFt4)-j*(t-REFt4)*(t-REFt4)*(t-REFt4)/6.0;
    }
    else if (t <= REFt6) {
        qref = REFs5+REFv5*(t-REFt5)+0.5*REFa5*(t-REFt5)*(t-REFt5);
    }
    else if (t <= REFt7) {
        qref = REFs6+REFv6*(t-REFt6)+0.5*REFa6*(t-REFt6)*(t-REFt6)
        +j*(t-REFt6)*(t-REFt6)*(t-REFt6)/6.0;
    }
    else {
        qref = REFs7;
    }


    if (REFidir == 0) {
        jref = -jref;
        aref = -aref;
        vref = -vref;
        qref = -qref;
    }
    qref += REFxstrt;
	
	return qref;
}



float MotionProfile::stateCalculation( int input_status) {
        
    if (input_status ==0) {
        REFstatus = 0;
        
    } else if ((input_status==1) & (REFstatus==0)) {
		REFxstrt = REFqmem;
        REFstatus = 1;
		reltim = 0;
    } else if ((input_status==2) & (REFstatus==1)) {
        REFstatus = 2;
    }
    
    
    
    switch ( REFstatus )
    {
        case 0:
            
            qref = REFqmem;
            jref = 0;
			aref = 0;
			vref = 0;
            
            break;
            
        case 1:
		    reltim += Ts;
			qref = p2p_get_b( reltim );
            if (reltim > REFt7 + rdelay*Ts ) {
				REFstatus = 2;
				REFqmem = qref;
			} 
			
            break;        
        case 2:
            
			qref = REFqmem; 
            jref = 0;
			aref = 0;
			vref = 0;
            break;
    }
    return qref;
}
