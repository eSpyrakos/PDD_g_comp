
#include <stdafx.h>
#include "trajectory.h"

float FuncXcom(float RealTime, float tstart, float twidth, float z, float x0, float Vmean){
float te, tmid, wt;
float Xzmp, xd, dx0, ddx0;
//float ddy0;
float k0, kd, w;
float Xcom; 

w = sqrt(Grav/z);
te = tstart + twidth;
tmid = (te+tstart)*0.5;
wt = w*(RealTime - tstart);
//Vmean = Vmean*0.277777777778;
xd = x0 + twidth*Vmean;
k0 = sinh(w*(te-tmid));
kd = sinh(w*(tmid-tstart));
Xzmp = (k0*x0 + kd*xd) / (k0+kd);
dx0 = w*(Xzmp-x0)*(cosh(w*(tmid-tstart))/kd);
ddx0 = w*w*(x0-Xzmp);

if(Interval(RealTime, tstart, te) == TRUE) Xcom = (x0-Xzmp)*cosh(wt) + (dx0/w)*sinh(wt) + Xzmp;
else if(Less(RealTime, tstart) == TRUE) Xcom = x0;
else Xcom = xd;
return Xcom;

}

float FuncYcom(float RealTime, float tstart, float twidth, float z, float y0, float yd){
float te, tmid, wt;
float Hyy, w, wtm;
//float ddy0;
float dy0OverOmega, Yzmp;
float Ycom;

w = sqrt(Grav/z);
te = tstart + twidth;
tmid = (te+tstart)*0.5;
wt = w*(RealTime - tstart);
wtm = w*(tmid-tstart);
Hyy = -sech(wtm);
Yzmp = (yd + y0*Hyy) / (1+Hyy);
//dy0 = w*(Yzmp-y0)*tanh(wtm);
dy0OverOmega = (Yzmp-y0)*tanh(wtm); // dy0/w (why would I multiply it with w then divide the same thing?

if(Interval(RealTime, tstart, te) == TRUE) Ycom = (y0-Yzmp)*cosh(wt) + dy0OverOmega*sinh(wt) + Yzmp;
//else if(Less(RealTime, tstart) == TRUE) Ycom = y0;
else Ycom = y0;
return Ycom;
}


// Aralik, ta dahil, tb haric
// When t>=ta AND t<tb
unsigned int Interval(float RealTime, float ta, float tb){
if((RealTime>(ta-0.0005))&&(RealTime<(tb-0.0005))) return TRUE;
else return FALSE;

}

// Aralik, ta da tb de dahil
// When t>=ta AND t<= tb
unsigned int EqInterval(float RealTime, float ta, float tb){
if((RealTime>(ta-0.0005))&&(RealTime<(tb+0.0005))) return TRUE;
else return FALSE;

}

// Esitlik, zaman ta e esit mi?
// When t==ta
unsigned int Equal(float RealTime, float ta){
if((RealTime>(ta-0.0005))&&(RealTime<(ta+0.0005))) return TRUE;
else return FALSE;

}

// Karsilastirma, ta dan esit ya da daha buyuk
// When t>=ta
unsigned int GrtOrEq(float RealTime, float ta){
if(RealTime>(ta-0.0005)) return TRUE;
else return FALSE;

}

// Zaman ta dan buyuk mu
// When t>ta
unsigned int Greater(float RealTime, float ta){
if(RealTime>(ta+0.0005)) return TRUE;
else return FALSE;

}
// Karsilastirma, ta ya esit ya da daha kucuk
// When t<=ta
unsigned int LessOrEq(float RealTime, float ta){
if(RealTime<(ta+0.0005)) return TRUE;
else return FALSE;
}

// When t<ta
unsigned int Less(float RealTime, float ta){
if(RealTime<(ta-0.0005)) return TRUE;
else return FALSE;
}





// 5th order polynomial for general purposes
float Polinom5(const float RealTime, const float tstart, const float twidth, const float z01, const float v01, const float a01, const float z02, const float v02, const float a02){

float COMz;
//float z01, z02, v01, v02, a01, a02;
float FCoeffs6[6];

  FCoeffs6[0] = -0.5*(12*z01-12*z02+6*v01*twidth+6*v02*twidth+a01*(twidth*twidth)-a02*(twidth*twidth))/(twidth*twidth*twidth*twidth*twidth);
  FCoeffs6[1] = 0.5*(30*z01-30*z02+16*v01*twidth+14*v02*twidth+3*a01*(twidth*twidth)-2*a02*(twidth*twidth))/(twidth*twidth*twidth*twidth);
  FCoeffs6[2] = -0.5*(20*z01-20*z02+12*v01*twidth+8*v02*twidth+3*a01*(twidth*twidth)-a02*(twidth*twidth))/(twidth*twidth*twidth);
  FCoeffs6[3] = 0.5*a01;
  FCoeffs6[4] = v01;
  FCoeffs6[5] = z01;

if(Interval(RealTime, tstart, (tstart+twidth)) == TRUE){
  COMz =  FCoeffs6[0]*(RealTime-tstart)*(RealTime-tstart)*(RealTime-tstart)*(RealTime-tstart)*(RealTime-tstart) + FCoeffs6[1]*(RealTime-tstart)*(RealTime-tstart)*(RealTime-tstart)*(RealTime-tstart) + FCoeffs6[2]*(RealTime-tstart)*(RealTime-tstart)*(RealTime-tstart) + FCoeffs6[3]*(RealTime-tstart)*(RealTime-tstart) + FCoeffs6[4]*(RealTime-tstart) + FCoeffs6[5];

}


else if (Less(RealTime, tstart)) COMz = z01;

else COMz = z02;

return COMz;

}

// 6th order polynomial for z-axis swing foot position generation (ask Barkan why 6th? if you cant get it)
float FootPolinom6(const float RealTime, const float tstart, const float twidth, const float z0, const float za, const float z1){
  //z0 = Initial z-axis foot position
  //za = Interval z-axis foot position
  //z1 = Final z-axis foot position
  // RealTime = Time variable, tstart = initial time, twidth = polynomial trajectory period
  float Tw6, Tw5, Tw4, Tw3;
  float n61, n51, n41, n31, n01;
  float COMz;
  Tw3 = twidth*twidth*twidth;
  Tw4 = Tw3*twidth;
  Tw5 = Tw4*twidth;
  Tw6 = Tw5*twidth;

  n61 = 32*(z0-2*za+z1)/Tw6;
  n51 = -6*(17*z0-32*za+15*z1)/Tw5;
  n41 = 3*(37*z0-64*za+27*z1)/Tw4;
  n31 = (-42*z0+64*za-22*z1)/Tw3;
  n01 = z0;
if(Interval(RealTime, tstart, (tstart+twidth)) == TRUE){
COMz = n61*(RealTime-tstart)*(RealTime-tstart)*(RealTime-tstart)*(RealTime-tstart)*(RealTime-tstart)*(RealTime-tstart) + n51*(RealTime-tstart)*(RealTime-tstart)*(RealTime-tstart)*(RealTime-tstart)*(RealTime-tstart) + n41*(RealTime-tstart)*(RealTime-tstart)*(RealTime-tstart)*(RealTime-tstart) + n31*(RealTime-tstart)*(RealTime-tstart)*(RealTime-tstart) + n01;
}
else if (Less(RealTime, tstart)) COMz = z0;
else COMz = z1;
return COMz;
}

float sech(float a){
return 1/cosh(a);
}

float coth(float a){
return cosh(a)/sinh(a);
}
//FILE *fp_poly5;

/*
  int  main()
    {
	fp_poly5 = fopen("poly5.dat", "w");
	int i;
	float t, Ycom;

	for(i=0; i<=5000; i++){
		t = i*0.001;
      Ycom = Ytraj(t, 0.5, 1.0, 0.0821-0.02, 0.0821-0.05, 0);
      fprintf(fp_poly5, "%f ", t);

	fprintf(fp_poly5, "%f ", Ycom);
	fprintf(fp_poly5, "\n");  


	}
	
fclose(fp_poly5);
return 0;
}
*/

/*float Ytraj(float RealTime, float t0, float te, float y0, float ym, int ID){
float w, Yzmp, dy0, tau, tmy, Hyy;
	tmy = (te + t0)*0.5;
	tau = RealTime - t0;
	w = sqrtf(Grav/Zc);
	Hyy = -sech(w*(tmy-t0));
	Yzmp = (ym + y0*Hyy) / (1+Hyy);
	dy0 = w*(Yzmp - y0)*tanh(w*(tmy-t0));
   
	//return dy0;    
	if( Interval(RealTime, t0, te) == TRUE ) return (y0-Yzmp)*cosh(w*tau) + (dy0/w)*sinh(w*tau) + Yzmp;
	else return y0;
}*/