
#include "stdafx.h"
#include "legkin.h"



void FKFootBody(float Angles[], char Leg, float *PosRes){
  //static float RF_Body[6];
float s1, s2, s3, s4, s5, s6, s45;
float c1, c2, c3, c4, c5, c6, c45;
float fk1, fk2 , fk3 , fk4 , fk5 , fk6 ;
float fk7, fk8 , fk9 , fk10 , fk11 , fk12;
float fk13, fk14, fk15, fk16, fk17, fk18;
float fk19, fk20, fk21, fk22, fk23, fk24;
float fk25, fk26, fk27, fk28;
float hk2, hk5;  
  // Trigonometic Parameters
  s1 = -sin(Angles[0]); // EKSILENDI
  s2 = sin(Angles[1]);
  s3 = sin(Angles[2]);
  s4 = sin(Angles[3]);
  s5 = sin(Angles[4]);
  s6 = sin(Angles[5]);
  c1 = cos(Angles[0]);
  c2 = cos(Angles[1]);
  c3 = cos(Angles[2]);
  c4 = cos(Angles[3]);
  c5 = cos(Angles[4]);
  c6 = cos(Angles[5]);
  s45 = s4*c5 + c4*s5;
  c45 = c4*c5 - s4*s5;

  //Subexpressions 
  fk1 = c2*c3*c6;
  fk2 = c2*s3*c4;
  fk3 = fk2 + s2*s4;
  fk4 = c2*s3*s4;
  fk5 = fk4 - s2*c4;
  fk6 = fk5*c5 + fk3*s5;
  fk7 = -(s1*s2*c3 + c1*s3);
  fk8 = s1*c2*s4;
  fk9 = s1*s2*s3;
  fk10 = c1*c3 - fk9;
  fk11 = s5*(fk8 + fk10*c4);
  fk12 = s1*c2*c4;
  fk13 = c5*(fk10*s4 - fk12);
  fk14 = s6*(fk13 + fk11) + c6*fk7;
  fk15 = c1*c2*s4;
  fk16 = c1*s2*s3;
  fk17 = fk16 + s1*c3;
  fk18 = c5*(fk17*c4 - fk15);
  fk19 = c1*c2*c4;
  fk20 = s5*(fk17*s4 + fk19);
  fk21 = c1*s2*c3;
  fk22 = fk21 - s1*s3;
  fk23 = (fk17*c4 - fk15)*s5;
  fk24 = (fk17*s4 + fk19)*c5;
  fk25 = fk22*s6 - c6*(fk24 + fk23);
  fk26 = fk22*c6 + s6*(fk24 + fk23);
  fk27 = c5*(fk8 + fk10*c4);
  fk28 = s5*(fk10*s4 - fk12);
  hk2 = (fk18 -   fk20);
  hk5 = -1/(fk6*s6 + fk1);
  
  // Avoid Singularity
  if((hk2<=MIN) && (hk2>0.0)) hk2 = MIN;
  if((hk2>=-MIN) && (hk2<0.0)) hk2 = -MIN;
  if((hk5>=MAX) && (hk5>0.0)) hk5 = MAX;
  if((hk5<=-MAX) && (hk5<0.0)) hk5 = -MAX; 

  // Right Foot Configuration, Position and Orientation
  PosRes[0] = gk1*(fk7*s6 - c6*(fk11 + fk13)) + gk2*(fk28 - fk27) + gk3*(fk12 - fk10*s4) + gk4*s1*c2 - gk5*c1;//x
  // Compensation on x-axis
  PosRes[0] = -2*0.00199 - PosRes[0];
  if(Leg == 'R')  PosRes[1] =  gk1*(c2*c3*s6 - fk6*c6) + gk2*(fk5*s5 - fk3*c5) + gk3*(s2*c4 - fk4) + gk4*s2 - 0.0821; //yr
  if(Leg == 'L')  PosRes[1] =  -(gk1*(c2*c3*s6 - fk6*c6) + gk2*(fk5*s5 - fk3*c5) + gk3*(s2*c4 - fk4) + gk4*s2) + 0.0821;//yl
  PosRes[2] = gk1*(fk22*s6 - c6*(fk24 + fk23)) - gk2*hk2 - gk3*(fk17*s4 + fk19) - gk4*c1*c2 - gk5*s1; //z
  PosRes[3] = -asin(fk26); //roll
  PosRes[4] = atan2(fk25,hk2); //pitch
  PosRes[5] = atan(fk14*hk5);
  //return RF_Body;
}

void IKFootBody(float RFConfig[], float PrevRFConfig[], float JointAngles[], float *JointAnglesReturn, char Leg){

int i, j;
//int k = 0;

float DRFConfig[6];
float (*Inv_Jacobc)[6];
//float Jacobc[6][6];
float (*JacobPointer)[6];
float *FKPointer;
//float FKRes[6];
//float FKLF[6];
//float FKRF[6];
//float BufferAngles[6];
float *QResid;

// Compensation on x-axis
RFConfig[0] = -2*0.00199 - RFConfig[0];
//if(Leg == 'R') RFConfig[1] = 0.0 + RFConfig[1]; // I know it's meaningless
if(Leg == 'L') RFConfig[1] = 2*0.0821 - RFConfig[1]; // This is meaningful
//Start Jacobian
// Trigonometic Parameters
JointAngles[0] = -JointAngles[0];
JacobPointer = JacobianComp6(JointAngles);
//for(i=0; i<6; i++) for(j=0; j<6; j++) Jacobc[i][j] = JacobPointer[i][j];
// Invert Jacobian
Inv_Jacobc = InverseMatrix6(JacobPointer);
for(i=0; i<6; i++) {
	DRFConfig[i] = RFConfig[i] - PrevRFConfig[i];
	JointAnglesReturn[i] = JointAngles[i];
}

 
for(i=0; i<6; i++){
	for(j=0; j<6; j++){
		JointAnglesReturn[i]  +=  Inv_Jacobc[i][j] * DRFConfig[j];
	}	
}
	JointAnglesReturn[0] = -JointAnglesReturn[0];
	//for(i=0; i<6; i++) BufferAngles[i] = JointAnglesReturn[i];
	// Raw Joint Angles Computed
  //Newton Raphson (Single Iteration)
  QResid = NewtonRaphson(RFConfig, JointAnglesReturn, Leg);
  for(i=1; i<6; i++) JointAnglesReturn[i] = JointAnglesReturn[i] + QResid[i];
  JointAnglesReturn[0] = JointAnglesReturn[0] - QResid[0];
  //JointAnglesReturn[0] = -JointAnglesReturn[0];
}// IK Procedure is Over

float* NewtonRaphson(float RFConfig[], float Angles[], char Leg){
int i, j;
float s1, s2, s3, s4, s5, s6, s45;
float c1, c2, c3, c4, c5, c6, c45;
float fk1, fk2 , fk3 , fk4 , fk5 , fk6 ;
float fk7, fk8 , fk9 , fk10 , fk11 , fk12;
float fk13, fk14, fk15, fk16, fk17, fk18;
float fk19, fk20, fk21, fk22, fk23, fk24;
float fk25, fk26, fk27, fk28, fk29, fk30;
float fk31, fk32, fk33, fk34, fk35, fk36;
float fk37, fk38, fk39, fk40;
float hk1, hk2, hk3, hk4, hk5, hk6, hk7;
float Jac[6][6];
float (*Inv_Jac)[6];
float ForKinDiff[6];
float ForKinCalc[6];
static float AngleDiff[6];

s1 = -sin(Angles[0]);
s2 = sin(Angles[1]);
s3 = sin(Angles[2]);
s4 = sin(Angles[3]);
s5 = sin(Angles[4]);
s6 = sin(Angles[5]);
c1 = cos(Angles[0]);
c2 = cos(Angles[1]);
c3 = cos(Angles[2]);
c4 = cos(Angles[3]);
c5 = cos(Angles[4]);
c6 = cos(Angles[5]);
s45 = s4*c5 + c4*s5;
c45 = c4*c5 - s4*s5;

//Subexpressions Found by MATLAB
fk1 = c2*c3*c6;
fk2 = c2*s3*c4;
fk3 = fk2 + s2*s4;
fk4 = c2*s3*s4;
fk5 = fk4 - s2*c4;
fk6 = fk5*c5 + fk3*s5;
fk7 = -(s1*s2*c3 + c1*s3);
fk8 = s1*c2*s4;
fk9 = s1*s2*s3;
fk10 = c1*c3 - fk9;
fk11 = s5*(fk8 + fk10*c4);
fk12 = s1*c2*c4;
fk13 = c5*(fk10*s4 - fk12);
fk14 = s6*(fk13 + fk11) + c6*fk7;
fk15 = c1*c2*s4;
fk16 = c1*s2*s3;
fk17 = fk16 + s1*c3;
fk18 = c5*(fk17*c4 - fk15);
fk19 = c1*c2*c4;
fk20 = s5*(fk17*s4 + fk19);
fk21 = c1*s2*c3;
fk22 = fk21 - s1*s3;
fk23 = (fk17*c4 - fk15)*s5;
fk24 = (fk17*s4 + fk19)*c5;
fk25 = fk22*s6 - c6*(fk24 + fk23);
fk26 = fk22*c6 + s6*(fk24 + fk23);
fk27 = c5*(fk8 + fk10*c4);
fk28 = s5*(fk10*s4 - fk12);
fk29 = 1 + ((fk14*fk14)/((fk1 + s6*fk6)*(fk1 + s6*fk6)));
fk30 = c2*s4 - s2*s3*c4;
fk31 = s2*s3*s4;
fk32 = -s1*(c2*s3*c4 + s2*s4);
fk33 = s1*s2*c4;
fk34 = s1*c2*s3*s4;
fk35 = -(fk16 + s1*c3);
fk36 = fk22*s45;
fk37 = c1*(c2*s3*c4 + s2*s4);
fk38 = c1*s2*c4;
fk39 = c1*c2*s3*s4;

// Manually Found Subexpressions
fk40 = c1*c2*c3;
hk1 = 1/sqrt(1 - fk26*fk26);
hk2 = (fk18 - fk20);
hk3 = hk2*hk2;
hk4 = 1/(fk25*fk25 + hk3);
hk5 = -1/(fk6*s6 + fk1);
hk6 = hk5*hk5;
hk7 = 1/fk29;

// First Row is Confirmed
Jac[0][0] = gk1*(-c6*(c5*(fk35*s4 - fk19) + s5*(fk35*c4 + fk15)) + s6*(s1*s3 - fk21))  + gk2*(s5*(fk35*s4 - fk19) - c5*(fk35*c4 + fk15)) + gk3*(fk19 - fk35*s4) + gk4*c1*c2 + gk5*s1;
Jac[0][1] = -gk1*(c6*(c5*(fk33 - fk34) + s5*fk32) + s6*s1*c2*c3) + gk2*(s5*(fk33 - fk34) - c5*fk32) + gk3*(fk34 - fk33) - gk4*s1*s2;
Jac[0][2] = gk1*(s6*(fk9 - c1*c3) - c6*fk7*s45) - gk2*fk7*c45 - gk3*fk7*s4;
Jac[0][3] = -gk1*c6*(s5*(fk12 - fk10*s4) + fk27) + gk2*(fk11 - c5*(fk12 - fk10*s4)) - gk3*(fk10*c4 + fk8);
Jac[0][4] = gk1*c6*(fk28 - fk27) + gk2*(fk11 + fk13);
Jac[0][5] = gk1*(c6*fk7 + s6*(fk11 + fk13));

// Second Row is Confirmed
Jac[1][0] = 0.0;
Jac[1][1] = -gk1*(c6*(-c5*(c4*c2 + fk31) + s5*fk30) + s6*(s2*c3)) - gk2*(s5*(c2*c4 + fk31) + c5*fk30) + gk3*(fk31 + c2*c4) + gk4*c2;
Jac[1][2] = -gk1*(c6*c2*c3*s45 + s6*c2*s3) - gk2*c2*c3*c45 - gk3*c2*c3*s4;
Jac[1][3] = -gk1*c6*(fk3*c5 + s5*(s2*c4 - fk4)) + gk2*(fk3*s5 - c5*(s2*c4 - fk4))  -gk3*(fk2 + s2*s4);
Jac[1][4] = -gk1*c6*(fk3*c5 - fk5*s5) + gk2*(fk5*c5 + fk3*s5);
Jac[1][5] = gk1*(fk1 + fk6*s6);

// Third Row is Confirmed
Jac[2][0] = gk1*(s6*fk7 - c6*(fk11 + fk13)) + gk2*(fk28 - fk27) + gk3*(fk12 - fk10*s4) + gk4*s1*c2 - gk5*c1;
Jac[2][1] = gk1*(-c6*(c5*(fk39 - fk38) + s5*fk37) + s6*fk40)  + gk2*(s5*(fk39 - fk38) - c5*fk37) + gk3*(fk38 - fk39) + gk4*c1*s2;
Jac[2][2] = gk1*(fk35*s6 - fk36*c6) - gk2*fk22*c45 - gk3*fk22*s4;
Jac[2][3] = gk1*c6*(s5*(fk19 + fk17*s4) - fk18) + gk2*(fk23 + c5*(fk19 + fk17*s4))   + gk3*(fk15 - fk17*c4);
Jac[2][4] = gk1*c6*(fk20 - fk18) + gk2*(fk24 + fk23);
Jac[2][5] = gk1*(s6*(fk24 + fk23) + c6*fk22);

// Fourth Row is Confirmed
Jac[3][0] = -fk14*hk1;
Jac[3][1] = -(s6*(fk37*s5 + c5*(fk39 - fk38)) + c6*fk40)*hk1;
Jac[3][2] = -(fk36*s6 + fk35*c6)*hk1;
Jac[3][3] = s6*(s5*(fk19 + fk17*s4) - fk18)*hk1;
Jac[3][4] = -(hk2*s6)*hk1;
Jac[3][5] = (s6*fk22 - c6*(fk23 + fk24))*hk1;

// Fifth Row is Confirmed
Jac[4][0] = (hk2*(s6*fk7 - c6*(fk11 + fk13)) + fk25*(fk28 - fk27))*hk4;
Jac[4][1] = (hk2*(-c6*(c5*(fk39 - fk38) + fk37*s5) + s6*fk40) - (fk25*(fk37*c5 + s5*(fk38 - fk39))))*hk4;
Jac[4][2] = (hk2*(s6*fk35 - c6*fk36) - fk25*(fk22*c45))*hk4;
Jac[4][3] = (hk2*c6*(s5*(fk19 + fk17*s4) - fk18) + fk25*(fk23 + c5*(fk19 + fk17*s4)))*hk4;
Jac[4][4] = (fk25*(fk24 + fk23) - c6*hk3)*hk4;
Jac[4][5] = hk2*fk26*hk4;

// Sixth Row is Confirmed
Jac[5][0] = (s6*(c5*(fk35*s4 - fk19) + s5*(fk15 + fk35*c4)) + c6*(s1*s3 - fk21))*hk5*hk7;
Jac[5][1] = ((((fk33 - fk34)*c5 + s5*fk32)*s6 - s1*c2*c3*c6)*hk5 - fk14*(((c2*c4 + fk31)*c5 - fk30*s5)*s6 + s2*c3*c6)*hk6)*hk7;
Jac[5][2] = hk7*(hk5*(s6*fk7*s45 + c6*(fk9 - c3*c1))  + fk14*(s6*c2*c3*s45 - c6*c2*s3)*hk6);
Jac[5][3] = s6*hk7*((fk27 + s5*(fk12 - fk10*s4))*hk5 + fk14*hk6*(fk3*c5 + s5*(s2*c4 - fk4)));
Jac[5][4] = s6*hk7*(hk5*(fk27 - fk28) + hk6*fk14*(fk3*c5 - fk5*s5));
Jac[5][5] = hk7*(hk5*(c6*(fk13 + fk11) - fk7*s6) - fk14*hk6*(s6*c2*c3 - fk6*c6));

Inv_Jac = InverseMatrix6(Jac);


// Foot Configuration, Position and Orientation
ForKinCalc[0] = gk1*(fk7*s6 - c6*(fk11 + fk13)) + gk2*(fk28 - fk27) + gk3*(fk12 - fk10*s4) + gk4*s1*c2 - gk5*c1;
if(Leg == 'R') ForKinCalc[1] =  gk1*(c2*c3*s6 - fk6*c6) + gk2*(fk5*s5 - fk3*c5) + gk3*(s2*c4 - fk4) + gk4*s2 - 0.0821;
if(Leg == 'L') ForKinCalc[1] =  (gk1*(c2*c3*s6 - fk6*c6) + gk2*(fk5*s5 - fk3*c5) + gk3*(s2*c4 - fk4) + gk4*s2) + 0.0821;
ForKinCalc[2] = gk1*(fk22*s6 - c6*(fk24 + fk23)) - gk2*hk2 - gk3*(fk17*s4 + fk19) - gk4*c1*c2 - gk5*s1;
ForKinCalc[3] = -asin(fk26);
ForKinCalc[4] = atan2(fk25,hk2);
ForKinCalc[5] = atan(fk14*hk5);

for(i=0; i<6; i++) ForKinDiff[i] = RFConfig[i] - ForKinCalc[i];

/*for(i=0; i<6; i++){
	for(j=0; j<6; j++){
		AngleDiff[i]  +=  Inv_Jac[i][j] * ForKinDiff[j];
	}	
}*/
AngleDiff[0] = Inv_Jac[0][0]*ForKinDiff[0] + Inv_Jac[0][1]*ForKinDiff[1] + Inv_Jac[0][2]*ForKinDiff[2] + Inv_Jac[0][3]*ForKinDiff[3] + Inv_Jac[0][4]*ForKinDiff[4] + Inv_Jac[0][5]*ForKinDiff[5];
AngleDiff[1] = Inv_Jac[1][0]*ForKinDiff[0] + Inv_Jac[1][1]*ForKinDiff[1] + Inv_Jac[1][2]*ForKinDiff[2] + Inv_Jac[1][3]*ForKinDiff[3] + Inv_Jac[1][4]*ForKinDiff[4] + Inv_Jac[1][5]*ForKinDiff[5];
AngleDiff[2] = Inv_Jac[2][0]*ForKinDiff[0] + Inv_Jac[2][1]*ForKinDiff[1] + Inv_Jac[2][2]*ForKinDiff[2] + Inv_Jac[2][3]*ForKinDiff[3] + Inv_Jac[2][4]*ForKinDiff[4] + Inv_Jac[2][5]*ForKinDiff[5];
AngleDiff[3] = Inv_Jac[3][0]*ForKinDiff[0] + Inv_Jac[3][1]*ForKinDiff[1] + Inv_Jac[3][2]*ForKinDiff[2] + Inv_Jac[3][3]*ForKinDiff[3] + Inv_Jac[3][4]*ForKinDiff[4] + Inv_Jac[3][5]*ForKinDiff[5];
AngleDiff[4] = Inv_Jac[4][0]*ForKinDiff[0] + Inv_Jac[4][1]*ForKinDiff[1] + Inv_Jac[4][2]*ForKinDiff[2] + Inv_Jac[4][3]*ForKinDiff[3] + Inv_Jac[4][4]*ForKinDiff[4] + Inv_Jac[4][5]*ForKinDiff[5];
AngleDiff[5] = Inv_Jac[5][0]*ForKinDiff[0] + Inv_Jac[5][1]*ForKinDiff[1] + Inv_Jac[5][2]*ForKinDiff[2] + Inv_Jac[5][3]*ForKinDiff[3] + Inv_Jac[5][4]*ForKinDiff[4] + Inv_Jac[5][5]*ForKinDiff[5];

return AngleDiff;
}// NR is over

float (*JacobianComp6(float JointAngles[]))[6] {
float s1, s2, s3, s4, s5, s6, s45;
float c1, c2, c3, c4, c5, c6, c45;
float fk1, fk2 , fk3 , fk4 , fk5 , fk6 ;
float fk7, fk8 , fk9 , fk10 , fk11 , fk12;
float fk13, fk14, fk15, fk16, fk17, fk18;
float fk19, fk20, fk21, fk22, fk23, fk24;
float fk25, fk26, fk27, fk28, fk29, fk30;
float fk31, fk32, fk33, fk34, fk35, fk36;
float fk37, fk38, fk39, fk40;
float hk1, hk2, hk3, hk4, hk5, hk6, hk7;
static float Jac[6][6];

s1 = sin(JointAngles[0]);
s2 = sin(JointAngles[1]);
s3 = sin(JointAngles[2]);
s4 = sin(JointAngles[3]);
s5 = sin(JointAngles[4]);
s6 = sin(JointAngles[5]);
c1 = cos(JointAngles[0]);
c2 = cos(JointAngles[1]);
c3 = cos(JointAngles[2]);
c4 = cos(JointAngles[3]);
c5 = cos(JointAngles[4]);
c6 = cos(JointAngles[5]);
s45 = s4*c5 + c4*s5;
c45 = c4*c5 - s4*s5;

//Subexpressions Found by MATLAB
fk1 = c2*c3*c6;
fk2 = c2*s3*c4;
fk3 = fk2 + s2*s4;
fk4 = c2*s3*s4;
fk5 = fk4 - s2*c4;
fk6 = fk5*c5 + fk3*s5;
fk7 = -(s1*s2*c3 + c1*s3);
fk8 = s1*c2*s4;
fk9 = s1*s2*s3;
fk10 = c1*c3 - fk9;
fk11 = s5*(fk8 + fk10*c4);
fk12 = s1*c2*c4;
fk13 = c5*(fk10*s4 - fk12);
fk14 = s6*(fk13 + fk11) + c6*fk7;
fk15 = c1*c2*s4;
fk16 = c1*s2*s3;
fk17 = fk16 + s1*c3;
fk18 = c5*(fk17*c4 - fk15);
fk19 = c1*c2*c4;
fk20 = s5*(fk17*s4 + fk19);
fk21 = c1*s2*c3;
fk22 = fk21 - s1*s3;
fk23 = (fk17*c4 - fk15)*s5;
fk24 = (fk17*s4 + fk19)*c5;
fk25 = fk22*s6 - c6*(fk24 + fk23);
fk26 = fk22*c6 + s6*(fk24 + fk23);
fk27 = c5*(fk8 + fk10*c4);
fk28 = s5*(fk10*s4 - fk12);
fk29 = 1 + ((fk14*fk14)/((fk1 + s6*fk6)*(fk1 + s6*fk6)));
fk30 = c2*s4 - s2*s3*c4;
fk31 = s2*s3*s4;
fk32 = -s1*(c2*s3*c4 + s2*s4);
fk33 = s1*s2*c4;
fk34 = s1*c2*s3*s4;
fk35 = -(fk16 + s1*c3);
fk36 = fk22*s45;
fk37 = c1*(c2*s3*c4 + s2*s4);
fk38 = c1*s2*c4;
fk39 = c1*c2*s3*s4;

// Manually Found Subexpressions
fk40 = c1*c2*c3;
hk1 = 1/sqrt(1 - fk26*fk26);
hk2 = (fk18 - fk20);
hk3 = hk2*hk2;
hk4 = 1/(fk25*fk25 + hk3);
hk5 = -1/(fk6*s6 + fk1);
hk6 = hk5*hk5;
hk7 = 1/fk29;

  // Avoid Singularity
  if((hk2<=MIN) && (hk2>0.0)) hk2 = MIN;
  if((hk2>=-MIN) && (hk2<0.0)) hk2 = -MIN;
  if((hk5>=MAX) && (hk5>0.0)) hk5 = MAX;
  if((hk5<=-MAX) && (hk5<0.0)) hk5 = -MAX; 

// First Row is Confirmed
Jac[0][0] = gk1*(-c6*(c5*(fk35*s4 - fk19) + s5*(fk35*c4 + fk15)) + s6*(s1*s3 - fk21))  + gk2*(s5*(fk35*s4 - fk19) - c5*(fk35*c4 + fk15)) + gk3*(fk19 - fk35*s4) + gk4*c1*c2 + gk5*s1;
Jac[0][1] = -gk1*(c6*(c5*(fk33 - fk34) + s5*fk32) + s6*s1*c2*c3) + gk2*(s5*(fk33 - fk34) - c5*fk32) + gk3*(fk34 - fk33) - gk4*s1*s2;
Jac[0][2] = gk1*(s6*(fk9 - c1*c3) - c6*fk7*s45) - gk2*fk7*c45 - gk3*fk7*s4;
Jac[0][3] = -gk1*c6*(s5*(fk12 - fk10*s4) + fk27) + gk2*(fk11 - c5*(fk12 - fk10*s4)) - gk3*(fk10*c4 + fk8);
Jac[0][4] = gk1*c6*(fk28 - fk27) + gk2*(fk11 + fk13);
Jac[0][5] = gk1*(c6*fk7 + s6*(fk11 + fk13));

// Second Row is Confirmed
Jac[1][0] = 0.0;
Jac[1][1] = -gk1*(c6*(-c5*(c4*c2 + fk31) + s5*fk30) + s6*(s2*c3)) - gk2*(s5*(c2*c4 + fk31) + c5*fk30) + gk3*(fk31 + c2*c4) + gk4*c2;
Jac[1][2] = -gk1*(c6*c2*c3*s45 + s6*c2*s3) - gk2*c2*c3*c45 - gk3*c2*c3*s4;
Jac[1][3] = -gk1*c6*(fk3*c5 + s5*(s2*c4 - fk4)) + gk2*(fk3*s5 - c5*(s2*c4 - fk4))  -gk3*(fk2 + s2*s4);
Jac[1][4] = -gk1*c6*(fk3*c5 - fk5*s5) + gk2*(fk5*c5 + fk3*s5);
Jac[1][5] = gk1*(fk1 + fk6*s6);

// Third Row is Confirmed
Jac[2][0] = gk1*(s6*fk7 - c6*(fk11 + fk13)) + gk2*(fk28 - fk27) + gk3*(fk12 - fk10*s4) + gk4*s1*c2 - gk5*c1;
Jac[2][1] = gk1*(-c6*(c5*(fk39 - fk38) + s5*fk37) + s6*fk40)  + gk2*(s5*(fk39 - fk38) - c5*fk37) + gk3*(fk38 - fk39) + gk4*c1*s2;
Jac[2][2] = gk1*(fk35*s6 - fk36*c6) - gk2*fk22*c45 - gk3*fk22*s4;
Jac[2][3] = gk1*c6*(s5*(fk19 + fk17*s4) - fk18) + gk2*(fk23 + c5*(fk19 + fk17*s4))   + gk3*(fk15 - fk17*c4);
Jac[2][4] = gk1*c6*(fk20 - fk18) + gk2*(fk24 + fk23);
Jac[2][5] = gk1*(s6*(fk24 + fk23) + c6*fk22);

// Fourth Row is Confirmed
Jac[3][0] = -fk14*hk1;
Jac[3][1] = -(s6*(fk37*s5 + c5*(fk39 - fk38)) + c6*fk40)*hk1;
Jac[3][2] = -(fk36*s6 + fk35*c6)*hk1;
Jac[3][3] = s6*(s5*(fk19 + fk17*s4) - fk18)*hk1;
Jac[3][4] = -(hk2*s6)*hk1;
Jac[3][5] = (s6*fk22 - c6*(fk23 + fk24))*hk1;

// Fifth Row is Confirmed
Jac[4][0] = (hk2*(s6*fk7 - c6*(fk11 + fk13)) + fk25*(fk28 - fk27))*hk4;
Jac[4][1] = (hk2*(-c6*(c5*(fk39 - fk38) + fk37*s5) + s6*fk40) - (fk25*(fk37*c5 + s5*(fk38 - fk39))))*hk4;
Jac[4][2] = (hk2*(s6*fk35 - c6*fk36) - fk25*(fk22*c45))*hk4;
Jac[4][3] = (hk2*c6*(s5*(fk19 + fk17*s4) - fk18) + fk25*(fk23 + c5*(fk19 + fk17*s4)))*hk4;
Jac[4][4] = (fk25*(fk24 + fk23) - c6*hk3)*hk4;
Jac[4][5] = hk2*fk26*hk4;

// Sixth Row is Confirmed
Jac[5][0] = (s6*(c5*(fk35*s4 - fk19) + s5*(fk15 + fk35*c4)) + c6*(s1*s3 - fk21))*hk5*hk7;
Jac[5][1] = ((((fk33 - fk34)*c5 + s5*fk32)*s6 - s1*c2*c3*c6)*hk5 - fk14*(((c2*c4 + fk31)*c5 - fk30*s5)*s6 + s2*c3*c6)*hk6)*hk7;
Jac[5][2] = hk7*(hk5*(s6*fk7*s45 + c6*(fk9 - c3*c1))  + fk14*(s6*c2*c3*s45 - c6*c2*s3)*hk6);
Jac[5][3] = s6*hk7*((fk27 + s5*(fk12 - fk10*s4))*hk5 + fk14*hk6*(fk3*c5 + s5*(s2*c4 - fk4)));
Jac[5][4] = s6*hk7*(hk5*(fk27 - fk28) + hk6*fk14*(fk3*c5 - fk5*s5));
Jac[5][5] = hk7*(hk5*(c6*(fk13 + fk11) - fk7*s6) - fk14*hk6*(s6*c2*c3 - fk6*c6));
return Jac;

}// Jacobian Computation is over

float (*InverseMatrix6(float a[][6]))[6] {
  static float a_inv[6][6];

  const int n = 6;
	int i, j, k, ii;
	float t, det;
	int ip[6];   

	det = lu6(a, ip);
	if (det != 0)
		for (k = n - 1; k >= 0; k--) {
			for (i = 0; i < n; i++) {
				ii = ip[i];  t = (ii == k);
				for (j = i - 1; j >= 0; j--)
					t -= a[ii][j] * a_inv[j][k];
				a_inv[i][k] = t;
			}
			for (i = n - 1; i >= 0; i--) {
				t = a_inv[i][k];  ii = ip[i];
				for (j = i + 1; j < n; j++)
					t -= a[ii][j] * a_inv[j][k];
				a_inv[i][k] = t / a[ii][i];
			}
		}

	return a_inv;
}

float lu6(float a[][6], int *ip) {
  const int n = 6;
	int i, j, k, ii, ik;
	float t, u, det;
	float weight[6];

	det = 0;                  
	for (k = n - 1; k >= 0; k--) {  
		ip[k] = k;             
		u = 0;                 
		for (j = n - 1; j >= 0; j--) {
			t = fabsf(a[k][j]); 
			if (t > u) u = t;
		}
		if (u == 0) goto EXIT;
		weight[k] = 1 / u;     
	}
	det = 1;                   
	for (k = 0; k < n; k++) {  
		u = -1;
		for (i = k; i < n; i++) {  
			ii = ip[i];            
			t = fabsf(a[ii][k]) * weight[ii];
			if (t > u) {  u = t;  j = i;  }
		}
		ik = ip[j];
		if (j != k) {
			ip[j] = ip[k];  ip[k] = ik;  
			det = -det; 
		}
		u = a[ik][k];  det *= u; 
		if (u == 0) goto EXIT;    
		for (i = k + 1; i < n; i++) {  
			ii = ip[i];
			t = (a[ii][k] /= u);
			for (j = k + 1; j < n; j++)
				a[ii][j] -= t * a[ik][j];
		}
	}
EXIT:

	return det;           
}

float* FKBodyStatic(float Angles[], char Leg){
static float RF_Body[6];
float s1, s2, s3, s4, s5, s6, s45;
float c1, c2, c3, c4, c5, c6, c45;
float fk1, fk2 , fk3 , fk4 , fk5 , fk6 ;
float fk7, fk8 , fk9 , fk10 , fk11 , fk12;
float fk13, fk14, fk15, fk16, fk17, fk18;
float fk19, fk20, fk21, fk22, fk23, fk24;
float fk25, fk26, fk27, fk28;
float hk2, hk5;  
  // Trigonometic Parameters
  s1 = -sin(Angles[0]); // EKSILENDI
  s2 = sin(Angles[1]);
  s3 = sin(Angles[2]);
  s4 = sin(Angles[3]);
  s5 = sin(Angles[4]);
  s6 = sin(Angles[5]);
  c1 = cos(Angles[0]);
  c2 = cos(Angles[1]);
  c3 = cos(Angles[2]);
  c4 = cos(Angles[3]);
  c5 = cos(Angles[4]);
  c6 = cos(Angles[5]);
  s45 = s4*c5 + c4*s5;
  c45 = c4*c5 - s4*s5;

  //Subexpressions 
  fk1 = c2*c3*c6;
  fk2 = c2*s3*c4;
  fk3 = fk2 + s2*s4;
  fk4 = c2*s3*s4;
  fk5 = fk4 - s2*c4;
  fk6 = fk5*c5 + fk3*s5;
  fk7 = -(s1*s2*c3 + c1*s3);
  fk8 = s1*c2*s4;
  fk9 = s1*s2*s3;
  fk10 = c1*c3 - fk9;
  fk11 = s5*(fk8 + fk10*c4);
  fk12 = s1*c2*c4;
  fk13 = c5*(fk10*s4 - fk12);
  fk14 = s6*(fk13 + fk11) + c6*fk7;
  fk15 = c1*c2*s4;
  fk16 = c1*s2*s3;
  fk17 = fk16 + s1*c3;
  fk18 = c5*(fk17*c4 - fk15);
  fk19 = c1*c2*c4;
  fk20 = s5*(fk17*s4 + fk19);
  fk21 = c1*s2*c3;
  fk22 = fk21 - s1*s3;
  fk23 = (fk17*c4 - fk15)*s5;
  fk24 = (fk17*s4 + fk19)*c5;
  fk25 = fk22*s6 - c6*(fk24 + fk23);
  fk26 = fk22*c6 + s6*(fk24 + fk23);
  fk27 = c5*(fk8 + fk10*c4);
  fk28 = s5*(fk10*s4 - fk12);
  hk2 = (fk18 -   fk20);
  hk5 = -1/(fk6*s6 + fk1);
  
  // Avoid Singularity
  if((hk2<=MIN) && (hk2>0.0)) hk2 = MIN;
  if((hk2>=-MIN) && (hk2<0.0)) hk2 = -MIN;
  if((hk5>=MAX) && (hk5>0.0)) hk5 = MAX;
  if((hk5<=-MAX) && (hk5<0.0)) hk5 = -MAX; 

  // Right Foot Configuration, Position and Orientation
  RF_Body[0] = gk1*(fk7*s6 - c6*(fk11 + fk13)) + gk2*(fk28 - fk27) + gk3*(fk12 - fk10*s4) + gk4*s1*c2 - gk5*c1;
  if(Leg == 'R') RF_Body[1] =  gk1*(c2*c3*s6 - fk6*c6) + gk2*(fk5*s5 - fk3*c5) + gk3*(s2*c4 - fk4) + gk4*s2 - 0.0821;
  if(Leg == 'L') RF_Body[1] =  -(gk1*(c2*c3*s6 - fk6*c6) + gk2*(fk5*s5 - fk3*c5) + gk3*(s2*c4 - fk4) + gk4*s2) + 0.0821;
  RF_Body[2] = gk1*(fk22*s6 - c6*(fk24 + fk23)) - gk2*hk2 - gk3*(fk17*s4 + fk19) - gk4*c1*c2 - gk5*s1;
  RF_Body[3] = -asin(fk26);
  RF_Body[4] = atan2(fk25,hk2);
  RF_Body[5] = atan(fk14*hk5);
  return RF_Body;
}

/*float* FKLFBodyStatic(float Angles[]){
// Don't be confused by RF, it's just a name in this function
static float RF_Body[6];
float s1, s2, s3, s4, s5, s6, s45;
float c1, c2, c3, c4, c5, c6, c45;
float fk1, fk2 , fk3 , fk4 , fk5 , fk6 ;
float fk7, fk8 , fk9 , fk10 , fk11 , fk12;
float fk13, fk14, fk15, fk16, fk17, fk18;
float fk19, fk20, fk21, fk22, fk23, fk24;
float fk25, fk26, fk27, fk28;
float hk2, hk5;  
  // Trigonometic Parameters
  s1 = -sin(Angles[0]); // EKSILENDI
  s2 = sin(Angles[1]);
  s3 = sin(Angles[2]);
  s4 = sin(Angles[3]);
  s5 = sin(Angles[4]);
  s6 = sin(Angles[5]);
  c1 = cos(Angles[0]);
  c2 = cos(Angles[1]);
  c3 = cos(Angles[2]);
  c4 = cos(Angles[3]);
  c5 = cos(Angles[4]);
  c6 = cos(Angles[5]);
  s45 = s4*c5 + c4*s5;
  c45 = c4*c5 - s4*s5;

  //Subexpressions 
  fk1 = c2*c3*c6;
  fk2 = c2*s3*c4;
  fk3 = fk2 + s2*s4;
  fk4 = c2*s3*s4;
  fk5 = fk4 - s2*c4;
  fk6 = fk5*c5 + fk3*s5;
  fk7 = -(s1*s2*c3 + c1*s3);
  fk8 = s1*c2*s4;
  fk9 = s1*s2*s3;
  fk10 = c1*c3 - fk9;
  fk11 = s5*(fk8 + fk10*c4);
  fk12 = s1*c2*c4;
  fk13 = c5*(fk10*s4 - fk12);
  fk14 = s6*(fk13 + fk11) + c6*fk7;
  fk15 = c1*c2*s4;
  fk16 = c1*s2*s3;
  fk17 = fk16 + s1*c3;
  fk18 = c5*(fk17*c4 - fk15);
  fk19 = c1*c2*c4;
  fk20 = s5*(fk17*s4 + fk19);
  fk21 = c1*s2*c3;
  fk22 = fk21 - s1*s3;
  fk23 = (fk17*c4 - fk15)*s5;
  fk24 = (fk17*s4 + fk19)*c5;
  fk25 = fk22*s6 - c6*(fk24 + fk23);
  fk26 = fk22*c6 + s6*(fk24 + fk23);
  fk27 = c5*(fk8 + fk10*c4);
  fk28 = s5*(fk10*s4 - fk12);
  hk2 = (fk18 -   fk20);
  hk5 = -1/(fk6*s6 + fk1);
  
  // Avoid Singularity
  if((hk2<=MIN) && (hk2>0.0)) hk2 = MIN;
  if((hk2>=-MIN) && (hk2<0.0)) hk2 = -MIN;
  if((hk5>=MAX) && (hk5>0.0)) hk5 = MAX;
  if((hk5<=-MAX) && (hk5<0.0)) hk5 = -MAX; 

  // Right Foot Configuration, Position and Orientation
  RF_Body[0] = gk1*(fk7*s6 - c6*(fk11 + fk13)) + gk2*(fk28 - fk27) + gk3*(fk12 - fk10*s4) + gk4*s1*c2 - gk5*c1;
  RF_Body[1] =  -(gk1*(c2*c3*s6 - fk6*c6) + gk2*(fk5*s5 - fk3*c5) + gk3*(s2*c4 - fk4) + gk4*s2) + 0.0821;
  RF_Body[2] = gk1*(fk22*s6 - c6*(fk24 + fk23)) - gk2*hk2 - gk3*(fk17*s4 + fk19) - gk4*c1*c2 - gk5*s1;
  RF_Body[3] = -asin(fk26);
  RF_Body[4] = atan2(fk25,hk2);
  RF_Body[5] = atan(fk14*hk5);
  return RF_Body;
}*/

// SIL
