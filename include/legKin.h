
#include "math.h"

#define gk1 0.0603
#define gk2 0.00115
#define gk3 0.2013
#define gk4 0.22663
#define gk5 8.4e-004
#define MIN 1.0e-10
#define MAX 1.0e+10

void FKFootBody(float Angles[], char Leg, float *PosRes);
void IKFootBody(float RFConfig[], float PrevRFConfig[], float JointAngles[],float *JointAnglesReturn, char Leg);
float (*InverseMatrix6(float a[][6]))[6];
float lu6(float a[][6], int *ip);
float (*JacobianComp6(float JointAngles[]))[6];
//float* FKLFBodyStatic(float Angles[]);
float* FKBodyStatic(float Angles[], char Leg);
float* NewtonRaphson(float RFConfig[], float Angles[], char Leg);

