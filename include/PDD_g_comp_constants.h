#ifndef PDD_g_comp_CONSTANTS_H_
#define PDD_g_comp_CONSTANTS_H_

#define JOINT_NUMBER 6
#define JOINT_GAINS_NUMBER 4
#define LEGS_NUMBER 2 
#define LOWER_BODY_DOF 15
#define HIP_PITCH_JOINT_INDEX 0
#define KNEE_PITCH_JOINT_INDEX 3
#define ANKLE_PITCH_JOINT_INDEX 5

#define TRAJ_POINTS 20000

#define VOLTAGE_TO_TORQUE_GAIN 6.1747

const double posPID_right[JOINT_NUMBER * JOINT_GAINS_NUMBER]  = {
    
                                                          // Link Position
							  3.174108e+000 ,	//0 Right Ankle
							  3.491918e+000 ,	//1 Right Knee
							 -3.016638e-002,	//2 Right Hip
							  5.409832e+000 ,	//3 Left Hip 
							  3.255037e+000 ,	//4 Left Knee
							  1.780261e+000 ,	//5 Left Ankle
                                                          
                                                          // Motor Position
							  4.154596e+001 ,	//6  Right Ankle
							  3.305623e+001 ,	//7  Right Knee
							  3.658584e+001 ,	//8  Right Hip
							  8.352039e+001 ,	//9  Left Hip 
							  5.433240e+001 ,	//10 Left Knee
 							  9.759321e+001 ,	//11 Left Ankle
                                                          
                                                          // Link Velocity
							  1.006793e+000 ,	//12 Right Ankle
							  6.031094e-001 ,	//13 Right Knee
							  3.203863e-001 ,	//14 Right Hip
							  5.201745e-001 ,	//15 Left Hip 
							  1.257204e-001 ,	//16 Left Knee
							  4.122792e-003 ,	//17 Left Ankle
                                                          
                                                          // Motor Velocity
							  2.221849e-001 ,	//18 Right Ankle
							  1.762404e-001 ,	//19 Right Knee
							  1.960166e-001 ,	//20 Right Hip
							  4.397065e-001 ,	//21 Left Hip 
							  2.862677e-001 ,	//22 Left Knee
							  5.112359e-001 	//23 Left Ankle
							};
							
const double posPID_left[JOINT_NUMBER * JOINT_GAINS_NUMBER]  = {
    
                                                          // Link Position
 							  1.780261e+000 ,	//0  Right Ankle
							  3.255037e+000 ,	//1  Right Knee
							  5.409832e+000 ,	//2  Right Hip
							 -3.016638e-002,	//3  Left Hip
							  3.491918e+000 ,	//4  Left Knee
							  3.174108e+000 ,	//5  Left Ankle
                                                          
                                                          // Motor Position
							  9.759321e+001 ,	//6  Right Ankle
							  5.433240e+001 ,	//7  Right Knee
							  8.352039e+001 ,	//8  Right Hip
							  3.658584e+001 ,	//9  Left Hip
							  3.305623e+001 ,	//10 Left Knee
							  4.154596e+001 ,	//11 Left Ankle
                                                          
                                                          // Link Velocity
							  4.122792e-003 ,	//12 Right Ankle
							  1.257204e-001 ,	//13 Right Knee
							  5.201745e-001 ,	//14 Right Hip
							  3.203863e-001 ,	//15 Left Hip
							  6.031094e-001 ,	//16 Left Knee
							  1.006793e+000 ,	//17 Left Ankle
                                                          
                                                          // Motor Velocity
							  5.112359e-001 ,	//18 Right Ankle
							  2.862677e-001 ,	//19 Right Knee
							  4.397065e-001 ,	//20 Right Hip
							  1.960166e-001 ,	//21 Left Hip
							  1.762404e-001 ,	//22 Left Knee
							  2.221849e-001 	//23 Left Ankle
						      };
                                                      
                                                      						      
const double posPID_double[JOINT_NUMBER * JOINT_GAINS_NUMBER]  = {
							    // link position
							    0  ,	//0  Ankle
							    0  ,	//1  Knee
							    0  ,	//2  Hip
							    // motor position
							    36.3625 ,	//3  Ankle
							    28.2021 ,	//4  Knee
							    20.4983 ,	//5  Hip
							    // link velocity
							    2.8544 ,	//6  Ankle
							    0.8996 ,	//7  Knee
							    0.1178 ,	//8  Hip
							    // motor velocity
							    0.1938 ,	//9  Ankle
							    0.1503 ,	//10 Knee
							    0.1098  	//11 Hip
							  };
////BENT MODEL GAINS HIGH VOLTAGES BOTH LEGS////
                                                          
const float ucs_mat_Sing[JOINT_NUMBER][JOINT_NUMBER]=
{      -0.40711,      -0.40711,     -0.40711,     -0.40711,     -0.40711,     -0.40711,
   -1.0522e-016,      -0.47391,     -0.47391,     -0.47391,     -0.47391,     -0.47391,
    7.8388e-017,    7.933e-017,     -0.14464,     -0.14464,     -0.14464,     -0.14464,
   -5.9165e-031,   1.1102e-016, -2.3666e-030,     -0.14464,     -0.14464,     -0.14464,
              0,             0,            0,            0,     -0.47391,     -0.47391,
              0,   2.2204e-016,  2.2204e-016,            0,  4.4409e-016,     -0.40711
};

const float ucs_mat_Doub[JOINT_NUMBER][JOINT_NUMBER / LEGS_NUMBER]=
{	-0.1159,   -0.1159,   -0.1159,
        -0.1159,   -0.1159,   -0.1159,
         0.0000,   -0.1081,   -0.1081,
         0.0000,   -0.1081,   -0.1081,
        -0.0000,   -0.0000,   -0.1069,
        -0.0000,   -0.0000,   -0.1069
};


////BENT MODEL GAINS HIGH VOLTAGES BOTH LEGS////

const double gtorque_single[JOINT_NUMBER] = {30.1376, 29.0724, 5.9147, -1*9.4133, -1*4.3860, -1*0.3076};
const double gtorque_double[JOINT_NUMBER / LEGS_NUMBER] = {47.2273, 44.5837, 7.1666};
const double feed_forward_single[JOINT_NUMBER] = {4.472007e+001, 3.654815e+001, 3.655567e+001, 8.893022e+001, 5.758744e+001, 9.937348e+001 };
const double feed_forward_double[JOINT_NUMBER / LEGS_NUMBER] = {36.3625, 28.2021, 20.4983};

// 
// ////%%%%%%%%%%%%%% ONLINE G-COMP DOUBLE SUPPORT %%%%%%%%%%%%%%%
// urefmat_Doub[0] =   gtorque_double[0]*(float((j8+j13)/2));
// urefmat_Doub[1] =   gtorque_double[1]*(float((j8+j13+j7+j12)/2));
// urefmat_Doub[2] =   gtorque_double[2]*(float((j8+j13+j7+j12+j4+j3)/2)); 
// ////%%%%%%%%%%%%%% ONLINE G-COMP DOUBLE SUPPORT %%%%%%%%%%%%%%% 
// ///////////////////////////////////////////////////////////////
// //%%%%%%%%%%%%%% ONLINE G-COMP SINGLE SUPPORT %%%%%%%%%%%%%%%
// //Right Support Leg
// urefmat_R[0] =  gtorque_single[0]*(float(j8));
// urefmat_R[1] =  gtorque_single[1]*(float(j8+j7));
// urefmat_R[2] =  gtorque_single[2]*(float(j8+j7+j3));
// urefmat_R[3] =  gtorque_single[3]*(float(j8+j7+j3+j4));
// urefmat_R[4] =  gtorque_single[4]*(float(j8+j7+j3+j4+j12));
// urefmat_R[5] =  gtorque_single[5]*(float(j8+j7+j3+j4+j12+j13));
// //Left Support Leg
// urefmat_L[0] =  gtorque_single[0]*(float(j13));
// urefmat_L[1] =  gtorque_single[1]*(float(j13+j12));
// urefmat_L[2] =  gtorque_single[2]*(float(j13+j12+j4)); 
// urefmat_L[3] =  gtorque_single[3]*(float(j13+j12+j4+j3));
// urefmat_L[4] =  gtorque_single[4]*(float(j13+j12+j4+j3+j7));
// urefmat_L[5] =  gtorque_single[5]*(float(j13+j12+j4+j3+j7+j8));
// //%%%%%%%%%%%%%% ONLINE G-COMP SINGLE SUPPORT %%%%%%%%%%%%%%%   
// 
// ugc_temp_Doub[0] = 0.0; ugc_temp_Doub[1] = 0.0; ugc_temp_Doub[2] = 0.0; ugc_temp_Doub[3] = 0.0; ugc_temp_Doub[4] = 0.0; ugc_temp_Doub[5] = 0.0;
//    ugc_temp_R[0] = 0.0;    ugc_temp_R[1] = 0.0;    ugc_temp_R[2] = 0.0;    ugc_temp_R[3] = 0.0;    ugc_temp_R[4] = 0.0;    ugc_temp_R[5] = 0.0;
//    ugc_temp_L[0] = 0.0;    ugc_temp_L[1] = 0.0;    ugc_temp_L[2] = 0.0;    ugc_temp_L[3] = 0.0;    ugc_temp_L[4] = 0.0;    ugc_temp_L[5] = 0.0;
// 
// // TEMPORARY GRAVITY COMPENSATION CONSTITUENTS COMPUTATION //
// //%%%%%%%%%%%%%%%%% DOUBLE SUPPORT %%%%%%%%%%%%%%%%%%
// for(int ii=0; ii<3; ii++)
// {                        //A A K K H H
// ugc_temp_Doub[0] =  ugc_temp_Doub[0] + ucs_mat_Doub[0][ii]*urefmat_Doub[ii];
// ugc_temp_Doub[1] =  ugc_temp_Doub[1] + ucs_mat_Doub[1][ii]*urefmat_Doub[ii];
// ugc_temp_Doub[2] =  ugc_temp_Doub[2] + ucs_mat_Doub[2][ii]*urefmat_Doub[ii];
// ugc_temp_Doub[3] =  ugc_temp_Doub[3] + ucs_mat_Doub[3][ii]*urefmat_Doub[ii];
// ugc_temp_Doub[4] =  ugc_temp_Doub[4] + ucs_mat_Doub[4][ii]*urefmat_Doub[ii];
// ugc_temp_Doub[5] =  ugc_temp_Doub[5] + ucs_mat_Doub[5][ii]*urefmat_Doub[ii];
// }
// //%%%%%%%%%%%%%%%%% DOUBLE SUPPORT %%%%%%%%%%%%%%%%%%
// 
// //%%%%%%%%%%%%%%%%% SINGLE SUPPORT %%%%%%%%%%%%%%%%%%
// for(int ii=0; ii<6; ii++)
// {
// ugc_temp_R[0] =  ugc_temp_R[0] + ucs_mat_Sing[0][ii]*urefmat_R[ii];
// ugc_temp_R[1] =  ugc_temp_R[1] + ucs_mat_Sing[1][ii]*urefmat_R[ii];
// ugc_temp_R[2] =  ugc_temp_R[2] + ucs_mat_Sing[2][ii]*urefmat_R[ii];
// ugc_temp_R[3] =  ugc_temp_R[3] + ucs_mat_Sing[3][ii]*urefmat_R[ii];
// ugc_temp_R[4] =  ugc_temp_R[4] + ucs_mat_Sing[4][ii]*urefmat_R[ii];
// ugc_temp_R[5] =  ugc_temp_R[5] + ucs_mat_Sing[5][ii]*urefmat_R[ii];
// 
// ugc_temp_L[0] =  ugc_temp_L[0] + ucs_mat_Sing[0][ii]*urefmat_L[ii];
// ugc_temp_L[1] =  ugc_temp_L[1] + ucs_mat_Sing[1][ii]*urefmat_L[ii];
// ugc_temp_L[2] =  ugc_temp_L[2] + ucs_mat_Sing[2][ii]*urefmat_L[ii];
// ugc_temp_L[3] =  ugc_temp_L[3] + ucs_mat_Sing[3][ii]*urefmat_L[ii];
// ugc_temp_L[4] =  ugc_temp_L[4] + ucs_mat_Sing[4][ii]*urefmat_L[ii];
// ugc_temp_L[5] =  ugc_temp_L[5] + ucs_mat_Sing[5][ii]*urefmat_L[ii];     
// }
// //%%%%%%%%%%%%%%%%% SINGLE SUPPORT %%%%%%%%%%%%%%%%%%
// // TEMPORARY GRAVITY COMPENSATION CONSTITUENTS COMPUTATION //
// 
// //%%%%%%%%%%%%%%%%% PHASE SWITCHING %%%%%%%%%%%%%%%%%% 
// //RIGHT SUPPORT LEG
// int Sway_L = 12000, Sway_R = 11500;
// 
// if (desPos[5] >= Sway_R) 
// {    
//     
// Gff[0] = feed_forward_single[0] *(float(desPos[8])/100000);
// Gff[1] = feed_forward_single[1] *(float(desPos[7])/100000);
// Gff[2] = feed_forward_single[2] *(float(desPos[3])/100000);
// Gff[3] = feed_forward_single[3] *(float(desPos[4])/100000);
// Gff[4] = feed_forward_single[4] *(float(desPos[12])/100000);
// Gff[5] = feed_forward_single[5] *(float(desPos[13])/100000);    
//     
// gcompvolt[8] = int(ugc_temp_R[0]*1000) + int(Gff[0]*1000);
// gcompvolt[7] = int(ugc_temp_R[1]*1000) + int(Gff[1]*1000);
// gcompvolt[3] = int(ugc_temp_R[2]*1000) + int(Gff[2]*1000);
// gcompvolt[4] = int(ugc_temp_R[3]*1000) + int(Gff[3]*1000);
// gcompvolt[12]= int(ugc_temp_R[4]*1000) + int(Gff[4]*1000);
// gcompvolt[13]= int(ugc_temp_R[5]*1000) + int(Gff[5]*1000);
// 
// mot_pos[8] = int(posPID[6]); mot_pos[7] = int(posPID[7]); mot_pos[3] = int(posPID[8]);    //Right Ankle, Knee, Hip
// mot_pos[4] = int(posPID[9]); mot_pos[12]= int(posPID[10]); mot_pos[13]= int(posPID[11]);  //Left Hip, Knee, Ankle 
// 
// mot_vel[8] = int(posPID[18]); mot_vel[7] = int(posPID[19]); mot_vel[3] = int(posPID[20]);
// mot_vel[4] = int(posPID[21]); mot_vel[12]= int(posPID[22]); mot_vel[13]= int(posPID[23]); 
// 
// link_pos[8] = int(posPID[0]); link_pos[7] = int(posPID[1]); link_pos[3] = int(posPID[2]);  
// link_pos[4] = int(posPID[3]); link_pos[12]= int(posPID[4]); link_pos[13]= int(posPID[5]); 
// 
// link_vel[8] = int(posPID[12]); link_vel[7] = int(posPID[13]); link_vel[3] = int(posPID[14]);
// link_vel[4] = int(posPID[15]); link_vel[12]= int(posPID[16]); link_vel[13]= int(posPID[17]); 
// 
// }
// 
// //LEFT SUPPORT LEG
// else if (desPos[10] >= Sway_L)
// {
// 
// Gff[0] = feed_forward_single[0] *(float(desPos[13])/100000);
// Gff[1] = feed_forward_single[1] *(float(desPos[12])/100000);
// Gff[2] = feed_forward_single[2] *(float(desPos[4])/100000);
// Gff[3] = feed_forward_single[3] *(float(desPos[3])/100000);
// Gff[4] = feed_forward_single[4] *(float(desPos[7])/100000);
// Gff[5] = feed_forward_single[5] *(float(desPos[8])/100000);
// 
// gcompvolt[8] = int(ugc_temp_L[5]*1000) + int(Gff[5]*1000);
// gcompvolt[7] = int(ugc_temp_L[4]*1000) + int(Gff[4]*1000);
// gcompvolt[3] = int(ugc_temp_L[3]*1000) + int(Gff[3]*1000);
// gcompvolt[4] = int(ugc_temp_L[2]*1000) + int(Gff[2]*1000);
// gcompvolt[12]= int(ugc_temp_L[1]*1000) + int(Gff[1]*1000);
// gcompvolt[13]= int(ugc_temp_L[0]*1000) + int(Gff[0]*1000);      
// 
// mot_pos[8] = int(posPID[6]); mot_pos[7] = int(posPID[7]); mot_pos[3] = int(posPID[8]); 
// mot_pos[4] = int(posPID[9]); mot_pos[12]= int(posPID[10]); mot_pos[13]= int(posPID[11]); 
// 
// mot_vel[8] = int(posPID[18]); mot_vel[7] = int(posPID[19]); mot_vel[3] = int(posPID[20]);
// mot_vel[4] = int(posPID[21]); mot_vel[12]= int(posPID[22]); mot_vel[13]= int(posPID[23]); 
// 
// link_pos[8] = int(posPID[0]); link_pos[7] = int(posPID[1]); link_pos[3] = int(posPID[2]);
// link_pos[4] = int(posPID[3]); link_pos[12]= int(posPID[4]); link_pos[13]= int(posPID[5]); 
// 
// link_vel[8] = int(posPID[12]); link_vel[7] = int(posPID[13]); link_vel[3] = int(posPID[14]);
// link_vel[4] = int(posPID[15]); link_vel[12]= int(posPID[16]); link_vel[13]= int(posPID[17]); 
//    
// }
// 
// //DOUBLE SUPPORT 
// else
// {
//     
// Gff[0] = feed_forward_double[0]*(float(desPos[8])/100000);    //Right Ankle
// Gff[1] = feed_forward_double[1]*(float(desPos[7])/100000);    //Right Knee
// Gff[2] = feed_forward_double[2]*(float(desPos[3])/100000);    //Right Hip
// Gff[3] = feed_forward_double[2]*(float(desPos[4])/100000);    //Left Hip
// Gff[4] = feed_forward_double[1]*(float(desPos[12])/100000);   //Left Knee
// Gff[5] = feed_forward_double[0]*(float(desPos[13])/100000);   //Left Ankle
// 
// gcompvolt[8] = int(ugc_temp_Doub[0]*1000) + int(Gff[0]*1000); 
// gcompvolt[7] = int(ugc_temp_Doub[2]*1000) + int(Gff[1]*1000);
// gcompvolt[3] = int(ugc_temp_Doub[4]*1000) + int(Gff[2]*1000);
// gcompvolt[4] = int(ugc_temp_Doub[5]*1000) + int(Gff[3]*1000);
// gcompvolt[12]= int(ugc_temp_Doub[3]*1000) + int(Gff[4]*1000);
// gcompvolt[13]= int(ugc_temp_Doub[1]*1000) + int(Gff[5]*1000);
// 
// mot_pos[8] = int(posPID[6]); mot_pos[7] = int(posPID[7]); mot_pos[3] = int(posPID[8]); 
// mot_pos[4] = int(posPID[9]); mot_pos[12]= int(posPID[10]); mot_pos[13]= int(posPID[11]); 
// 
// mot_vel[8] = int(posPID[18]); mot_vel[7] = int(posPID[19]); mot_vel[3] = int(posPID[20]);
// mot_vel[4] = int(posPID[21]); mot_vel[12]= int(posPID[22]); mot_vel[13]= int(posPID[23]); 
// 
// link_pos[8] = int(posPID[0]); link_pos[7] = int(posPID[1]); link_pos[3] = int(posPID[2]);
// link_pos[4] = int(posPID[3]); link_pos[12]= int(posPID[4]); link_pos[13]= int(posPID[5]); 
// 
// link_vel[8] = int(posPID[12]); link_vel[7] = int(posPID[13]); link_vel[3] = int(posPID[14]);
// link_vel[4] = int(posPID[15]); link_vel[12]= int(posPID[16]); link_vel[13]= int(posPID[17]); 
//   
// }

#endif
