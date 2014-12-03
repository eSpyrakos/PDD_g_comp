#ifndef PDD_g_comp_THREAD_H_
#define PDD_g_comp_THREAD_H_

#include <math.h>
#include <GYM/generic_thread.hpp>
#include <idynutils/yarp_single_chain_interface.h>

#include "PDD_g_comp_constants.h"


#define PI 3.14159265
#define TRAJ_SAMPLES 50000
#define SAMPLES 1000000

/*


int gcompvolt[30]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
// UDP GAIN ARRAYS
int mot_pos[30]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int mot_vel[30]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int link_pos[30]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int link_vel[30]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


	
float Gff[6];
float urefmat_L[6];
float urefmat_R[6];
float urefmat_Doub[6];
float ugc_temp_L[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float ugc_temp_R[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float ugc_temp_Doub[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
char Gcomp_Gains[30];



	if(loop%trajdt==0)
		{
			tau = curSimTime;
			//onlineTrajectory();
		}

		r = SetDesiredVelocity(desVel);
		if (loop == trajPoints-1)
		{
			loop = 0;	
		}
		for (int ii = 0; ii<15; ii++)
		{
			desPos[ii] = traj[ii][loop]; 

		}
		r = SetDesiredPosition(desPos);

		j3 = joint[3].absPos1; j4 = joint[4].absPos1; j7 = joint[7].absPos1;
		j8 = joint[8].absPos1; j12 = joint[12].absPos1; j13 = joint[13].absPos1;

		////%%%%%%%%%%%%%% ONLINE G-COMP DOUBLE SUPPORT %%%%%%%%%%%%%%%
		urefmat_Doub[0] =   25.7516*(float((j8+j13)/2)/100000);
		urefmat_Doub[1] =   19.6539*(float((j8+j13+j7+j12)/2)/100000);
		urefmat_Doub[2] =   5.8693*(float((j8+j13+j7+j12+j4+j3)/2)/100000); 
		////%%%%%%%%%%%%%% ONLINE G-COMP DOUBLE SUPPORT %%%%%%%%%%%%%%%   

		//%%%%%%%%%%%%%% ONLINE G-COMP SINGLE SUPPORT %%%%%%%%%%%%%%%
		//Right Support Leg
		urefmat_R[0] =  30.1376*(float(j8)/100000);
		urefmat_R[1] = 	29.0724*(float(j8+j7)/100000);
		urefmat_R[2] = 	5.9147*(float(j8+j7+j3)/100000);
		urefmat_R[3] = 	-1*9.4133*(float(j8+j7+j3+j4)/100000);
		urefmat_R[4] = 	-1*4.3860*(float(j8+j7+j3+j4+j12)/100000);
		urefmat_R[5] = 	-1*0.3076*(float(j8+j7+j3+j4+j12+j13)/100000);
		//Left Support Leg
		urefmat_L[0] =  30.1376*(float(j13)/100000);
		urefmat_L[1] = 	29.0724*(float(j13+j12)/100000);
		urefmat_L[2] = 	5.9147*(float(j13+j12+j4)/100000); 
		urefmat_L[3] = 	-1*9.4133*(float(j13+j12+j4+j3)/100000);
		urefmat_L[4] = 	-1*4.3860*(float(j13+j12+j4+j3+j7)/100000);
		urefmat_L[5] = 	-1*0.3076*(float(j13+j12+j4+j3+j7+j8)/100000);
		//%%%%%%%%%%%%%% ONLINE G-COMP SINGLE SUPPORT %%%%%%%%%%%%%%%		

		ugc_temp_Doub[0] = 0.0; ugc_temp_Doub[1] = 0.0; ugc_temp_Doub[2] = 0.0; ugc_temp_Doub[3] = 0.0; ugc_temp_Doub[4] = 0.0; ugc_temp_Doub[5] = 0.0;
		ugc_temp_R[0] = 0.0; ugc_temp_R[1] = 0.0; ugc_temp_R[2] = 0.0; ugc_temp_R[3] = 0.0; ugc_temp_R[4] = 0.0; ugc_temp_R[5] = 0.0;
		ugc_temp_L[0] = 0.0; ugc_temp_L[1] = 0.0; ugc_temp_L[2] = 0.0; ugc_temp_L[3] = 0.0; ugc_temp_L[4] = 0.0; ugc_temp_L[5] = 0.0; 

		//%%%%%%%%%%%%%%%%% DOUBLE SUPPORT %%%%%%%%%%%%%%%%%%
		for(int ii=0; ii<3; ii++)
		{
			//A A K K H H
			ugc_temp_Doub[0] =  ugc_temp_Doub[0] + ucs_mat_Doub[0][ii]*urefmat_Doub[ii];
			ugc_temp_Doub[1] =  ugc_temp_Doub[1] + ucs_mat_Doub[1][ii]*urefmat_Doub[ii];
			ugc_temp_Doub[2] =  ugc_temp_Doub[2] + ucs_mat_Doub[2][ii]*urefmat_Doub[ii];
			ugc_temp_Doub[3] =  ugc_temp_Doub[3] + ucs_mat_Doub[3][ii]*urefmat_Doub[ii];
			ugc_temp_Doub[4] =  ugc_temp_Doub[4] + ucs_mat_Doub[4][ii]*urefmat_Doub[ii];
			ugc_temp_Doub[5] =  ugc_temp_Doub[5] + ucs_mat_Doub[5][ii]*urefmat_Doub[ii];
		}
		//%%%%%%%%%%%%%%%%% DOUBLE SUPPORT %%%%%%%%%%%%%%%%%%

		//%%%%%%%%%%%%%%%%% SINGLE SUPPORT %%%%%%%%%%%%%%%%%%
		for(int ii=0; ii<6; ii++)
		{
			ugc_temp_R[0] =  ugc_temp_R[0] + ucs_mat_Sing[0][ii]*urefmat_R[ii];
			ugc_temp_R[1] =  ugc_temp_R[1] + ucs_mat_Sing[1][ii]*urefmat_R[ii];
			ugc_temp_R[2] =  ugc_temp_R[2] + ucs_mat_Sing[2][ii]*urefmat_R[ii];
			ugc_temp_R[3] =  ugc_temp_R[3] + ucs_mat_Sing[3][ii]*urefmat_R[ii];
			ugc_temp_R[4] =  ugc_temp_R[4] + ucs_mat_Sing[4][ii]*urefmat_R[ii];
			ugc_temp_R[5] =  ugc_temp_R[5] + ucs_mat_Sing[5][ii]*urefmat_R[ii];

			ugc_temp_L[0] =  ugc_temp_L[0] + ucs_mat_Sing[0][ii]*urefmat_L[ii];
			ugc_temp_L[1] =  ugc_temp_L[1] + ucs_mat_Sing[1][ii]*urefmat_L[ii];
			ugc_temp_L[2] =  ugc_temp_L[2] + ucs_mat_Sing[2][ii]*urefmat_L[ii];
			ugc_temp_L[3] =  ugc_temp_L[3] + ucs_mat_Sing[3][ii]*urefmat_L[ii];
			ugc_temp_L[4] =  ugc_temp_L[4] + ucs_mat_Sing[4][ii]*urefmat_L[ii];
			ugc_temp_L[5] =  ugc_temp_L[5] + ucs_mat_Sing[5][ii]*urefmat_L[ii];	
		}
		//%%%%%%%%%%%%%%%%% SINGLE SUPPORT %%%%%%%%%%%%%%%%%%

		//%%%%%%%%%%%%%%%%% PHASE SWITCHING %%%%%%%%%%%%%%%%%% 
		//RIGHT SUPPORT LEG
		int P_S_R = 1, P_S_L = 1, SW_L = -50, SW_R = -30, Sway_L = 12000, Sway_R = 11500;
		//if (S_VAL_R >= P_S_R && FTSensor[2].mfz > SW_R && desPos[5] > Sway_Temp) 
		if (desPos[5] >= Sway_R) 
		{


		Gff[0] = 4.472007e+001 *(float(desPos[8])/100000);
		Gff[1] = 3.654815e+001 *(float(desPos[7])/100000);
		Gff[2] = 3.655567e+001 *(float(desPos[3])/100000);
		Gff[3] = 8.893022e+001 *(float(desPos[4])/100000);
		Gff[4] = 5.758744e+001 *(float(desPos[12])/100000);
		Gff[5] = 9.937348e+001 *(float(desPos[13])/100000);

		gcompvolt[8] = int(ugc_temp_R[0]*1000) + int(Gff[0]*1000);
		gcompvolt[7] = int(ugc_temp_R[1]*1000) + int(Gff[1]*1000);
		gcompvolt[3] = int(ugc_temp_R[2]*1000) + int(Gff[2]*1000);
		gcompvolt[4] = int(ugc_temp_R[3]*1000) + int(Gff[3]*1000);
		gcompvolt[12]= int(ugc_temp_R[4]*1000) + int(Gff[4]*1000);
		gcompvolt[13]= int(ugc_temp_R[5]*1000) + int(Gff[5]*1000);

		mot_pos[8] = int(posPID[6]); mot_pos[7] = int(posPID[7]); mot_pos[3] = int(posPID[8]); 
		mot_pos[4] = int(posPID[9]); mot_pos[12]= int(posPID[10]); mot_pos[13]= int(posPID[11]); 

		mot_vel[8] = int(posPID[18]); mot_vel[7] = int(posPID[19]); mot_vel[3] = int(posPID[20]);
		mot_vel[4] = int(posPID[21]); mot_vel[12]= int(posPID[22]); mot_vel[13]= int(posPID[23]); 

		link_pos[8] = int(posPID[0]); link_pos[7] = int(posPID[1]); link_pos[3] = int(posPID[2]);
		link_pos[4] = int(posPID[3]); link_pos[12]= int(posPID[4]); link_pos[13]= int(posPID[5]); 

		link_vel[8] = int(posPID[12]); link_vel[7] = int(posPID[13]); link_vel[3] = int(posPID[14]);
		link_vel[4] = int(posPID[15]); link_vel[12]= int(posPID[16]); link_vel[13]= int(posPID[17]); 

		r = SetGComp(gcompvolt);
		r = SetLQRLinkGains(link_pos,link_vel); 
		r = SetLQRMotorGains(mot_pos,mot_vel);   

		}
		//LEFT SUPPORT LEG
		else if (desPos[10] >= Sway_L)
		//else if (S_VAL_L >= P_S_L && FTSensor[5].mfz > SW_L && desPos[10] > Sway)
		{


		Gff[0] = 4.472007e+001 *(float(desPos[13])/100000);
		Gff[1] = 3.654815e+001 *(float(desPos[12])/100000);
		Gff[2] = 3.655567e+001 *(float(desPos[4])/100000);
		Gff[3] = 8.893022e+001 *(float(desPos[3])/100000);
		Gff[4] = 5.758744e+001 *(float(desPos[7])/100000);
		Gff[5] = 9.937348e+001 *(float(desPos[8])/100000);

		gcompvolt[8] = int(ugc_temp_L[5]*1000) + int(Gff[5]*1000);
		gcompvolt[7] = int(ugc_temp_L[4]*1000) + int(Gff[4]*1000);
		gcompvolt[3] = int(ugc_temp_L[3]*1000) + int(Gff[3]*1000);
		gcompvolt[4] = int(ugc_temp_L[2]*1000) + int(Gff[2]*1000);
		gcompvolt[12]= int(ugc_temp_L[1]*1000) + int(Gff[1]*1000);
		gcompvolt[13]= int(ugc_temp_L[0]*1000) + int(Gff[0]*1000);	

		mot_pos[8] = int(posPID[6]); mot_pos[7] = int(posPID[7]); mot_pos[3] = int(posPID[8]); 
		mot_pos[4] = int(posPID[9]); mot_pos[12]= int(posPID[10]); mot_pos[13]= int(posPID[11]); 

		mot_vel[8] = int(posPID[18]); mot_vel[7] = int(posPID[19]); mot_vel[3] = int(posPID[20]);
		mot_vel[4] = int(posPID[21]); mot_vel[12]= int(posPID[22]); mot_vel[13]= int(posPID[23]); 

		link_pos[8] = int(posPID[0]); link_pos[7] = int(posPID[1]); link_pos[3] = int(posPID[2]);
		link_pos[4] = int(posPID[3]); link_pos[12]= int(posPID[4]); link_pos[13]= int(posPID[5]); 

		link_vel[8] = int(posPID[12]); link_vel[7] = int(posPID[13]); link_vel[3] = int(posPID[14]);
		link_vel[4] = int(posPID[15]); link_vel[12]= int(posPID[16]); link_vel[13]= int(posPID[17]); 

		r = SetGComp(gcompvolt);
		r = SetLQRLinkGains(link_pos,link_vel); 
		r = SetLQRMotorGains(mot_pos,mot_vel);   
		}
		//DOUBLE SUPPORT 
		//else if (S_VAL_L < P_S_R && S_VAL_R < P_S_R && FTSensor[2].mfz <= SW_L && FTSensor[5].mfz <= SW_L)
		else
		{

		Gff[0] = 54.2535*(float(desPos[8])/100000);    //Right Ankle
		Gff[1] = 29.2865*(float(desPos[7])/100000);    //Right Knee
		Gff[2] = 27.8388*(float(desPos[3])/100000);	   //Right Hip
		Gff[3] = 27.8388*(float(desPos[4])/100000);	   //Left Hip
		Gff[4] = 29.2865*(float(desPos[12])/100000);   //Left Knee
		Gff[5] = 54.2535*(float(desPos[13])/100000);   //Left Ankle

                gcompvolt[8] = int(ugc_temp_Doub[0]*1000) + int(Gff[0]*1000); 
		gcompvolt[7] = int(ugc_temp_Doub[2]*1000) + int(Gff[1]*1000);
		gcompvolt[3] = int(ugc_temp_Doub[4]*1000) + int(Gff[2]*1000);
		gcompvolt[4] = int(ugc_temp_Doub[5]*1000) + int(Gff[3]*1000);
		gcompvolt[12]= int(ugc_temp_Doub[3]*1000) + int(Gff[4]*1000);
		gcompvolt[13]= int(ugc_temp_Doub[1]*1000) + int(Gff[5]*1000);

		mot_pos[8] = int(posPID[6]); mot_pos[7] = int(posPID[7]); mot_pos[3] = int(posPID[8]); 
		mot_pos[4] = int(posPID[9]); mot_pos[12]= int(posPID[10]); mot_pos[13]= int(posPID[11]); 

		mot_vel[8] = int(posPID[18]); mot_vel[7] = int(posPID[19]); mot_vel[3] = int(posPID[20]);
		mot_vel[4] = int(posPID[21]); mot_vel[12]= int(posPID[22]); mot_vel[13]= int(posPID[23]); 

		link_pos[8] = int(posPID[0]); link_pos[7] = int(posPID[1]); link_pos[3] = int(posPID[2]);
		link_pos[4] = int(posPID[3]); link_pos[12]= int(posPID[4]); link_pos[13]= int(posPID[5]); 

		link_vel[8] = int(posPID[12]); link_vel[7] = int(posPID[13]); link_vel[3] = int(posPID[14]);
		link_vel[4] = int(posPID[15]); link_vel[12]= int(posPID[16]); link_vel[13]= int(posPID[17]); 

		r = SetGComp(gcompvolt);
		r = SetLQRLinkGains(link_pos,link_vel); 
		r = SetLQRMotorGains(mot_pos,mot_vel);   
		}*/


/**
 * @brief PDD_g_comp control thread
 * 
 **/
class PDD_g_comp_thread : public generic_thread
{
private:
    walkman::yarp_single_chain_interface torso;
    walkman::yarp_single_chain_interface left_leg;
    walkman::yarp_single_chain_interface right_leg;
    
    double traj_imported[LOWER_BODY_DOF][TRAJ_SAMPLES];
    
    
    yarp::sig::Vector q_left_leg;
    yarp::sig::Vector q_dot_left_leg;
    
    yarp::sig::Vector q_right_leg;
    yarp::sig::Vector q_dot_right_leg;
    


    /**
    * @brief ...
    * 
    * @param traj ...
    * @return bool
    */
    bool importTrajectory(double traj[LOWER_BODY_DOF][TRAJ_SAMPLES]);
 
public:
    
    /**
     * @brief constructor
     * 
     * @param module_prefix the prefix of the module
     * @param rf resource finderce
     * @param ph param helper
     */
     PDD_g_comp_thread( std::string module_prefix, yarp::os::ResourceFinder rf, std::shared_ptr<paramHelp::ParamHelperServer> ph );
    
    
    /**
     * @brief PDD_g_comp control thread initialization
     * 
     * @return true on succes, false otherwise
     */
    virtual bool custom_init();
    
    /**
     * @brief PDD_g_comp control thread main loop
     * 
     */
    virtual void run();
    
    void sense_legs();
    
    void control_law();
    
};

#endif
