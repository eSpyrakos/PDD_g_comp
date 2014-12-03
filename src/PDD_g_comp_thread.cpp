#include <yarp/os/all.h>
#include <stdio.h>

#include "PDD_g_comp_thread.h"


PDD_g_comp_thread::PDD_g_comp_thread(  std::string module_prefix, 
                                       yarp::os::ResourceFinder rf, 
                                       std::shared_ptr< paramHelp::ParamHelperServer > ph) :
                                    torso("torso", module_prefix, "coman", true),
                                    left_leg("left_leg", module_prefix, "coman", true),
                                    right_leg("right_leg", module_prefix, "coman", true),
                                    generic_thread( module_prefix, rf, ph )
{
}

bool PDD_g_comp_thread::custom_init()
{
    // import the desired trajectory from .txt
    importTrajectory(traj_imported);
    
    // set the control mode on the chains
//     torso.setTorqueMode();
//     left_leg.setTorqueMode();
//     right_leg.setTorqueMode();
    torso.setPositionMode();
    left_leg.setPositionMode();
    right_leg.setPositionMode();
    return true;
}

void PDD_g_comp_thread::run()
{  
    sense_legs();
    control_law();
    
}

void PDD_g_comp_thread::sense_legs()
{
    // sense left_leg
    left_leg.sensePosition(q_left_leg);
    left_leg.senseVelocity(q_dot_left_leg);
    // sense right_leg
    right_leg.sensePosition(q_right_leg);
    right_leg.senseVelocity(q_dot_right_leg);
    
    // DEBUG
    std::cout << "Left Leg q : " << q_left_leg.toString() << std::endl;
    std::cout << "Left Leg q_dot : " << q_dot_left_leg.toString() << std::endl;
    std::cout << "Right Leg q : " << q_right_leg.toString() << std::endl;
    std::cout << "Right Leg q_dot : " << q_dot_right_leg.toString() << std::endl;
}


void PDD_g_comp_thread::control_law()
{
    
}


bool PDD_g_comp_thread::importTrajectory(double traj[LOWER_BODY_DOF][TRAJ_SAMPLES])
{
	FILE *datafile;

	datafile=fopen("RefJointAngles.txt","r");
	if(datafile != NULL){		
		for(int p=0;p<TRAJ_POINTS;p++){
		    fscanf(datafile,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
			    &traj[0][p],&traj[1][p],&traj[2][p],						// torso
			    &traj[3][p],&traj[5][p],&traj[6][p],&traj[7][p],&traj[8][p],&traj[9][p],		// right leg
			    &traj[4][p],&traj[10][p],&traj[11][p],&traj[12][p],&traj[13][p],&traj[14][p]);	// left leg
		}	
	}
	else {
	  std::cout << "File RefJointAngles.txt not opened correctly" << std::endl;
	  fclose(datafile);
	  return false;
	}
	
	fclose(datafile);
	return true;
}
