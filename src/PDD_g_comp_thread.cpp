#include <yarp/os/all.h>
#include <yarp/math/Math.h>
#include <stdio.h>

#include "PDD_g_comp_thread.h"

using namespace yarp::math;


PDD_g_comp_thread::PDD_g_comp_thread(  std::string module_prefix, 
                                       yarp::os::ResourceFinder rf, 
                                       std::shared_ptr< paramHelp::ParamHelperServer > ph) :
                                    torso("torso", module_prefix, "coman", true),
                                    left_leg("left_leg", module_prefix, "coman", true),
                                    right_leg("right_leg", module_prefix, "coman", true),
                                    
                                    q_desired_left_leg(JOINT_NUMBER, 2),
                                    q_desired_right_leg(JOINT_NUMBER, 2),
                                    
                                    left_single_left_leg_motor_position_gains((JOINT_NUMBER / LEGS_NUMBER), &posPID_left[9]),   // HIP, KNEE, ANKLE
                                    left_single_left_leg_motor_velocity_gains((JOINT_NUMBER / LEGS_NUMBER), &posPID_left[21]),  // HIP, KNEE, ANKLE
                                    left_single_right_leg_motor_position_gains((JOINT_NUMBER / LEGS_NUMBER), &posPID_left[6]),  // ANKLE, KNEE, HIP       
                                    left_single_right_leg_motor_velocity_gains((JOINT_NUMBER / LEGS_NUMBER), &posPID_left[18]), // ANKLE, KNEE, HIP  
                                    
                                    right_single_left_leg_motor_position_gains((JOINT_NUMBER / LEGS_NUMBER), &posPID_right[9]),   // HIP, KNEE, ANKLE
                                    right_single_left_leg_motor_velocity_gains((JOINT_NUMBER / LEGS_NUMBER), &posPID_right[21]),  // HIP, KNEE, ANKLE
                                    right_single_right_leg_motor_position_gains((JOINT_NUMBER / LEGS_NUMBER), &posPID_right[6]),  // ANKLE, KNEE, HIP       
                                    right_single_right_leg_motor_velocity_gains((JOINT_NUMBER / LEGS_NUMBER), &posPID_right[18]), // ANKLE, KNEE, HIP 
                                    
                                    double_leg_motor_position_gains((JOINT_NUMBER / LEGS_NUMBER), &posPID_double[3]),   // ANKLE, KNEE, HIP
                                    double_leg_motor_velocity_gains((JOINT_NUMBER / LEGS_NUMBER), &posPID_double[9]),   // ANKLE, KNEE, HIP
                                    
                                    urefmat_double((JOINT_NUMBER / LEGS_NUMBER), 0.0),
                                    ugc_double(JOINT_NUMBER, 0.0),
                                    ucs_yarp_mat_double(JOINT_NUMBER, (JOINT_NUMBER / LEGS_NUMBER)), 
                                    Gff_double(JOINT_NUMBER, 0.0),
                                    
                                    generic_thread( module_prefix, rf, ph )
{
}

bool PDD_g_comp_thread::custom_init()
{
    // initializing ucs yarp matrix for double support
    for(int i = 0; i < JOINT_NUMBER; i++) {
        yarp::sig::Vector actual_row;
        for(int j = 0 ; j < (JOINT_NUMBER / LEGS_NUMBER); j++) {
            actual_row.push_back(ucs_mat_Doub[i][j]);
        }
        ucs_yarp_mat_double.setRow(i, actual_row);
        actual_row.clear();
    }
    
    
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
    
    std::cout << "Desired Torque : " << desired_torque.toString() << std::endl;
    
        
    yarp::sig::Vector right_chain_ref(torque_right_leg);
    right_chain_ref[0] = desired_torque[3];
    right_chain_ref[3] = desired_torque[4];
    right_chain_ref[5] = desired_torque[5];
    
    yarp::sig::Vector left_chain_ref(torque_left_leg);
    left_chain_ref[0] = desired_torque[2];
    left_chain_ref[3] = desired_torque[1];
    left_chain_ref[5] = desired_torque[0];

    right_leg.move(right_chain_ref);
    left_leg.move(left_chain_ref);
   
}

void PDD_g_comp_thread::sense_legs()
{
    // sense left_leg
    left_leg.sensePosition(q_left_leg);
    left_leg.senseVelocity(q_dot_left_leg);
    left_leg.senseTorque(torque_left_leg);
    // sense right_leg
    right_leg.sensePosition(q_right_leg);
    right_leg.senseVelocity(q_dot_right_leg);
    right_leg.senseTorque(torque_right_leg);
    
    // DEBUG
//     std::cout << "Left Leg q : " << q_left_leg.toString() << std::endl;
//     std::cout << "Left Leg q_dot : " << q_dot_left_leg.toString() << std::endl;
//     std::cout << "Right Leg q : " << q_right_leg.toString() << std::endl;
//     std::cout << "Right Leg q_dot : " << q_dot_right_leg.toString() << std::endl;
}


void PDD_g_comp_thread::control_law()
{
    
//     yarp::sig::Vector double_support_pitch_vector(6);
//     double_support_pitch_vector[0] = q_left_leg[HIP_PITCH_JOINT_INDEX];
//     double_support_pitch_vector[1] = q_left_leg[KNEE_PITCH_JOINT_INDEX];
//     double_support_pitch_vector[2] = q_left_leg[ANKLE_PITCH_JOINT_INDEX];
//     double_support_pitch_vector[3] = q_right_leg[HIP_PITCH_JOINT_INDEX];
//     double_support_pitch_vector[4] = q_right_leg[KNEE_PITCH_JOINT_INDEX];
//     double_support_pitch_vector[5] = q_right_leg[ANKLE_PITCH_JOINT_INDEX];
    
    // Double Support
    // position
    yarp::sig::Vector double_support_pitch_position(JOINT_NUMBER);
    double_support_pitch_position[0] = q_left_leg[ANKLE_PITCH_JOINT_INDEX];
    double_support_pitch_position[1] = q_left_leg[KNEE_PITCH_JOINT_INDEX];
    double_support_pitch_position[2] = q_left_leg[HIP_PITCH_JOINT_INDEX];
    double_support_pitch_position[3] = q_right_leg[HIP_PITCH_JOINT_INDEX];
    double_support_pitch_position[4] = q_right_leg[KNEE_PITCH_JOINT_INDEX];
    double_support_pitch_position[5] = q_right_leg[ANKLE_PITCH_JOINT_INDEX];
    // velocity
    yarp::sig::Vector double_support_pitch_velocity(JOINT_NUMBER);
    double_support_pitch_velocity[0] = q_dot_left_leg[ANKLE_PITCH_JOINT_INDEX];
    double_support_pitch_velocity[1] = q_dot_left_leg[KNEE_PITCH_JOINT_INDEX];
    double_support_pitch_velocity[2] = q_dot_left_leg[HIP_PITCH_JOINT_INDEX];
    double_support_pitch_velocity[3] = q_dot_right_leg[HIP_PITCH_JOINT_INDEX];
    double_support_pitch_velocity[4] = q_dot_right_leg[KNEE_PITCH_JOINT_INDEX];
    double_support_pitch_velocity[5] = q_dot_right_leg[ANKLE_PITCH_JOINT_INDEX];
    // position gains
    yarp::sig::Vector double_support_position_gains(JOINT_NUMBER);
    double_support_position_gains[0] = ( double_leg_motor_position_gains[0] / 2 );
    double_support_position_gains[1] = ( double_leg_motor_position_gains[1] / 2 );
    double_support_position_gains[2] = ( double_leg_motor_position_gains[2] / 2 );
    double_support_position_gains[3] = ( double_leg_motor_position_gains[2] / 2 );
    double_support_position_gains[4] = ( double_leg_motor_position_gains[1] / 2 );
    double_support_position_gains[5] = ( double_leg_motor_position_gains[0] / 2 );
    // velocity gains
    yarp::sig::Vector double_support_velocity_gains(JOINT_NUMBER);
    double_support_velocity_gains[0] = ( double_leg_motor_velocity_gains[0] / 2 );
    double_support_velocity_gains[1] = ( double_leg_motor_velocity_gains[1] / 2 );
    double_support_velocity_gains[2] = ( double_leg_motor_velocity_gains[2] / 2 );
    double_support_velocity_gains[3] = ( double_leg_motor_velocity_gains[2] / 2 );
    double_support_velocity_gains[4] = ( double_leg_motor_velocity_gains[1] / 2 );
    double_support_velocity_gains[5] = ( double_leg_motor_velocity_gains[0] / 2 );
    
    //updating urefmat_double
    urefmat_double[0] = gtorque_double[0] * ( ( q_desired_left_leg[ANKLE_PITCH_JOINT_INDEX] + 
                                              q_desired_right_leg[ANKLE_PITCH_JOINT_INDEX] ) / 2 );
    
    urefmat_double[1] = gtorque_double[1] * ( ( q_desired_left_leg[ANKLE_PITCH_JOINT_INDEX] + 
                                              q_desired_right_leg[ANKLE_PITCH_JOINT_INDEX] +
                                              q_desired_left_leg[KNEE_PITCH_JOINT_INDEX] + 
                                              q_desired_right_leg[KNEE_PITCH_JOINT_INDEX] ) / 2 );
    
    urefmat_double[2] = gtorque_double[2] * ( ( q_desired_left_leg[ANKLE_PITCH_JOINT_INDEX] + 
                                              q_desired_right_leg[ANKLE_PITCH_JOINT_INDEX] +
                                              q_desired_left_leg[KNEE_PITCH_JOINT_INDEX] + 
                                              q_desired_right_leg[KNEE_PITCH_JOINT_INDEX] + 
                                              q_desired_left_leg[HIP_PITCH_JOINT_INDEX] + 
                                              q_desired_right_leg[HIP_PITCH_JOINT_INDEX]) / 2 );
    // updating ugc_double
    ugc_double.zero();
    ugc_double += ucs_yarp_mat_double * urefmat_double;
    
    // updating Gff_double
    Gff_double[0] = feed_forward_double[0] * q_desired_right_leg[ANKLE_PITCH_JOINT_INDEX];
    Gff_double[1] = feed_forward_double[1] * q_desired_right_leg[KNEE_PITCH_JOINT_INDEX];
    Gff_double[2] = feed_forward_double[2] * q_desired_right_leg[HIP_PITCH_JOINT_INDEX];
    Gff_double[3] = feed_forward_double[2] * q_desired_left_leg[HIP_PITCH_JOINT_INDEX];
    Gff_double[4] = feed_forward_double[1] * q_desired_left_leg[KNEE_PITCH_JOINT_INDEX];
    Gff_double[5] = feed_forward_double[0] * q_desired_left_leg[ANKLE_PITCH_JOINT_INDEX];
    
    
    // torques
    yarp::sig::Vector double_support_position_torque = double_support_pitch_position * double_support_position_gains;
    yarp::sig::Vector double_support_velocity_torque = double_support_pitch_velocity * double_support_velocity_gains;
    yarp::sig::Vector double_support_ff_torque = Gff_double + ugc_double;
    
    desired_torque = double_support_position_torque + double_support_velocity_torque + double_support_ff_torque;
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
