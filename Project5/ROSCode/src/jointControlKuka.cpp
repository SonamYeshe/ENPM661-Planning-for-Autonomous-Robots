// "Copyright [2017] <Michael Kam>"
/** @file jointControlKuka.cpp
 *  @brief This jointControlKuka.cpp is a ros node that use to control the iiwa in gazebo
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 *
 *  jointControlKuka is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  jointControlKuka is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with jointControlKuka.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include <kukaControl.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <robotic_polishing/Trajectory.h>
#include <geometry_msgs/Twist.h>
#include <iiwa_msgs/JointPosition.h>
#include <sensor_msgs/JointState.h>
#include <kdl/chain.hpp>
#include "dijkstraPQ.h"
// #include "Eigen/Core"
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <string>
#include <sstream>
#include <cmath>
#include <vector>

sensor_msgs::JointState joints;
bool initialized = false;
double threshold = 0.005;
#define PI 3.14159265
position test1, test2;

//--------------------------------------------Callback of reading the joint position ---------------------//

// callback for reading joint values
/** @brief get_joints is a callback function that subscribes the joint position data
 * and set it into joints vector. You can think of this as a feedback.
 *  @param[in] data sensor_msgs::JointState that contains joint position
 *  @return none
 */
void get_joints(const sensor_msgs::JointState & data) {
	for (int i = 0; i < data.position.size(); i++) {
		// if this is not the first time the callback function is read, obtain the joint positions
		if (initialized) {
			joints.position[i] = data.position[i];
			// otherwise initilize them with 0.0 values
		} else {
			joints.position.push_back(0.0);
		}
	}
	initialized = true;
}

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "joint");
	ros::NodeHandle nh_;
	ros::NodeHandle home("~");

	//--------------------------------------------(DO NOT TOUCH) Initialization for the KDL --------------------------//
	kukaControl kc;

	KDL::Chain chain = kc.LWR_HSC_nozzle();
	KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(
			chain); // define the forward kinematic solver via the defined chain
	KDL::ChainIkSolverVel_pinv iksolverv = KDL::ChainIkSolverVel_pinv(chain); // Inverse velocity solver
	KDL::ChainIkSolverPos_NR iksolver(chain, fksolver, iksolverv, 100, 1e-6); // Maximum 100 iterations, stop at accuracy 1e-6

	//---2018/05/03 Michael add rcm-------------------------------------------------------------------------------------
	// KDL::Chain chainRCM = kc.LWR_HSC_nozzle_RCM_used();
  kc.setRCMtoollength(0.0);
	//KDL::ChainIkSolverPos_NR iksolverRCM = kc.updateChainIKsolver(fksolverRCM, kc.LWR_HSC_nozzle_RCM_used());
	//fksolverRCM = kc.updateChainFKsolver(kc.LWR_HSC_nozzle_RCM_used());
	//iksolverRCM = kc.updateChainIKsolver(fksolverRCM, kc.LWR_HSC_nozzle_RCM_used());
	KDL::ChainFkSolverPos_recursive fksolverRCM =
			KDL::ChainFkSolverPos_recursive(kc.LWR_HSC_nozzle_RCM_used()); // define the forward kinematic solver via the defined chain
	KDL::ChainIkSolverVel_pinv iksolvervRCM = KDL::ChainIkSolverVel_pinv(
			kc.LWR_HSC_nozzle_RCM_used());  // Inverse velocity solver
	KDL::ChainIkSolverPos_NR iksolverRCM(kc.LWR_HSC_nozzle_RCM_used(),
			fksolverRCM, iksolvervRCM, 100, 1e-6); // Maximum 100 iterations, stop at accuracy 1e-6

	//-----------------------------------------------------------------------------------------------------------
  // kc.setRCMtoollength(0.2); 2018.05.03 The function did not change the kc
	unsigned int nj = chain.getNrOfJoints(); // get the number of joints from the chain
	KDL::JntArray jointpositions = KDL::JntArray(nj); // define a joint array in KDL format for the joint positions
	KDL::JntArray jointpositions_new = KDL::JntArray(nj); // define a joint array in KDL format for the next joint positions
	KDL::JntArray joint_ref = KDL::JntArray(nj);
	trajectory_msgs::JointTrajectory joint_cmd;
	trajectory_msgs::JointTrajectoryPoint pt;
	kc.initializePoints(pt, nj, 0.0);
	kc.nameJoints(joint_cmd, nj);
	kc.initializeJoints(jointpositions);

	// subscribe to joint topic and send command to command the robot
	ros::Publisher cmd_pub = nh_.advertise<trajectory_msgs::JointTrajectory>(
			"iiwa/PositionJointInterface_trajectory_controller/command", 10);
	ros::Subscriber joints_sub = nh_.subscribe("/iiwa/joint_states", 10,
			get_joints);
	// set the parameter of the ros loop
	int loop_freq = 10;
	float dt = static_cast<float>(1) / loop_freq; // 2017.12.15 Bug fix! float dt = static_cast<float>(1/ loop_freq);
	ros::Rate loop_rate(loop_freq);

	// Other parameters to set
	KDL::Frame cartpos, cartposTool;
	geometry_msgs::Twist xyz;
	bool kinematics_status;
	//--------------------------------------------(DO NOT TOUCH) End of initialization for the KDL ------------------//

	position rcm_point;
	/*
	rcm_point.x = 0.04;
	rcm_point.y = -0.550;
	rcm_point.z = 0.35;
	*/
	std::vector<position> test_ref_before;
	std::vector<position> test_ref;
	std::vector<position> return_ref;
	// (TODO)Read the Data point from the launch file
	float xStart, yStart, zStart, xEnd, yEnd, zEnd;
	double roll, pitch, yaw, x, y, z;
	int iterationMax;
	iterationMax = 1;
	roll = 3.141084;
	pitch = 0;
	yaw = 1.571;
	KDL::Rotation rpy = KDL::Rotation::RPY(roll, pitch, yaw);
	test1.z = 0.02;

  //--------------------------------------------(TODO)Read the Data point from the launch file ---------------------//
    // 2018 05 04 Michael
    rcm_point.x = -0.02;
    rcm_point.y = -0.480;
    rcm_point.z = 0.25;
	  std::ifstream file("path.txt");
	  std::vector<float> r;
	  std::string word;
	  while (file >> word) {
	    r.push_back(atof(word.c_str()));
	    //std::cout << atof(word.c_str()) << std::endl;
	  }

	   int sizeData=r.size();
	   int ccp_iteration = sizeData / 3;
	   for (int i = 0; i < ccp_iteration; i++) {
	       test1.x = r[3 * i]/1000;
	       test1.y = 0.1+r[3 * i + 1]/1000;
	       test_ref_before.push_back(test1);
	     }


  //----------------------------------------------------------------------------------------------------------------//
/*
	for(double bb=0;bb<40;bb++)
	{
	  test1.x = 0+ bb * 0.001;
	  test1.y = -0.52;
	  test_ref_before.push_back(test1);

	}
*/


/*   test1.x = 0.0;
   test1.y = -0.550;
	 test_ref_before.push_back(test1);

	 test1.x = 0.02;
	 test1.y = -0.550;
	 test_ref_before.push_back(test1);

	 test1.x = 0.04;
	 test1.y = -0.550;
	 test_ref_before.push_back(test1);
*/
	double x_new, y_new, delta_theta_rotate_Z, delta_theta_rotate_Y, L, H;

	KDL::Rotation rpy_buffer;
	std::vector<position>::iterator now = test_ref_before.begin();
	while (now != test_ref_before.end()) {

		/* -----------------Rotation part-----------------------------
		 * 1. rcm incert point are always at the same location.
		 *
		 *
		 *
		 */

	  test2.x = rcm_point.x;
		test2.y = rcm_point.y;
		test2.z = rcm_point.z;
		test2.rpyM = rpy;
		H = rcm_point.z-test1.z;
		y_new = now->x - rcm_point.x;  // y_new
		x_new = now->y - rcm_point.y;  // x_new
		delta_theta_rotate_Z = atan2(y_new, x_new);
		std::cout << "Angle Z for point ( " << now->x << " , " << now->y
				<< " )= " << delta_theta_rotate_Z << std::endl;
		L = sqrt(pow(y_new, 2) + pow(x_new, 2));
		delta_theta_rotate_Y = atan2(L, H);
		std::cout << "Angle X for point ( " << now->x << " , " << now->y
				<< " )= " << delta_theta_rotate_Y << std::endl;
		std::cout << std::endl;
		test2.rpyM.DoRotZ(delta_theta_rotate_Z);
		test2.rpyM.DoRotY(delta_theta_rotate_Y);
		test2.rotationOrTranslation = 1; // using rcm kinematic chain
		test_ref.push_back(test2);

		/* -----------------Translation part-----------------------------
		 *  1. get the orientation from the rotation part
		 *  2. get the points from the test_ref_before
		 *  3. push them into the test_ref
		 *
		 */

		 test2.x=now->x;
		 test2.y=now->y;
		 test2.z=now->z;
		 // test2.rpyM used the same in previous
		 test2.rotationOrTranslation = 2;
		 test_ref.push_back(test2);
		++now;
	}

//-----------return position-------------//
	/*
	 test1.x = 0.08;
	 test1.y = -0.62;
	 test1.z = 0.0;
	 test1.nx = 3.131084;
	 test1.ny = -0.004296;
	 test1.nz = 1.593423;
	 return_ref.push_back(test1);

	 test1.x = 0.0;
	 test1.y = -0.62;
	 test1.z = 0.35;
	 test1.nx = 3.131084;
	 test1.ny = -0.004296;
	 test1.nz = 1.593423;
	 return_ref.push_back(test1);
	 */
	// (TODO) Start point is the rcm point to insert. You should move the robot to the point,
	//float xStart, yStart, zStart, xEnd, yEnd, zEnd;
	//double roll, pitch, yaw, x, y, z;
	//int iterationMax;
	iterationMax = 1;
	// ***********************************************************************************/
	ROS_INFO("Set command cartpos configuration");
	// start point
	x = rcm_point.x;
	y = rcm_point.y;
	z = rcm_point.z;
	//roll = 3.131084;
	//pitch = 0;
	//yaw = 0;

	pt.time_from_start = ros::Duration(3.0);
	rpy = KDL::Rotation::RPY(roll, pitch, yaw); // Rotation built from Roll-Pitch-Yaw angles
	cartpos.p[0] = x;
	cartpos.p[1] = y;
	cartpos.p[2] = z;
	cartpos.M = rpy; // 2017.12.10 Bug here!!!!!!! Causing IK failed...... because you did not initialize it.

	// "Set current joint configuration before IK"
	int ik_error = iksolverRCM.CartToJnt(jointpositions, cartpos,
			jointpositions_new);
	ROS_INFO("ik_error= %d", ik_error);
	kc.evalPoints(pt, jointpositions_new, nj);
	pt.time_from_start = ros::Duration(1.0);
	joint_cmd.points.push_back(pt);
	pt.time_from_start = ros::Duration(1.0);
	joint_cmd.points[0] = pt;
	joint_cmd.header.stamp = ros::Time::now();
	kinematics_status = fksolverRCM.JntToCart(jointpositions_new, cartpos);
	joints.position.push_back(0.0);

	joint_ref = jointpositions_new;
	std::vector<position>::iterator now_cmd = test_ref.begin();
	std::vector<position>::iterator return_cmd = return_ref.begin();
	bool arrived = false;
	double d0, d1, d2, d3, d4, d5, d6;
	double d01, d02, d03, d04, d05, d06, d07;
	int iterationTime = 0;

	/*************************************************
	 *   fksolverRCM.~ChainFkSolverPos_recursive();
       iksolvervRCM.~ChainIkSolverVel_pinv();
       iksolverRCM.~ChainIkSolverPos_NR();
	 *************************************************/
	       fksolverRCM.~ChainFkSolverPos_recursive();
	       iksolvervRCM.~ChainIkSolverVel_pinv();
	       iksolverRCM.~ChainIkSolverPos_NR();
         std::cout << "Yo! you kill three of us" << std::endl;
	while (ros::ok()) {

		if (iterationTime != iterationMax) {
			if (now_cmd != test_ref.end()) { // 2017.12.12 Can't add while inside the while
					// if joint error<0.001
				ROS_INFO("joints.position[0]= %f", joints.position[0]);
				ROS_INFO("joints.position[1]= %f", joints.position[1]);
				ROS_INFO("joints.position[2]= %f", joints.position[2]);
				ROS_INFO("joints.position[3]= %f", joints.position[3]);
				ROS_INFO("joints.position[4]= %f", joints.position[4]);
				ROS_INFO("joints.position[5]= %f", joints.position[5]);
				ROS_INFO("joints.position[6]= %f\n", joints.position[6]);

				d0 = joint_ref(0) - joints.position[0];
				d01 = fabs(d0);
				d1 = joint_ref(1) - joints.position[1];
				d02 = fabs(d1);
				d2 = joint_ref(2) - joints.position[2];
				d03 = fabs(d2);
				d3 = joint_ref(3) - joints.position[3];
				d04 = fabs(d3);
				d4 = joint_ref(4) - joints.position[4];
				d05 = fabs(d4);
				d5 = joint_ref(5) - joints.position[5];
				d06 = fabs(d5);
				d6 = joint_ref(6) - joints.position[6];
				d07 = fabs(d6);

				ROS_INFO("joint_ref(0)= %f", joint_ref(0));
				ROS_INFO("joint_ref(1)= %f", joint_ref(1));
				ROS_INFO("joint_ref(2)= %f", joint_ref(2));
				ROS_INFO("joint_ref(3)= %f", joint_ref(3));
				ROS_INFO("joint_ref(4)= %f", joint_ref(4));
				ROS_INFO("joint_ref(5)= %f", joint_ref(5));
				ROS_INFO("joint_ref(6)= %f", joint_ref(6));

				ROS_INFO("d0= %f", d01);
				ROS_INFO("d1= %f", d02);
				ROS_INFO("d2= %f", d03);
				ROS_INFO("d3= %f", d04);
				ROS_INFO("d4= %f", d05);
				ROS_INFO("d5= %f", d06);
				ROS_INFO("d6= %f", d07);

				if (d01 < threshold && d02 < threshold && d03 < threshold
						&& d04 < threshold && d05 < threshold && d06 < threshold
						&& d07 < threshold) {
					arrived = true;
				} else {
					arrived = false;
				}
				if (arrived) {  //{ take out element
					cartpos.p[0] = now_cmd->x;
					cartpos.p[1] = now_cmd->y;
					cartpos.p[2] = now_cmd->z;
					ROS_INFO("-------------------------------Here is the next coordinate to run---------------------------------------------- ");
					ROS_INFO("cartpos.p[0]= %f", cartpos.p[0]);
					ROS_INFO("cartpos.p[1]= %f", cartpos.p[1]);
					ROS_INFO("cartpos.p[2]= %f", cartpos.p[2]);
					// rpy = KDL::Rotation::RPY(now_cmd->nx, now_cmd->ny, now_cmd->nz);
					ROS_INFO("roll= %f", now_cmd->nx);
					ROS_INFO("pitch= %f", now_cmd->ny);
					ROS_INFO("yaw= %f", now_cmd->nz);
					// cartpos.M = rpy;  // rotation
					cartpos.M = now_cmd->rpyM;
					for (int k = 0; k < nj; k++) {
						jointpositions(k) = joints.position[k];
					}
					// IK }
					if (now_cmd->rotationOrTranslation == 1) {

            KDL::ChainFkSolverPos_recursive fksolverRCM = KDL::ChainFkSolverPos_recursive(kc.LWR_HSC_nozzle_RCM_used()); // define the forward kinematic solver via the defined chain
            KDL::ChainIkSolverVel_pinv iksolvervRCM = KDL::ChainIkSolverVel_pinv(
               kc.LWR_HSC_nozzle_RCM_used());  // Inverse velocity solver
            KDL::ChainIkSolverPos_NR iksolverRCM(kc.LWR_HSC_nozzle_RCM_used(),
               fksolverRCM, iksolvervRCM, 100, 1e-6); // Maximum 100 iterations, stop at accuracy 1e-6
            std::cout << "Yo! you save three of us" << std::endl;

            /***********************************************************************************************************
            // https://stackoverflow.com/questions/13771318/what-is-the-lifetime-and-scope-of-the-enclosed-loop-variable
            // The life of the object is inside of those curly braces. The default constructor gets called on line 3 of
            // your code. The destructor would be called when you get to the }.
             * *********************************************************************************************************/


						// i. rotate
					  ROS_INFO("If you don't see me, I died through IK_RCM");
						int ik_error = iksolverRCM.CartToJnt(jointpositions,
								cartpos, jointpositions_new);
						ROS_INFO("If you see me, I survived through IK_RCM");
						// ii. get the nozzle position after rotate on rcm---> we going to use it to compute "m"
						kinematics_status = fksolver.JntToCart(
								jointpositions_new, cartposTool);
            ROS_INFO("If you see me, I survived through FK_RCM");
						// iii. compute m
						kc.cartpos_tool = cartposTool;
						kc.m = sqrt(
								pow(cartposTool.p[0] - rcm_point.x, 2)
										+ pow(cartposTool.p[1] - rcm_point.y, 2)
										+ pow(cartposTool.p[2] - rcm_point.z,
												2));

					} else { // Translation IK

						int ik_error = iksolver.CartToJnt(jointpositions,
								cartpos, jointpositions_new);
						// compute T
						kc.T = sqrt(
								pow(cartpos.p[0] - rcm_point.x, 2)
										+ pow(cartpos.p[1] - rcm_point.y, 2)
										+ pow(cartpos.p[2] - rcm_point.z, 2));
						std::cout << "rcm_point.z: " << rcm_point.z << std::endl;
						std::cout << "test1.z: " << test1.z << std::endl;
						std::cout << "kc.T: " << kc.T << std::endl;
						// compute DeltaP
						kc.deltaP = kc.T - kc.m;
						std::cout << "kc.m(before): " << kc.m << std::endl;
						std::cout << "kc.deltaP: " << kc.deltaP << std::endl;
						// update
						kc.m = kc.m + kc.deltaP;
            std::cout << "kc.m(after): " << kc.m << std::endl;
						kc.setRCMtoollength(kc.deltaP); // update the distance from end-effector to rcm point

					}

					ROS_INFO("jointpositions_new(0)= %f",
							jointpositions_new(0));
					ROS_INFO("jointpositions_new(1)= %f",
							jointpositions_new(1));
					ROS_INFO("jointpositions_new(2)= %f",
							jointpositions_new(2));
					ROS_INFO("jointpositions_new(3)= %f",
							jointpositions_new(3));
					ROS_INFO("jointpositions_new(4)= %f",
							jointpositions_new(4));
					ROS_INFO("jointpositions_new(5)= %f",
							jointpositions_new(5));
					ROS_INFO("jointpositions_new(6)= %f",
							jointpositions_new(6));
					kinematics_status = fksolver.JntToCart(jointpositions_new,
							cartpos);
					ROS_INFO("Get FK result");
					if (kinematics_status >= 0) {
						ROS_INFO("FK_x= %f", cartpos.p[0]);
						ROS_INFO("FK_y= %f", cartpos.p[1]);
						ROS_INFO("FK_z= %f", cartpos.p[2]);
						cartpos.M.GetRPY(roll, pitch, yaw);
						ROS_INFO("FK_Rx= %f", roll);
						ROS_INFO("FK_Ry= %f", pitch);
						ROS_INFO("FK_Rz= %f\n", yaw);
						ROS_INFO("This iteration end here---------------------------------------------------");
					}

					kc.evalPoints(pt, jointpositions_new, nj);
					pt.time_from_start = ros::Duration(dt);
					joint_cmd.points[0] = pt;
					joint_ref = jointpositions_new;
					++now_cmd;
				}
			} else {
				//std::reverse(test_ref.begin(), test_ref.end());
				//now_cmd = test_ref.begin();
				iterationTime++;
			}

		} else { //if (iterationTime == iterationMax)  then move to the return position
			if (return_cmd != return_ref.end()) {
				d0 = joint_ref(0) - joints.position[0];
				d01 = fabs(d0);
				d1 = joint_ref(1) - joints.position[1];
				d02 = fabs(d1);
				d2 = joint_ref(2) - joints.position[2];
				d03 = fabs(d2);
				d3 = joint_ref(3) - joints.position[3];
				d04 = fabs(d3);
				d4 = joint_ref(4) - joints.position[4];
				d05 = fabs(d4);
				d5 = joint_ref(5) - joints.position[5];
				d06 = fabs(d5);
				d6 = joint_ref(6) - joints.position[6];
				d07 = fabs(d6);

				if (d01 < threshold && d02 < threshold && d03 < threshold
						&& d04 < threshold && d05 < threshold && d06 < threshold
						&& d07 < threshold) {
					arrived = true;
				} else {
					arrived = false;
				}
				if (arrived) {  //{ take out element
					cartpos.p[0] = return_cmd->x;
					cartpos.p[1] = return_cmd->y;
					cartpos.p[2] = return_cmd->z;
					ROS_INFO("FK_x= %f", cartpos.p[0]);
					ROS_INFO("FK_y= %f", cartpos.p[1]);
					ROS_INFO("FK_z= %f", cartpos.p[2]);
					rpy = KDL::Rotation::RPY(return_cmd->nx, return_cmd->ny,
							return_cmd->nz);
					ROS_INFO("FK_Rx= %f", return_cmd->nx);
					ROS_INFO("FK_Ry= %f", return_cmd->ny);
					ROS_INFO("FK_Rz= %f\n", return_cmd->nz);
					cartpos.M = rpy;
					cartpos.M.GetRPY(roll, pitch, yaw);
					for (int k = 0; k < nj; k++) {
						jointpositions(k) = joints.position[k];
					}
					// IK }
					int ik_error = iksolver.CartToJnt(jointpositions, cartpos,
							jointpositions_new);
					kc.evalPoints(pt, jointpositions_new, nj);
					pt.time_from_start = ros::Duration(0.5);
					joint_cmd.points[0] = pt;
					joint_ref = jointpositions_new;
					++return_cmd;
				}
			}
		}

		joint_cmd.header.stamp = ros::Time::now();
		cmd_pub.publish(joint_cmd);
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;

}

