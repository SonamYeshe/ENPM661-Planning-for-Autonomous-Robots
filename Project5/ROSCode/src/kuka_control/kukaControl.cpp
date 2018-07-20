// "Copyright [2017] <Michael Kam>"
/** @file kukaControl.cpp
 *  @brief This kukaControl.cpp is an implement file of controlling the iiwa kuka arm
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 *
 *  kukaControl is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  kukaControl is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with kukaControl.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include <kukaControl.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/Twist.h>
#include <iiwa_msgs/JointPosition.h>
#include <sensor_msgs/JointState.h>
#include <kdl/chain.hpp>
// #include "Eigen/Core"
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <string>
#include <sstream>

kukaControl::kukaControl() {
}

// initialize the joint positions with a non-zero value to be used in the solvers
void kukaControl::initializeJoints(KDL::JntArray & jointPositions) {

	jointPositions(0) = -1.57;
	jointPositions(1) = 0;
	jointPositions(2) = 0;
	jointPositions(3) = -1.57;
	jointPositions(4) = 0;
	jointPositions(5) = 1.57;
	jointPositions(6) = 0;

	/*
	 jointPositions(0) = -2.019529;
	 jointPositions(1) = -0.31057;
	 jointPositions(2) = 0.38259;
	 jointPositions(3) = -1.604;
	 jointPositions(4) = 0.1818;
	 jointPositions(5) = 1.10304;
	 jointPositions(6) = 0.778779;
	 */
}

// initialize a joint command point
void kukaControl::initializePoints(trajectory_msgs::JointTrajectoryPoint & pt,
		int nj, float init) {
	for (int i = 0; i < nj; ++i)
		pt.positions.push_back(init);
}

// defines the joint names for the robot (used in the jointTrajectory messages)
void kukaControl::nameJoints(trajectory_msgs::JointTrajectory & cmd, int nj) {
	for (int i = 1; i <= nj; ++i) {
		std::ostringstream joint_name;
		joint_name << "iiwa_joint_";
		joint_name << i;
		cmd.joint_names.push_back(joint_name.str());
	}
}

// loads the joint space points to be sent as a command to the robot
void kukaControl::evalPoints(trajectory_msgs::JointTrajectoryPoint & point,
		KDL::JntArray & jointPositions, int nj) {
	for (int i = 0; i < nj; ++i) {
		while (jointPositions(i) > M_PI)
			jointPositions(i) -= 2 * M_PI;
		while (jointPositions(i) < -M_PI)
			jointPositions(i) += 2 * M_PI;
		point.positions[i] = jointPositions(i);
	}
}

KDL::Chain kukaControl::LWR_knife() {
	KDL::Chain chain;

	// base
	chain.addSegment(
	// KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame::DH_Craig1989(0, 0, 0.33989, 0)));
			KDL::Segment(KDL::Joint(KDL::Joint::None),
					KDL::Frame::DH_Craig1989(0, 0, 0.34, 0)));
	// joint 1
	chain.addSegment(
			KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
					KDL::Frame::DH_Craig1989(0, -M_PI_2, 0, 0)));
	// joint 2
	chain.addSegment(
	// KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH_Craig1989(0, M_PI_2, 0.40011, 0)));
			KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
					KDL::Frame::DH_Craig1989(0, M_PI_2, 0.40, 0)));
	// joint 3
	chain.addSegment(
			KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
					KDL::Frame::DH_Craig1989(0, M_PI_2, 0, 0)));
	// joint 4
	chain.addSegment(
			KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
					KDL::Frame::DH_Craig1989(0, -M_PI_2, 0.4000, 0)));
	// joint 5
	chain.addSegment(
			KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
					KDL::Frame::DH_Craig1989(0, -M_PI_2, 0, 0)));
	// joint 6
	chain.addSegment(
			KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
					KDL::Frame::DH_Craig1989(0, M_PI_2, 0, 0)));

	// joint 7 (with flange adapter)
	/*  chain.addSegment(
	 KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
	 KDL::Frame::DH_Craig1989(0, 0, 0.12597, 0)));*/
	// for HSC
	chain.addSegment(
			KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
					KDL::Frame::DH_Craig1989(0, 0, 0.12597 + 0.41275, 0)));

	KDL::Frame ee = KDL::Frame(KDL::Vector(0.0625, -0.0925, -0.07))
			* KDL::Frame(
					KDL::Rotation::EulerZYX(32.0 * M_PI / 180, 0.0,
							54.18 * M_PI / 180));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), ee));

	return chain;

}

KDL::Chain kukaControl::LWR_HSC_nozzle() {
	KDL::Chain chain;
	// base
	chain.addSegment(
	// KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame::DH_Craig1989(0, 0, 0.33989, 0)));
			KDL::Segment(KDL::Joint(KDL::Joint::None),
					KDL::Frame::DH_Craig1989(0, 0, 0.34, 0)));
			      //KDL::Frame::DH_Craig1989(0, 0, 0.36, 0)));
	// joint 1
	chain.addSegment(
			KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
					KDL::Frame::DH_Craig1989(0, -M_PI_2, 0, 0)));
	// joint 2
	chain.addSegment(
	// KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH_Craig1989(0, M_PI_2, 0.40011, 0)));
			KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
					KDL::Frame::DH_Craig1989(0, M_PI_2, 0.40, 0)));
	          //KDL::Frame::DH_Craig1989(0, M_PI_2, 0.42, 0)));
	// joint 3
	chain.addSegment(
			KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
					KDL::Frame::DH_Craig1989(0, M_PI_2, 0, 0)));
	// joint 4
	chain.addSegment(
			KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
					KDL::Frame::DH_Craig1989(0, -M_PI_2, 0.4000, 0)));
	// joint 5
	chain.addSegment(
			KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
					KDL::Frame::DH_Craig1989(0, -M_PI_2, 0, 0)));
	// joint 6
	chain.addSegment(
			KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
					KDL::Frame::DH_Craig1989(0, M_PI_2, 0, 0)));

	// joint 7 (with flange adapter)
	/*  chain.addSegment(
	 KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
	 KDL::Frame::DH_Craig1989(0, 0, 0.12597, 0)));*/
	// for HSC
	chain.addSegment(
			KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
					KDL::Frame::DH_Craig1989(0, 0, 0.12597 + 0.4, 0)));

	return chain;

}

KDL::Chain kukaControl::LWR_HSC_nozzle_RCM_used() {
	KDL::Chain chain;
	// base
	chain.addSegment(
	// KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame::DH_Craig1989(0, 0, 0.33989, 0)));
			KDL::Segment(KDL::Joint(KDL::Joint::None),
					KDL::Frame::DH_Craig1989(0, 0, 0.34, 0)));
          //KDL::Frame::DH_Craig1989(0, M_PI_2, 0.36, 0)));
	// joint 1
	chain.addSegment(
			KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
					KDL::Frame::DH_Craig1989(0, -M_PI_2, 0, 0)));
	// joint 2
	chain.addSegment(
	// KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH_Craig1989(0, M_PI_2, 0.40011, 0)));
			KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
					KDL::Frame::DH_Craig1989(0, M_PI_2, 0.40, 0)));
	          //KDL::Frame::DH_Craig1989(0, M_PI_2, 0.42, 0)));
	// joint 3
	chain.addSegment(
			KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
					KDL::Frame::DH_Craig1989(0, M_PI_2, 0, 0)));
	// joint 4
	chain.addSegment(
			KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
					KDL::Frame::DH_Craig1989(0, -M_PI_2, 0.4000, 0)));
	// joint 5
	chain.addSegment(
			KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
					KDL::Frame::DH_Craig1989(0, -M_PI_2, 0, 0)));
	// joint 6
	chain.addSegment(
			KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
					KDL::Frame::DH_Craig1989(0, M_PI_2, 0, 0)));

	// joint 7 (with flange adapter)
	/*  chain.addSegment(
	 KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
	 KDL::Frame::DH_Craig1989(0, 0, 0.12597, 0)));*/
	// for HSC
	chain.addSegment(
			KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
					KDL::Frame::DH_Craig1989(0, 0,
							0.12597 + Dist_from_EndeffectorToRcmPoint, 0)));

	return chain;

}

void kukaControl::setRCMtoollength(double Dist_fromEndeffectorToRcmPoint) {
	std::cout << "(output before)Dist_from_EndeffectorToRcmPoint: " << Dist_from_EndeffectorToRcmPoint << std::endl;
	Dist_from_EndeffectorToRcmPoint = Dist_from_EndeffectorToRcmPoint
			- Dist_fromEndeffectorToRcmPoint;
	std::cout << "(input)Dist_fromEndeffectorToRcmPoint: " << Dist_fromEndeffectorToRcmPoint << std::endl;

	std::cout << "(output after)Dist_from_EndeffectorToRcmPoint: " << Dist_from_EndeffectorToRcmPoint << std::endl;
}





