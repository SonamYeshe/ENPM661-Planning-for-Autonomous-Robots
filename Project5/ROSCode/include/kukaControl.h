// "Copyright [2017] <Michael Kam>"
/** @file kukaControl.h
 *  @brief This kukaControl.h is a header file of controlling the iiwa kuka arm
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

#ifndef INCLUDE_KUKACONTROL_H_
#define INCLUDE_KUKACONTROL_H_
/** @brief kukaControl is an implementation of controlling the iiwa kuka arm
 * *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 */
class kukaControl {
private:
	// this only apply to LWR_HSC_nozzle_RCM_used();
	double Dist_from_EndeffectorToRcmPoint = 0.2;
	// KDL::ChainFkSolverPos_recursive fksolver;
public:
	/**constructor */
	kukaControl();
	/**@brief initializeJoints sets the initial value for the IK solver
	 * @param[in] jointPositions vector that store joint position value
	 * @return none     */
	void initializeJoints(KDL::JntArray & jointPositions);
	/**@brief initialize_points() set the _init into _pt
	 * @param[in] pt trajectory_msgs::JointTrajectoryPoint
	 * @param[in] nj number of joint of the robot
	 * @param[in] init initial value want to set
	 * @return none     */
	void initializePoints(trajectory_msgs::JointTrajectoryPoint & pt, int nj,
			float init);
	/**@brief nameJoints() name the joint
	 * @param[in] cmd stores the command message to control iiwa
	 * @param[in] nj number of joint of the robot
	 * @return none     */
	void nameJoints(trajectory_msgs::JointTrajectory & cmd, int nj);
	/**@brief evalPoints set the jointPositions value into point
	 * @param[in] point stores the command message to control iiwa
	 * @param[in] jointPositions vector that store joint position value
	 * @param[in] nj number of joint of the robot
	 * @return none     */
	void evalPoints(trajectory_msgs::JointTrajectoryPoint & point,
			KDL::JntArray & jointPositions, int nj);

	double L_tool_length;
	double m;
	double T;
	double deltaP;
	KDL::Frame cartpos_tool;

	void setRCMtoollength(double Dist_fromEndeffectorToRcmPoint);

	/**@brief the function defined the kinematic chain of the iiwa robot
	 * @param[in] none
	 * @return KDL::Chain     */
	KDL::Chain LWR_knife();
	KDL::Chain LWR_HSC_nozzle();
	KDL::Chain LWR_HSC_nozzle_RCM_used();

};

#endif  // INCLUDE_KUKACONTROL_H_
