/*
 *  DumboCartVelController.h
 *
 *
 *  Created on: Feb 7, 2014
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2014, Francisco Viña, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef DUMBOCARTVELCONTROLLER_H_
#define DUMBOCARTVELCONTROLLER_H_

#include <control_msgs/JointTrajectoryControllerState.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <kdl_wrapper/kdl_wrapper.h>
#include <boost/thread.hpp>


class DumboCartVelController
{
public:
	ros::NodeHandle n_;

	/// declaration of topics to publish
	ros::Publisher topicPub_CommandVel_;

	/// declaration of topics to subscribe, callback is called for new messages arriving
	ros::Subscriber topicSub_JointState_;
	ros::Subscriber topicSub_CommandTwist_;

	DumboCartVelController();
	virtual ~DumboCartVelController();


	bool isInitialized();

	void getROSParameters();

	bool setArmSelect(std::string ArmSelect);

	void topicCallback_twist(const geometry_msgs::TwistStampedPtr &msg);

	virtual void topicCallback_joint_states(const control_msgs::JointTrajectoryControllerStatePtr &msg);

	// calculates joint velocities taking as input twist of *_arm_7_link
	// expressed with respect to the base frame
	// twist: input twist of *_arm_7_link
	// q_dot: output joint velocities command to be sent to manipulator.
	virtual bool calculateJointVelCommand(const geometry_msgs::TwistStamped &twist,
			KDL::JntArray &q_dot);


	void publishJointVelCommand(const KDL::JntArray &q_dot);


protected:
	bool m_kdl_wrapper_initialized;

	double m_v_limit;
	double m_w_limit;


	unsigned int m_DOF;
	bool m_initialized;

	std::string m_arm_select;

	std::vector<std::string> m_joint_names;

	control_msgs::JointTrajectoryControllerState m_joint_state_msg;

	boost::mutex m_mutex;

	KDLWrapper m_dumbo_kdl_wrapper;

	bool m_received_js;
};

#endif /* DUMBOCARTVELCONTROLLER_H_ */
