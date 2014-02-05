/*
 *  dumbo_cart_vel_controller_node.cpp
 *
 *
 *  Created on: Feb 5, 2014
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


#include <ros/ros.h>
#include <brics_actuator/JointVelocities.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <kdl_wrapper/kdl_wrapper.h>
#include <boost/thread.hpp>


class CartVelControllerNode
{
public:
	ros::NodeHandle n_;

	/// declaration of topics to publish
	ros::Publisher topicPub_CommandVel_;

	/// declaration of topics to subscribe, callback is called for new messages arriving
	ros::Subscriber topicSub_JointState_;
	ros::Subscriber topicSub_CommandTwist_;

	CartVelControllerNode()
	{
		n_ = ros::NodeHandle("~");
		m_kdl_wrapper_initialized = false;
		m_received_js = false;
		m_initialized = false;

		getROSParameters();

		m_joint_pos = std::vector<double>(m_DOF, 0.0);
		m_joint_vel = std::vector<double>(m_DOF, 0.0);

		m_joint_pos_msg.header.stamp = ros::Time::now();
		m_joint_vel_msg.velocities.resize(m_DOF);


		if(!m_kdl_wrapper_initialized)
		{

			if(m_dumbo_kdl_wrapper.init("arm_base_link", m_arm_select+std::string("_arm_7_link")) )
			{
				m_dumbo_kdl_wrapper.ik_solver_vel->setLambda(0.3);
				m_kdl_wrapper_initialized = true;
			}

			else
			{
				ROS_ERROR("Error initializing Dumbo KDL wrapper");
				return;
			}
		}

		if(isInitialized())
		{
			topicPub_CommandVel_ = n_.advertise<brics_actuator::JointVelocities>("/" + m_arm_select + "_arm_controller/command_vel", 1);
			topicSub_JointState_ = n_.subscribe("/" + m_arm_select + "_arm_controller/state", 1, &CartVelControllerNode::topicCallback_joint_states, this);
			topicSub_CommandTwist_ = n_.subscribe("command_twist", 1, &CartVelControllerNode::topicCallback_twist, this);
		}

	}

	~CartVelControllerNode()
	{
	}


	bool isInitialized()
	{
		return m_initialized;
	}

	void getROSParameters()
	{
		/// Get joint names
		XmlRpc::XmlRpcValue JointNamesXmlRpc;
		if (n_.hasParam("joint_names"))
		{
			n_.getParam("joint_names", JointNamesXmlRpc);
		}

		else
		{
			ROS_ERROR("Parameter joint_names not set, shutting down node...");
			n_.shutdown();
			return;
		}


		/// Resize and assign of values to the JointNames
		m_joint_names.resize(JointNamesXmlRpc.size());
		for (int i = 0; i < JointNamesXmlRpc.size(); i++)
		{
			m_joint_names[i] = (std::string)JointNamesXmlRpc[i];
		}

		m_DOF = m_joint_names.size();

		/// Get arm selection parameter (left or right arm)
		if(m_joint_names[0].substr(0, 4) == "left")
		{
			m_arm_select = "left";
		}

		else if(m_joint_names[0].substr(0,5) == "right")
		{
			m_arm_select = "right";
		}

		else
		{
			ROS_ERROR("Error choosing left/right arm from joint_names parameter");
			n_.shutdown();
			return;
		}

		// get the velocity limits

		if (n_.hasParam("v_limit"))
		{
			n_.getParam("v_limit", m_v_limit);
		}

		else
		{
			ROS_ERROR("Parameter v_limit not set, shutting down node...");
			n_.shutdown();
			return;
		}

		if (n_.hasParam("w_limit"))
		{
			n_.getParam("w_limit", m_w_limit);
		}

		else
		{
			ROS_ERROR("Parameter w_limit not set, shutting down node...");
			n_.shutdown();
			return;
		}


		if( m_v_limit<=0.0 || m_w_limit<=0.0)
		{
			ROS_ERROR("Invalid gains/limits (<= 0.0), shutting down node...");
			n_.shutdown();
			return;
		}

		m_initialized = true;

	}

	bool setArmSelect(std::string ArmSelect)
	{
		if(ArmSelect!="left" && ArmSelect!="right")
		{
			ROS_ERROR("Invalid arm_select parameter, shutting down node...");
			n_.shutdown();
			return false;
		}
		m_arm_select = ArmSelect;
		return true;
	}

	void topicCallback_twist(const geometry_msgs::TwistStampedPtr &msg)
	{
		m_twist = *msg;
		if(calculateJointVel(*msg))
			publishJointVel();
	}

	void topicCallback_joint_states(const control_msgs::JointTrajectoryControllerStatePtr &msg)
	{
		ROS_DEBUG("Received joint states");

		m_joint_pos_msg = *msg;

		// search for joints in joint state msg
		if(msg->actual.positions.size()==m_DOF)
		{
			for(unsigned int i=0; i<m_DOF; i++)
			{
				if(msg->joint_names[i]!=m_joint_names[i])
				{
					ROS_ERROR("Error in received joint name");
					return;
				}

				else
				{
					m_mutex.lock();
					m_joint_pos[i] = msg->actual.positions[i];
					m_mutex.unlock();
				}
			}
		}
		m_received_js = true;
	}


	// calculates joint velocities taking as input twist of *_arm_7_link with
	bool calculateJointVel(const geometry_msgs::TwistStamped &twist)
	{
		if(twist.header.frame_id != "/arm_base_link" && twist.header.frame_id != "arm_base_link")
		{
			static ros::Time t = ros::Time::now();
			if((ros::Time::now() - t).toSec()>2.0)
			{
				ROS_ERROR("Input twist not expressed in arm_base_link frame");
				t = ros::Time::now();
			}
			return false;
		}

		if(!m_kdl_wrapper_initialized)
		{
			static ros::Time t = ros::Time::now();
			if((ros::Time::now() - t).toSec()>2.0)
			{
				ROS_ERROR("Dumbo KDL object not initialized");
				t = ros::Time::now();
			}
			return false;
		}

		if(!m_received_js)
		{
			static ros::Time t = ros::Time::now();
			if((ros::Time::now() - t).toSec()>2.0)
			{
				ROS_ERROR("Haven't received joint states");
				t = ros::Time::now();
			}
			return false;
		}

		KDL::Twist v_in;
		v_in(0) = twist.twist.linear.x;
		v_in(1) = twist.twist.linear.y;
		v_in(2) = twist.twist.linear.z;
		v_in(3) = twist.twist.angular.x;
		v_in(4) = twist.twist.angular.y;
		v_in(5) = twist.twist.angular.z;


		//saturate velocity screw
		double v_scale = (sqrt(pow(v_in(0), 2.0) + pow(v_in(1), 2.0) + pow(v_in(2), 2.0))/m_v_limit);

		if(v_scale>1.0)
		{
			for(int i=0; i<3; i++)
			{
				v_in(i) /= v_scale;
			}
		}

		double w_scale = sqrt(pow(v_in(3), 2.0) + pow(v_in(4), 2.0) + pow(v_in(5), 2.0))/(m_w_limit);

		if(w_scale>1.0)
		{
			for(int i=0; i<3; i++)
			{
				v_in(i+3) /= w_scale;
			}
		}

		KDL::JntArray q_in(7);
		KDL::JntArray q_dot_out;

		m_mutex.lock();
		for(unsigned int i=0; i<7; i++) q_in(i) = m_joint_pos[i];
		m_mutex.unlock();

		bool ret = m_dumbo_kdl_wrapper.ik_solver_vel->CartToJnt(q_in, v_in, q_dot_out);
		for(unsigned int i=0; i<7; i++) m_joint_vel[i] = q_dot_out(i);

		return true;
	}


	void publishJointVel()
	{
		ROS_DEBUG("Publish vel");
		if(m_received_js)
		{
			ros::Time now = ros::Time::now();
			if((now - m_joint_pos_msg.header.stamp).toSec()<0.2)
			{
				for(unsigned int i=0; i<m_DOF; i++)
				{
					m_joint_vel_msg.velocities[i].unit = "rad";
					m_joint_vel_msg.velocities[i].joint_uri = m_joint_names[i].c_str();
					m_joint_vel_msg.velocities[i].value = m_joint_vel[i];
				}
				topicPub_CommandVel_.publish(m_joint_vel_msg);
				ROS_DEBUG("Velocity command");
			}
			else
			{
				static ros::Time t = ros::Time::now();
				if((ros::Time::now() - t).toSec()>2.0)
				{
					ROS_ERROR("Joint pos too old");
					t = ros::Time::now();
				}
				return;
			}

		}
		else
		{
			static ros::Time t = ros::Time::now();
			if((ros::Time::now() - t).toSec()>2.0)
			{
				ROS_ERROR("Haven't received joint pos");
				t = ros::Time::now();
			}
			return;
		}

	}


private:
	bool m_kdl_wrapper_initialized;

	double m_v_limit;
	double m_w_limit;


	unsigned int m_DOF;
	bool m_initialized;

	std::string m_arm_select;

	std::vector<std::string> m_joint_names;

	geometry_msgs::TwistStamped m_twist;
	control_msgs::JointTrajectoryControllerState m_joint_pos_msg;
	brics_actuator::JointVelocities m_joint_vel_msg;

	std::vector<double> m_joint_pos;
	std::vector<double> m_joint_vel;

	boost::mutex m_mutex;

	KDLWrapper m_dumbo_kdl_wrapper;

	bool m_received_js;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cart_vel_controller");

	CartVelControllerNode controller;


	if(!controller.isInitialized())
	{
		ROS_ERROR("Controller not initialized, shutting down node.");
		return 0;
	}

	ros::spin();

	return 0;
}
