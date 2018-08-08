#include "ros/ros.h"
#include <stdexcept>
#include <ar2cpp/ar2.h>
#include <ar2cpp/joint.h>

namespace ar2cpp
{
	AR2::AR2()
	{
		joints[0].name = "joint_1";
		joints[0].setMotorId(1);
		joints[1].name = "joint_2";
		joints[1].setMotorId(2);
		joints[2].name = "joint_3";
		joints[2].setMotorId(3);
		joints[3].name = "joint_4";
		joints[3].setMotorId(4);
		joints[4].name = "joint_5";
		joints[4].setMotorId(5);
		joints[5].name = "joint_6";
		joints[5].setMotorId(6);

	}

	AR2::~AR2()
	{

	}

	Joint AR2::getJoint(std::string jointName)
	{
		int numJoints = sizeof(joints);
		for (int i = 0; i < numJoints; i++)
		{
			if (joints[i].name == jointName)
			{
				return joints[i];
			}
		}

		throw std::runtime_error("Could not find joint with name " + jointName);
	}

	void AR2::setJoint(Joint joint)
	{
		bool foundJoint = false;

		int numJoints = sizeof(joints);
		for (int i = 0; i < numJoints; i++)
		{
			if (joints[i].name == joint.name)
			{
				foundJoint = true;
				joints[i] = joint;
			}
		}

		if (foundJoint == false)
		{
			throw std::runtime_error("Could not find joint with name " + joint.name);
		}
	}
}