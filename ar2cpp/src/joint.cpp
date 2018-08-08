#include <stdlib.h>
#include <math.h>
#include <stdexcept>
#include "ros/ros.h"
#include <ar2cpp/joint.h>

#define PI 3.14159265359
#define TAU 6.28318530718

namespace ar2cpp
{
	Joint::Joint()
	{
		
	}

	Joint::Joint(uint8_t motorId)
	{
		setMotorId(motorId);
	}

	Joint::~Joint()
	{

	}

	void Joint::setActuatorType(uint8_t actuatorType)
	{
		this->_actuatorType = actuatorType;
	}

	uint8_t Joint::getMotorId()
	{
		return this->_motorId;
	}

	void Joint::setMotorId(uint8_t motorId)
	{
		this->_motorId = motorId;
	}

	double Joint::_filterAngle(double angle)
	{
		_angleReads = _angleReads + 1;

		// put value at front of array
		for (int i = _filterPrevious - 1; i > 0; i--) {
			_previousAngles[i] = _previousAngles[i - 1];
		}
		_previousAngles[0] = angle;


		int filterIterations = _filterPrevious;
		if (_angleReads < _filterPrevious) {
			filterIterations = _angleReads;
		}

		double angleSum = 0;
		for (int i = 0; i < filterIterations; i++) {
			angleSum = angleSum + _previousAngles[i];
		}

		double filterResult = angleSum / (filterIterations * 1.0);

		//ROS_INFO("%f, %f, %f, %i", angle, angleSum, filterResult, filterIterations);

		return filterResult;
	}

	double Joint::readAngle()
	{
		if (_actuatorType == ACTUATOR_TYPE_MOTOR) {
			uint16_t position;
			return position;
		}
		else if (_actuatorType == ACTUATOR_TYPE_SERVO)
		{
			return _previousEffort;
		}
		else
		{
			return 0;
		}
	}

	void Joint::actuate(double effort, uint8_t duration = 15)
	{
		if (_actuatorType == ACTUATOR_TYPE_MOTOR)
		{
			if (effort > 1.0) effort = 1.0;
			if (effort < -1.0) effort = -1.0;
			if (abs(effort * 100.0) < 20) return; // because it's too little to do anything

			uint8_t data[4];
			data[3] = duration;
		}
		else if (_actuatorType == ACTUATOR_TYPE_SERVO)
		{
			if (effort != _previousEffort)
			{
				uint8_t data[4];
			}
		}

		_previousEffort = effort;
	}

	void Joint::setServoLimits(uint8_t minValue, uint8_t maxValue)
	{
		this->_minServoValue = minValue;
		this->_maxServoValue = maxValue;
	}

	double Joint::getPreviousEffort() {
		return this->_previousEffort;
	}
	
	int Joint::getActuatorType()
	{
		return _actuatorType;
	}
}