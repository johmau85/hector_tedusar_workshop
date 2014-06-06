/*
 * gripper_exception.cpp
 *
 *  Created on: 27.11.2012
 *      Author: alex
 */

#include "weiss_wsg_gripper/gripper_exceptions.h"

using namespace weiss_wsg_gripper;

GripperException::GripperException(const std::string& message) :
		message_(message) {
}

GripperException::~GripperException() throw () {
}

const char* GripperException::what() const throw () {
	return message_.c_str();
}

GripperTransportException::GripperTransportException(const std::string& message) :
		GripperException(message) {
}

GripperTimeoutException::GripperTimeoutException(const std::string &message) : GripperException(message)
{
}


GripperInternalErrorException::GripperInternalErrorException(
		const std::string& message) :
		GripperException(message) {
}

GripperCommandFailureException::GripperCommandFailureException(
		const std::string& message, Gripper::Command command,
		Gripper::ErrorCode error_code) :
		GripperException(message), command_(command), error_code_(error_code) {
}

GripperCommandFailureException::GripperCommandFailureException(
		Gripper::Command command, Gripper::ErrorCode error_code) :
		GripperException(
				"command " + Gripper::getCommandName(command)
						+ " failed with error code "
						+ Gripper::getErrorCodeName(error_code)), command_(
				command), error_code_(error_code) {
}

GripperIllegalArgumentException::GripperIllegalArgumentException(
		const std::string& message) :
		GripperException(message) {
}

GripperIllegalStateException::GripperIllegalStateException(
		const std::string& message) :
		GripperException(message) {
}

