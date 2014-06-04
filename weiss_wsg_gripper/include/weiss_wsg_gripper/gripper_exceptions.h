/*
 * gripper_exception.h
 *
 *  Created on: 27.11.2012
 *      Author: alex
 */

#ifndef WEISS_WSG_GRIPPER_GRIPPER_EXCEPTION_H_
#define WEISS_WSG_GRIPPER_GRIPPER_EXCEPTION_H_

#include <exception>
#include <string>
#include "weiss_wsg_gripper/gripper.h"

namespace weiss_wsg_gripper {

class GripperException: public std::exception {
public:
	GripperException(const std::string &message);
	virtual ~GripperException() throw ();
	virtual const char* what() const throw ();
private:
	GripperException();
	const std::string message_;
};

class GripperTransportException: public GripperException {
public:
	GripperTransportException(const std::string &message);
};

class GripperInternalErrorException: public GripperException {
public:
	GripperInternalErrorException(const std::string &message);
};

class GripperCommandFailureException: public GripperException {
public:
	GripperCommandFailureException(const std::string &message,
			Gripper::Command command, Gripper::ErrorCode error_code);

	GripperCommandFailureException(Gripper::Command command,
			Gripper::ErrorCode error_code);

	inline Gripper::Command getCommand() const {
		return command_;
	}

	inline Gripper::ErrorCode getErrorCode() const {
		return error_code_;
	}

private:
	const Gripper::Command command_;
	const Gripper::ErrorCode error_code_;
};

class GripperIllegalArgumentException: public GripperException {
public:
	GripperIllegalArgumentException(const std::string &message);
};

class GripperIllegalStateException: public GripperException {
public:
	GripperIllegalStateException(const std::string &message);
};

}

#endif /* WEISS_WSG_GRIPPER_GRIPPER_EXCEPTION_H_ */
