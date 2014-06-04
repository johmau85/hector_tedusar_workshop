/*
 * gripper_packet_receiver.cpp
 *
 *  Created on: 30.11.2012
 *      Author: alex
 */

#include <ros/ros.h>
#include "weiss_wsg_gripper/gripper_packet_receiver.h"

using namespace weiss_wsg_gripper;

GripperPacketReceiver::GripperPacketReceiver() {
	reset();
}

bool GripperPacketReceiver::feed(unsigned char rx_byte) {
	bool received_valid_packet = false;
	checksum_.feed(rx_byte);
	//ROS_INFO(
	//		"GripperPacketReceiver::feed called with rx_byte = 0x%02X", rx_byte);

	switch (rx_state_) {
	case RX_STATE_PREAMBLE:
		if (rx_byte == 0xAA) {
			if (++rx_count_ >= 3) {
				rx_state_ = RX_STATE_COMMAND;
				rx_count_ = 0;
			}
		} else {
			reset();
		}
		break;

	case RX_STATE_COMMAND:
		if (Gripper::getCommandFromId(rx_byte, rx_command_)) {
			rx_state_ = RX_STATE_SIZE;
			rx_count_ = 0;
			rx_payload_size_ = 0;
		} else {
			reset();
		}
		break;

	case RX_STATE_SIZE:
		rx_payload_size_ |= static_cast<size_t>(rx_byte) << (8 * rx_count_);
		if (++rx_count_ >= 2) {
			if (rx_payload_size_ <= MAX_RX_PAYLOAD_SIZE) {
				if (rx_payload_size_ != 0)
					rx_state_ = RX_STATE_PAYLOAD;
				else
					rx_state_ = RX_STATE_CHECKSUM;
				rx_count_ = 0;
				rx_payload_.clear();
			} else {
				rx_state_ = RX_STATE_PREAMBLE;
				rx_count_ = 0;
			}
		}
		break;

	case RX_STATE_PAYLOAD:
		rx_payload_.push_back(rx_byte);
		if (++rx_count_ >= rx_payload_size_) {
			rx_state_ = RX_STATE_CHECKSUM;
			rx_count_ = 0;
		}
		break;

	case RX_STATE_CHECKSUM:
		if (++rx_count_ >= 2) {
			if (checksum_.getValue() == 0) {
				// received valid packet! :-)
				received_valid_packet = true;
			}
			reset();
		}
		break;
	}

	//ROS_INFO(
	//		"GripperPacketReceiver state: state = %d, count = %d, payload_size = %d, checksum = %d", rx_state_, rx_count_, rx_payload_size_, checksum_.getValue());

	return received_valid_packet;
}

Gripper::Command GripperPacketReceiver::getCommand() {
	return rx_command_;
}

const std::vector<unsigned char> &GripperPacketReceiver::getPayload() {
	return rx_payload_;
}

void GripperPacketReceiver::reset() {
	rx_state_ = RX_STATE_PREAMBLE;
	rx_count_ = 0;
	rx_payload_size_ = 0;
	checksum_.reset();
}

