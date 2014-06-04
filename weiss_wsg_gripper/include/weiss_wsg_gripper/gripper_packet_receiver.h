/*
 * gripper_packet_receiver.h
 *
 *  Created on: 30.11.2012
 *      Author: alex
 */

#ifndef WEISS_WSG_GRIPPER_GRIPPER_PACKET_RECEIVER_H_
#define WEISS_WSG_GRIPPER_GRIPPER_PACKET_RECEIVER_H_

#include <vector>
#include "weiss_wsg_gripper/gripper.h"
#include "weiss_wsg_gripper/gripper_packet_checksum.h"

namespace weiss_wsg_gripper {

class GripperPacketReceiver {
public:
	GripperPacketReceiver();

	bool feed(unsigned char byte);
	Gripper::Command getCommand();
	const std::vector<unsigned char> &getPayload();

private:
	enum RxState {
		RX_STATE_PREAMBLE,
		RX_STATE_COMMAND,
		RX_STATE_SIZE,
		RX_STATE_PAYLOAD,
		RX_STATE_CHECKSUM,
	};

	// sensible maximum value for the payload size.
	// TODO: find out what's a realistic maximum size.
	static const size_t MAX_RX_PAYLOAD_SIZE = 128;

	void reset();

	RxState rx_state_;
	size_t rx_count_;
	Gripper::Command rx_command_;
	size_t rx_payload_size_;
	std::vector<unsigned char> rx_payload_;
	GripperPacketChecksum checksum_;
};

}

#endif /* WEISS_WSG_GRIPPER_GRIPPER_PACKET_RECEIVER_H_ */
