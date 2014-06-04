/*
 * gripper_packet_checksum.h
 *
 *  Created on: 30.11.2012
 *      Author: alex
 */

#ifndef WEISS_WSG_GRIPPER_GRIPPER_PACKET_CHECKSUM_H_
#define WEISS_WSG_GRIPPER_GRIPPER_PACKET_CHECKSUM_H_

#include <vector>

namespace weiss_wsg_gripper {

class GripperPacketChecksum {
public:
	GripperPacketChecksum();

	void reset();
	void feed(unsigned char data);
	void feed(const std::vector<unsigned char> &data);
	unsigned short getValue();

private:
	unsigned short crc_;
};

}

#endif /* WEISS_WSG_GRIPPER_GRIPPER_PACKET_CHECKSUM_H_ */
