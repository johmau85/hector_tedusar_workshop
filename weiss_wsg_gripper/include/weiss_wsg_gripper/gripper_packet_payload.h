/*
 * gripper_packet_payload.h
 *
 *  Created on: 29.11.2012
 *      Author: alex
 */

#ifndef WEISS_WSG_GRIPPER_GRIPPER_PACKET_PAYLOAD_H_
#define WEISS_WSG_GRIPPER_GRIPPER_PACKET_PAYLOAD_H_

#include <cstddef>
#include <string>
#include <vector>

namespace weiss_wsg_gripper {

	class GripperPacketPayload {
	public:
		static const GripperPacketPayload EMPTY_PAYLOAD;

		GripperPacketPayload();
		GripperPacketPayload(const std::vector<unsigned char> &contents);
		~GripperPacketPayload();

		const std::vector<unsigned char> &getContents() const;
		void setContents(const std::vector<unsigned char> contents);
		size_t getSize() const;
		size_t getReadableSize();

		void writeData(const std::vector<unsigned char> &data);
		void writeUint8(unsigned char value);
		void writeUint16(unsigned short value);
		void writeUint32(unsigned int value);
		void writeFloat(float value);
		void writeString(const std::string &string);

		void readData(std::vector<unsigned char> &buffer, size_t count);
		unsigned char readUint8();
		unsigned short readUint16();
		unsigned int readUint32();
		float readFloat();
		std::string readString(size_t length);

	private:
		GripperPacketPayload(const GripperPacketPayload &);
		GripperPacketPayload &operator=(const GripperPacketPayload &);

		std::vector<unsigned char> contents_;
		size_t read_index_;
	};

}

#endif /* WEISS_WSG_GRIPPER_GRIPPER_PACKET_PAYLOAD_H_ */
