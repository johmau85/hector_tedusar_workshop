/*
 * gripper_packet_payload.cpp
 *
 *  Created on: 29.11.2012
 *      Author: alex
 */

#include "weiss_wsg_gripper/gripper_packet_payload.h"

using namespace weiss_wsg_gripper;

const GripperPacketPayload GripperPacketPayload::EMPTY_PAYLOAD;

GripperPacketPayload::GripperPacketPayload() :
		contents_(), read_index_(0) {
}

GripperPacketPayload::GripperPacketPayload(
		const std::vector<unsigned char> &contents) :
		contents_(contents), read_index_(0) {
}

GripperPacketPayload::~GripperPacketPayload() {
}

const std::vector<unsigned char> &GripperPacketPayload::getContents() const {
	return contents_;
}

void GripperPacketPayload::setContents(
		const std::vector<unsigned char> contents) {
	contents_ = contents;
	read_index_ = 0;
}

size_t GripperPacketPayload::getSize() const {
	return contents_.size();
}

size_t GripperPacketPayload::getReadableSize() {
	return contents_.size() - read_index_;
}

void GripperPacketPayload::writeData(const std::vector<unsigned char> &data) {
	contents_.insert(contents_.end(), data.begin(), data.end());
}

void GripperPacketPayload::writeUint8(unsigned char value) {
	contents_.push_back(value);
}

void GripperPacketPayload::writeUint16(unsigned short value) {
	contents_.push_back(value);
	contents_.push_back(value >> 8);
}

void GripperPacketPayload::writeUint32(unsigned int value) {
	contents_.push_back(value);
	contents_.push_back(value >> 8);
	contents_.push_back(value >> 16);
	contents_.push_back(value >> 24);
}

void GripperPacketPayload::writeFloat(float value) {
	writeUint32(*reinterpret_cast<unsigned int *>(&value));
}

void GripperPacketPayload::writeString(const std::string &string) {
	for (std::string::const_iterator it = string.begin(); it != string.end();
			++it) {
		contents_.push_back(*it);
	}
}

void GripperPacketPayload::readData(std::vector<unsigned char> &buffer,
		size_t count) {
	for (; count != 0; --count) {
		buffer.push_back(contents_.at(read_index_++));
	}
}

unsigned char GripperPacketPayload::readUint8() {
	return contents_.at(read_index_++);
}

unsigned short GripperPacketPayload::readUint16() {
	unsigned short result;
	result = contents_.at(read_index_++);
	result |= static_cast<unsigned short>(contents_.at(read_index_++)) << 8;
	return result;
}

unsigned int GripperPacketPayload::readUint32() {
	unsigned int result;
	result = contents_.at(read_index_++);
	result |= static_cast<unsigned int>(contents_.at(read_index_++)) << 8;
	result |= static_cast<unsigned int>(contents_.at(read_index_++)) << 16;
	result |= static_cast<unsigned int>(contents_.at(read_index_++)) << 24;
	return result;
}

float GripperPacketPayload::readFloat() {
	unsigned int int_result = readUint32();
	return *reinterpret_cast<float *>(&int_result);
}

std::string GripperPacketPayload::readString(size_t length) {
	std::string result;
	for (; length != 0; --length) {
		result.push_back(contents_.at(read_index_++));
	}
	return result;
}

