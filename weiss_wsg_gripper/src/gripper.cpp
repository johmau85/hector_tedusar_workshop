/*
 * gripper.cpp
 *
 *  Created on: 28.11.2012
 *      Author: alex
 */

#include <string>
#include <boost/assert.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <ros/ros.h>
#include <weiss_wsg_gripper/gripper.h>
#include <weiss_wsg_gripper/gripper_exceptions.h>
#include <weiss_wsg_gripper/gripper_packet_payload.h>
#include <weiss_wsg_gripper/gripper_packet_receiver.h>
#include <weiss_wsg_gripper/transport.h>

#define ARRAY_LENGTH(array) (sizeof(array)/sizeof(array[0]))

using namespace weiss_wsg_gripper;

static const std::string command_names[] = {
#define X(name, id) #name,
#include <weiss_wsg_gripper/gripper_commands.def>
#undef X
    };

static const int command_ids[] = {
#define X(name, id) id,
#include <weiss_wsg_gripper/gripper_commands.def>
#undef X
    };

static const std::string error_code_names[] = {
#define X(name) #name,
#include <weiss_wsg_gripper/gripper_error_codes.def>
#undef X
    };

static const Gripper::SystemStateFlag system_state_flag_values[] = {
#define X(name, bit_number) Gripper::name,
#include <weiss_wsg_gripper/gripper_system_state_flags.def>
#undef X
    };

static const std::string system_state_flag_names[] = {
#define X(name, bit_number) #name,
#include <weiss_wsg_gripper/gripper_system_state_flags.def>
#undef X
    };

static const std::string grasping_state_names[] = {
#define X(name) #name,
#include <weiss_wsg_gripper/gripper_grasping_states.def>
#undef X
    };

Gripper::Gripper(const std::string & transport_uri, bool debug_output_enabled, float io_timeout)
    : async_packet_signal_(), connected_(false), debug_output_enabled_(debug_output_enabled), io_timeout_(io_timeout)
{
  //ROS_INFO("%s: start of constructor; creating transport", __func__);
  transport_ = Transport::createFromUri(transport_uri);
  //ROS_INFO("%s: created transport, creating packet receiver", __func__);
  packet_receiver_ = boost::make_shared<GripperPacketReceiver>();
  //ROS_INFO("%s: end of constructor", __func__);
}

Gripper::~Gripper()
{
  disconnect();
}

void Gripper::setDebugOutputEnabled(bool debug_output_enabled)
{
  debug_output_enabled_ = debug_output_enabled;
}

boost::signals::connection Gripper::registerAsyncPacketCallback(
    AsyncPacketCallback callback)
{
  BOOST_ASSERT(callback);
  return async_packet_signal_.connect(callback);
}

void Gripper::connect()
{
  if (connected_)
    throw GripperIllegalStateException(
        "tried to connect gripper that's already connected");

  if (debug_output_enabled_)
      ROS_INFO("Gripper: connecting transport...");
  transport_->connect();

//  if (debug_output_enabled_)
//    ROS_INFO("Testing connection by sending loop packet...");
//  GripperPacketPayload payload;
//  payload.writeUint32(0xDEAD);
//  payload.writeUint32(0xBEEF);
//  transceivePacket(CMD_LOOP, 0, payload, &payload);
//  ROS_INFO("Checking packet");

//  if (!(payload.readUint32() == 0xDEAD && payload.readUint32() == 0xBEEF)) {
//    transport_->disconnect();
//    throw GripperInternalErrorException(
//        "Gripper::connect failed: loop command returned something different from what was sent");
//  }

  if (debug_output_enabled_)
      ROS_INFO("Gripper: connected.");
  connected_ = true;
}

void Gripper::disconnect()
{
  if (connected_)
  {
    connected_ = false;
    transport_->disconnect();
  }
}

bool Gripper::isConnected()
{
  return connected_;
}

void Gripper::checkForAsyncPackets()
{
  std::vector<unsigned char> buffer;

  while (true)
  {
    buffer.clear();
    transport_->receive(buffer, 0);
    if (buffer.empty())
      break;

    if (debug_output_enabled_)
    {
      std::ostringstream ss;
      for (size_t i = 0; i < buffer.size(); ++i)
        ss << " 0x" << std::hex << (unsigned int) buffer.at(i);
      ROS_INFO("Gripper: received packet: [%s]", ss.str().c_str());
    }

    for (size_t i = 0; i < buffer.size(); ++i)
    {
      if (packet_receiver_->feed(buffer.at(i)))
        dispatchAsyncPacket();
    }
  }
}

void Gripper::executeHomingSequence(HomingDirection direction)
{
  GripperPacketPayload payload;
  payload.writeUint8(direction);
  transceivePacket(CMD_HOMING, 0, payload, 0);
}

void Gripper::prepositionFingers(BlockingBehaviour bb, MovementType mt,
    float width, float speed)
{
  GripperPacketPayload payload;
  payload.writeUint8(bb << 1 | mt);
  payload.writeFloat(width);
  payload.writeFloat(speed);
  transceivePacket(CMD_PREPOSITION_FINGERS, 0, payload, 0);
}

void Gripper::transmitPrepositionFingersCommand(BlockingBehaviour bb, MovementType mt, float width,
                                   float speed)
{
    GripperPacketPayload payload;
    payload.writeUint8(bb << 1 | mt);
    payload.writeFloat(width);
    payload.writeFloat(speed);
    transmitPacket(CMD_PREPOSITION_FINGERS, payload);
}


void Gripper::stop()
{
  transceivePacket(CMD_STOP, 0, GripperPacketPayload::EMPTY_PAYLOAD, 0);
}

void Gripper::transmitStopCommand()
{
    transmitPacket(CMD_STOP, GripperPacketPayload::EMPTY_PAYLOAD);
}

void Gripper::issueFastStop()
{
  transceivePacket(CMD_ISSUE_FAST_STOP, 0, GripperPacketPayload::EMPTY_PAYLOAD,
      0);
}

void Gripper::acknowledgeFastStop()
{
  GripperPacketPayload payload;
  payload.writeString("ack");
  transceivePacket(CMD_ACK_FAST_STOP, 0, payload, 0);
}

void Gripper::graspPart(float width, float speed)
{
  //ROS_INFO("Gripper::graspPart called");
  GripperPacketPayload payload;
  payload.writeFloat(width);
  payload.writeFloat(speed);
  transceivePacket(CMD_GRASP_PART, 0, payload, 0);
}

void Gripper::releasePart(float open_width, float speed)
{
  GripperPacketPayload payload;
  payload.writeFloat(open_width);
  payload.writeFloat(speed);
  transceivePacket(CMD_RELEASE_PART, 0, payload, 0);
}

void Gripper::setAcceleration(float acceleration)
{
  GripperPacketPayload payload;
  payload.writeFloat(acceleration);
  transceivePacket(CMD_SET_ACCELERATION, 0, payload, 0);
}

float Gripper::getAcceleration()
{
  GripperPacketPayload payload;
  transceivePacket(CMD_GET_ACCELERATION, 0, GripperPacketPayload::EMPTY_PAYLOAD,
      &payload);
  return payload.readFloat();
}

void Gripper::setForceLimit(float force)
{
  GripperPacketPayload payload;
  payload.writeFloat(force);
  transceivePacket(CMD_SET_FORCE_LIMIT, 0, payload, 0);
}

float Gripper::getForceLimit()
{
  GripperPacketPayload payload;
  transceivePacket(CMD_GET_FORCE_LIMIT, 0, GripperPacketPayload::EMPTY_PAYLOAD,
      &payload);
  return payload.readFloat();
}

void Gripper::setSoftLimits(const SoftLimits &limits)
{
  GripperPacketPayload payload;
  payload.writeFloat(limits.limit_minus_);
  payload.writeFloat(limits.limit_plus_);
  transceivePacket(CMD_SET_SOFT_LIMITS, 0, payload, 0);
}

void Gripper::getSoftLimits(SoftLimits &limits)
{
  GripperPacketPayload payload;
  transceivePacket(CMD_GET_SOFT_LIMITS, 0, GripperPacketPayload::EMPTY_PAYLOAD,
      &payload);
  limits.limit_minus_ = payload.readFloat();
  limits.limit_plus_ = payload.readFloat();
}

void Gripper::clearSoftLimits()
{
  transceivePacket(CMD_CLEAR_SOFT_LIMITS, 0,
      GripperPacketPayload::EMPTY_PAYLOAD, 0);
}

void Gripper::setOverdriveMode(bool enabled)
{
  GripperPacketPayload payload;
  payload.writeUint8(enabled ? 1 : 0);
  transceivePacket(CMD_OVERDRIVE_MODE, 0, payload, 0);
}

void Gripper::tareForceSensor()
{
  transceivePacket(CMD_TARE_FORCE_SENSOR, 0,
      GripperPacketPayload::EMPTY_PAYLOAD, 0);
}

unsigned int Gripper::getSystemState(AutoUpdateMode mode,
    unsigned short minimum_interval)
{
  GripperPacketPayload payload;
  payload.writeUint8(mode);
  payload.writeUint16(minimum_interval);
  transceivePacket(CMD_GET_SYSTEM_STATE, 0, payload, &payload);
  return payload.readUint32();
}

Gripper::GraspingState Gripper::getGraspingState(AutoUpdateMode mode,
    unsigned short minimum_interval)
{
  GripperPacketPayload payload;
  payload.writeUint8(mode);
  payload.writeUint16(minimum_interval);
  transceivePacket(CMD_GET_GRASPING_STATE, 0, payload, &payload);
  return static_cast<GraspingState>(payload.readUint8());
}

void Gripper::getGraspingStatistics(bool reset_after_reading,
    GraspingStatistics &statistics)
{
  GripperPacketPayload payload;
  payload.writeUint8(reset_after_reading ? 1 : 0);
  transceivePacket(CMD_GET_GRASPING_STATISTICS, 0, payload, &payload);
  statistics.total_count_ = payload.readUint32();
  statistics.no_part_count_ = payload.readUint16();
  statistics.lost_part_count_ = payload.readUint16();
}

float Gripper::getOpeningWidth(AutoUpdateMode mode,
    unsigned short minimum_interval)
{
  GripperPacketPayload payload;
  payload.writeUint8(mode);
  payload.writeUint16(minimum_interval);
  transceivePacket(CMD_GET_OPENING_WIDTH, 0, payload, &payload);
  return payload.readFloat();
}

float Gripper::getSpeed(AutoUpdateMode mode, unsigned short minimum_interval)
{
  GripperPacketPayload payload;
  payload.writeUint8(mode);
  payload.writeUint16(minimum_interval);
  transceivePacket(CMD_GET_SPEED, 0, payload, &payload);
  return payload.readFloat();
}

float Gripper::getForce(AutoUpdateMode mode, unsigned short minimum_interval)
{
  GripperPacketPayload payload;
  payload.writeUint8(mode);
  payload.writeUint16(minimum_interval);
  transceivePacket(CMD_GET_FORCE, 0, payload, &payload);
  return payload.readFloat();
}

void Gripper::getSystemInformation(SystemInformation &system_information)
{
  GripperPacketPayload payload;
  transceivePacket(CMD_GET_SYSTEM_INFORMATION, 0,
      GripperPacketPayload::EMPTY_PAYLOAD, &payload);
  system_information.gripper_type_ = payload.readUint8();
  system_information.hardware_revision_ = payload.readUint8();
  system_information.software_revision_ = payload.readUint16();
  system_information.serial_number_ = payload.readUint32();
}

void Gripper::setDeviceTag(const std::string &device_tag)
{
  GripperPacketPayload payload;
  payload.writeString(device_tag);
  transceivePacket(CMD_SET_DEVICE_TAG, 0, payload, 0);
}

std::string Gripper::getDeviceTag()
{
  GripperPacketPayload payload;
  transceivePacket(CMD_GET_DEVICE_TAG, 0, GripperPacketPayload::EMPTY_PAYLOAD,
      &payload);
  return payload.readString(payload.getReadableSize());
}

void Gripper::getSystemLimits(SystemLimits &system_limits)
{
  GripperPacketPayload payload;
  transceivePacket(CMD_GET_SYSTEM_LIMITS, 0,
      GripperPacketPayload::EMPTY_PAYLOAD, &payload);
  system_limits.stroke_ = payload.readFloat();
  system_limits.min_speed_ = payload.readFloat();
  system_limits.max_speed_ = payload.readFloat();
  system_limits.min_acceleration_ = payload.readFloat();
  system_limits.max_acceleration_ = payload.readFloat();
  system_limits.min_force_ = payload.readFloat();
  system_limits.nominal_force_ = payload.readFloat();
  system_limits.overdrive_force_ = payload.readFloat();
}

const std::string &Gripper::getCommandName(Command command)
{
  return command_names[command];
}

int Gripper::getCommandId(Command command)
{
  return command_ids[command];
}

bool Gripper::getCommandFromId(int command_id, Command &command)
{
  const int length = ARRAY_LENGTH(command_ids);
  for (int i = 0; i < length; ++i)
  {
    if (command_ids[i] == command_id)
    {
      command = static_cast<Command>(i);
      return true;
    }
  }
  return false;
}

const std::string &Gripper::getErrorCodeName(ErrorCode error_code)
{
  return error_code_names[error_code];
}

const std::string &Gripper::getSystemStateFlagName(SystemStateFlag flag)
{
  const int length = ARRAY_LENGTH(system_state_flag_values);
  for (int i = 0; i < length; ++i)
  {
    if (system_state_flag_values[i] == flag)
      return system_state_flag_names[i];
  }
  throw GripperIllegalArgumentException(
      "getSystemStateFlagName: illegal SystemStateFlag value");
}

std::vector<Gripper::SystemStateFlag> Gripper::expandSystemStateFlags(
    unsigned int flags)
{
  std::vector<SystemStateFlag> result;
  const int length = ARRAY_LENGTH(system_state_flag_values);
  for (int i = 0; i < length; ++i)
  {
    if ((flags & system_state_flag_values[i]) != 0)
      result.push_back(system_state_flag_values[i]);
  }
  return result;
}

const std::string &Gripper::getGraspingStateName(GraspingState grasping_state)
{
  if ((0 <= grasping_state)
      && (grasping_state < ARRAY_LENGTH(grasping_state_names)))
    return grasping_state_names[grasping_state];
  throw GripperIllegalArgumentException(
      "Gripper::getGraspingStateName: illegal GraspingState value");
}

void Gripper::transceivePacket(Command command, ErrorCode *error_code,
    const GripperPacketPayload &transmit_payload,
    GripperPacketPayload *receive_payload)
{
  ErrorCode ec;

  transmitPacket(command, transmit_payload);
  if (receive_payload != 0)
  {
    receivePacket(command, ec, *receive_payload);
  }
  else
  {
    GripperPacketPayload dummy_payload;
    receivePacket(command, ec, dummy_payload);
  }
  if (error_code != 0)
    *error_code = ec;

  switch (ec)
  {
  case E_SUCCESS:
  case E_CMD_PENDING:
    // should be OK.
    break;

  case E_CHECKSUM_ERROR:
  case E_NO_PARAM_EXPECTED:
  case E_NOT_ENOUGH_PARAMS:
  case E_CMD_UNKNOWN:
  case E_CMD_FORMAT_ERROR:
    throw GripperInternalErrorException(
        "protocol error (" + getErrorCodeName(ec) + ")");
    break;

  default:
    throw GripperCommandFailureException(command, ec);
  }
}

void Gripper::transmitPacket(Command command,
    const GripperPacketPayload &payload)
{
  std::vector<unsigned char> raw_packet;

  //ROS_INFO(
  //		"Gripper::transmitPacket called with command = %d and payload = %p", command, &payload);

  raw_packet.push_back(0xAA);
  raw_packet.push_back(0xAA);
  raw_packet.push_back(0xAA);
  raw_packet.push_back(getCommandId(command));

  const std::vector<unsigned char> &payload_contents = payload.getContents();
  raw_packet.push_back(payload_contents.size());
  raw_packet.push_back(payload_contents.size() >> 8);
  raw_packet.insert(raw_packet.end(), payload_contents.begin(),
      payload_contents.end());

  GripperPacketChecksum checksum;
  checksum.feed(raw_packet);
  unsigned short cs = checksum.getValue();
  raw_packet.push_back(cs);
  raw_packet.push_back(cs >> 8);

  if (debug_output_enabled_)
  {
    std::ostringstream ss;
    for (size_t i = 0; i < raw_packet.size(); ++i)
      ss << " 0x" << std::hex << (unsigned int) raw_packet.at(i);
    ROS_INFO("Gripper: transmitting packet: [%s]", ss.str().c_str());
  }

  transport_->transmit(raw_packet);
  transport_->flush();
}

void Gripper::receivePacket(const Command command, ErrorCode &error_code,
    GripperPacketPayload &payload)
{
  std::vector<unsigned char> buffer;
  bool received = false;
  boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration timeout = boost::posix_time::microseconds(io_timeout_ / 1e-6);

  while (!received && (boost::posix_time::microsec_clock::local_time() - start_time) < timeout)
  {
    if (debug_output_enabled_)
      ROS_INFO("Gripper: waiting for packet with command %s", getCommandName(command).c_str());

    buffer.clear();
    transport_->receive(buffer, io_timeout_);

    if (debug_output_enabled_)
    {
      std::ostringstream ss;
      for (size_t i = 0; i < buffer.size(); ++i)
        ss << " 0x" << std::hex << (unsigned int) buffer.at(i);
      ROS_INFO("Gripper: received packet: [%s]", ss.str().c_str());
    }

    for (size_t i = 0; i < buffer.size(); ++i)
    {
      if (packet_receiver_->feed(buffer.at(i)))
      {
        if (packet_receiver_->getCommand() == command)
        {
          payload.setContents(packet_receiver_->getPayload());
          error_code = static_cast<ErrorCode>(payload.readUint16());
          received = true;
        }
        else
        {
          dispatchAsyncPacket();
        }
      }
    }
  }

  if (!received)
      throw GripperTimeoutException(std::string(__func__) + " timed out");
}

void Gripper::dispatchAsyncPacket()
{
  if (!async_packet_signal_.empty())
  {
    // extract info from packet:
    AsyncPacketInfo packet_info;
    packet_info.command_ = packet_receiver_->getCommand();

    GripperPacketPayload gpp(packet_receiver_->getPayload());
    packet_info.error_code_ = static_cast<ErrorCode>(gpp.readUint16());

    switch (gpp.getReadableSize())
    {
    case 0:
      packet_info.payload_.as_uint_ = 0;
      break;

    case 1:
      packet_info.payload_.as_uint_ = gpp.readUint8();
      break;

    case 2:
      packet_info.payload_.as_uint_ = gpp.readUint16();
      break;

    case 4:
      packet_info.payload_.as_uint_ = gpp.readUint32();
      break;

    default:
      throw GripperInternalErrorException(
          "illegal async packet payload size: "
              + boost::lexical_cast<std::string>(gpp.getReadableSize()));
    }

    // emit signal:
    async_packet_signal_(packet_info);
  }
}

