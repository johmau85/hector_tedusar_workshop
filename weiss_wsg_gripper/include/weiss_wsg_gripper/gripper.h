/*
 * gripper.h
 *
 *  Created on: 27.11.2012
 *      Author: alex
 */

#ifndef WEISS_WSG_GRIPPER_GRIPPER_H_
#define WEISS_WSG_GRIPPER_GRIPPER_H_

#include <list>
#include <string>
#include <vector>
#include <boost/function.hpp>
#include <boost/signals.hpp>

namespace weiss_wsg_gripper
{

class GripperPacketPayload;
class GripperPacketReceiver;
class Transport;
// FIXME add documentation to enums and functions
class Gripper {
public:
  enum Command
  {
#define X(name, id) name,
#include "weiss_wsg_gripper/gripper_commands.def"
#undef X
  };

  enum ErrorCode
  {
#define X(name) name,
#include "weiss_wsg_gripper/gripper_error_codes.def"
#undef X
  };

  enum SystemStateFlag
  {
#define X(name, bit_number) name = 1<<bit_number,
#include "weiss_wsg_gripper/gripper_system_state_flags.def"
#undef X
  };

  enum HomingDirection
  {
    HOMING_DIRECTION_DEFAULT = 0,
    HOMING_DIRECTION_POSITIVE = 1,
    HOMING_DIRECTION_NEGATIVE = 2,
  };

  enum BlockingBehaviour
  {
    BLOCKING_CLAMP_ON_BLOCK = 0, BLOCKING_STOP_ON_BLOCK = 1,
  };

  enum MovementType
  {
    MOVEMENT_TYPE_ABSOLUTE = 0, MOVEMENT_TYPE_RELATIVE = 1,
  };

  enum AutoUpdateMode
  {
    AUTO_UPDATE_DISABLED = 0, AUTO_UPDATE_ALWAYS = 1, AUTO_UPDATE_ON_CHANGE = 3,
  };

  enum GraspingState
  {
#define X(name) name,
#include "weiss_wsg_gripper/gripper_grasping_states.def"
#undef X
  };

  struct SoftLimits
  {
    float limit_minus_;
    float limit_plus_;
  };

  struct GraspingStatistics
  {
    unsigned int total_count_;
    unsigned short no_part_count_;
    unsigned short lost_part_count_;
  };

  struct SystemInformation
  {
    enum GripperType
    {
      GRIPPER_TYPE_UNKNOWN = 0, GRIPPER_TYPE_WSG_50 = 1,
    };

    unsigned char gripper_type_;
    unsigned char hardware_revision_;
    unsigned short software_revision_;
    unsigned int serial_number_;
  };

  struct SystemLimits
  {
    float stroke_;
    float min_speed_;
    float max_speed_;
    float min_acceleration_;
    float max_acceleration_;
    float min_force_; // why not max force?
    float nominal_force_;
    float overdrive_force_;
  };

  struct AsyncPacketInfo
  {
    Command command_;
    ErrorCode error_code_;
    union
    {
      unsigned int as_uint_;
      float as_float_;
    } payload_;
  };

  typedef void AsyncPacketCallbackSignature(const AsyncPacketInfo &);
  typedef boost::function<AsyncPacketCallbackSignature> AsyncPacketCallback;

  /**
   * Constructs a Gripper using a transport specified by a transport URI.
   * @param transport_uri URI to create the transport from. Currently, the
   *   following formats are supported (without the angle brackets):
   *   - can:<name of CAN node>:<CAN id> constructs a CAN node transport,
   *     e.g. can::100. Name of CAN node defaults to
   *     can_node::DEFAULT_CAN_NODE_NAME.
   *   - tcpip:<host name>:<port number> constructs a TCP/IP transport, e.g.
   *     tcpip:192.168.5.10:1000.
   * @throw std::invalid_argument if an invalid URI is passed.
   */
  Gripper(const std::string & transport_uri, bool debug_output_enabled, float io_timeout);
  ~Gripper();

  void setDebugOutputEnabled(bool debug_output_enabled);
  boost::signals::connection registerAsyncPacketCallback(
      AsyncPacketCallback callback);

  void connect();
  void disconnect();
  bool isConnected();

  void checkForAsyncPackets();

  void executeHomingSequence(HomingDirection direction);

  void prepositionFingers(BlockingBehaviour bb, MovementType mt, float width,
      float speed);
  // Quickfix for schunk_lpa4w_control: executes Preposition Fingers command without waiting for result.
  void transmitPrepositionFingersCommand(BlockingBehaviour bb, MovementType mt, float width,
      float speed);

  void stop();
  void issueFastStop();
  void acknowledgeFastStop();
  void graspPart(float width, float speed);
  void releasePart(float open_width, float speed);
  void setAcceleration(float acceleration);
  float getAcceleration();
  void setForceLimit(float force);
  float getForceLimit();
  void setSoftLimits(const SoftLimits &limits);
  void getSoftLimits(SoftLimits &limits);
  void clearSoftLimits();
  void setOverdriveMode(bool enabled);
  void tareForceSensor();
  unsigned int getSystemState(AutoUpdateMode mode,
      unsigned short minimum_interval);
  GraspingState getGraspingState(AutoUpdateMode mode,
      unsigned short minimum_interval);
  void getGraspingStatistics(bool reset_after_reading,
      GraspingStatistics &statistics);
  float getOpeningWidth(AutoUpdateMode mode, unsigned short minimum_interval);
  float getSpeed(AutoUpdateMode mode, unsigned short minimum_interval);
  float getForce(AutoUpdateMode mode, unsigned short minimum_interval);
  void getSystemInformation(SystemInformation &system_information);
  void setDeviceTag(const std::string &device_tag);
  std::string getDeviceTag();
  void getSystemLimits(SystemLimits &system_limits);

  static const std::string &getCommandName(Command command);
  static int getCommandId(Command command);
  static bool getCommandFromId(int command_id, Command &command);
  static const std::string &getErrorCodeName(ErrorCode error_code);
  static const std::string &getSystemStateFlagName(SystemStateFlag flag);
  static std::vector<SystemStateFlag> expandSystemStateFlags(
      unsigned int flags);
  static const std::string &getGraspingStateName(GraspingState grasping_state);

private:
  Gripper(const Gripper &);
  Gripper &operator=(const Gripper &);

  void transceivePacket(Command command, ErrorCode *error_code,
      const GripperPacketPayload &transmit_payload,
      GripperPacketPayload *receive_payload);
  void transmitPacket(Command command, const GripperPacketPayload &payload);
  void receivePacket(Command command, ErrorCode &error_code,
      GripperPacketPayload &payload);
  void dispatchAsyncPacket();

  boost::shared_ptr<Transport> transport_;
  boost::shared_ptr<GripperPacketReceiver> packet_receiver_;
  boost::signal<AsyncPacketCallbackSignature> async_packet_signal_;
  bool connected_;
  bool debug_output_enabled_;
  float io_timeout_;
};

}

#endif /* WEISS_WSG_GRIPPER_GRIPPER_H_ */
