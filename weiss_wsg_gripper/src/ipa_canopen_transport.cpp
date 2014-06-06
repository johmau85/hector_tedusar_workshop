#include <weiss_wsg_gripper/ipa_canopen_transport.h>
#include <algorithm>
#include <ipa_canopen_core/canopen.h>
#include <weiss_wsg_gripper/gripper_exceptions.h>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>

// Using a static function here so we don't have to include canopen.h in header file:
static void handleIncomingCanMessage(weiss_wsg_gripper::IpaCanopenTransport * transport, const TPCANRdMsg m) {
    transport->handleReceivedCanMessage(m.Msg.DATA, m.Msg.LEN);
}

namespace weiss_wsg_gripper {

IpaCanopenTransport::IpaCanopenTransport(int can_id)
    : can_id_(can_id)
{
}

IpaCanopenTransport::~IpaCanopenTransport()
{
}

void IpaCanopenTransport::connect()
{
    if (!canopen::isCANConnected())
        throw GripperTransportException("ipa_canopen_core has not yet opened CAN interface; must be done before connecting to gripper");

    canopen::incomingCANMessageHandlers[can_id_ + 1] = boost::bind(handleIncomingCanMessage, this, _1);
}

void IpaCanopenTransport::disconnect()
{
    canopen::incomingCANMessageHandlers.erase(can_id_ + 1);
}

void IpaCanopenTransport::transmit(const std::vector<unsigned char> & buffer)
{
    // Split buffer into chunks of 8 bytes and send a CAN message for each one:
    for (size_t i = 0; i < buffer.size(); i += 8)
    {
        size_t size = std::min<size_t>(8, buffer.size() - i);
        transmitCanMessage(&buffer.at(i), size);
    }
}

void IpaCanopenTransport::flush()
{
    // Ignored.
}

void IpaCanopenTransport::receive(std::vector<unsigned char> & buffer, float timeout)
{
    boost::unique_lock<boost::mutex> lock(receive_buffer_lock_);
    if (receive_buffer_.empty() && timeout > 0)
        receive_buffer_nonempty_.timed_wait(lock, boost::posix_time::microseconds(timeout / 1e-6));
    buffer.assign(receive_buffer_.begin(), receive_buffer_.end());
    receive_buffer_.clear();
}

void IpaCanopenTransport::handleReceivedCanMessage(const unsigned char * data, size_t size)
{
    boost::lock_guard<boost::mutex> lock(receive_buffer_lock_);
    receive_buffer_.insert(receive_buffer_.end(), data, data + size);
    receive_buffer_nonempty_.notify_all();
}

void IpaCanopenTransport::transmitCanMessage(const unsigned char * data, size_t size)
{
    if (size > 8)
        throw GripperTransportException(std::string(__func__) + ": Illegal value for size");

    TPCANMsg can_message;
    std::memset(&can_message, 0, sizeof(can_message));
    can_message.ID = can_id_;
    can_message.MSGTYPE = MSGTYPE_STANDARD;
    can_message.LEN = size;
    for (size_t i = 0; i < size; ++i)
        can_message.DATA[i] = data[i];
    canopen::sendCANMessage(can_message);
}




}

