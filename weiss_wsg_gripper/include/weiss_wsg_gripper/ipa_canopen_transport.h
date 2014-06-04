#ifndef WEISS_WSG_GRIPPER_IPA_CANOPEN_TRANSPORT_H_
#define WEISS_WSG_GRIPPER_IPA_CANOPEN_TRANSPORT_H_

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <weiss_wsg_gripper/transport.h>

namespace weiss_wsg_gripper {

class IpaCanopenTransport : public Transport {
public:
    IpaCanopenTransport(int can_id);
    virtual ~IpaCanopenTransport();

    virtual void connect();
    virtual void disconnect();
    virtual void transmit(const std::vector<unsigned char> & buffer);
    virtual void flush();
    virtual void receive(std::vector<unsigned char> & buffer, bool blocking);

    void handleReceivedCanMessage(const unsigned char * data, size_t size);

private:
    void transmitCanMessage(const unsigned char * data, size_t size);

    int can_id_;

    boost::mutex receive_buffer_lock_;
    boost::condition_variable receive_buffer_nonempty_;
    std::vector<unsigned char> receive_buffer_;
};

}

#endif
