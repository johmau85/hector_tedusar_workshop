/*
 * gripper_packet_transport.h
 *
 *  Created on: 27.11.2012
 *      Author: alex
 */

#ifndef WEISS_WSG_GRIPPER_TRANSPORT_H_
#define WEISS_WSG_GRIPPER_TRANSPORT_H_

#include <vector>
#include <boost/shared_ptr.hpp>

namespace weiss_wsg_gripper
{

class Transport;
typedef boost::shared_ptr<Transport> TransportPtr;

/**
 * Abstract base class for the transport layer of the gripper communication protocol.
 */
class Transport
{
public:
  /**
   * Constructor.
   *
   * Implementations should set all parameters needed for their operation using their constructor.
   * @throw GripperIllegalArgumentException  if a supplied parameter has an unsupported value.
   */
  Transport();

  /**
   * Destructor.
   *
   * Implementations should cleanup nicely, potentially performing a disconnect(). Must not throw.
   */
  virtual ~Transport();

  /**
   * Opens a connection to the gripper.
   *
   * Parameters needed for opening the connection should be set using the implementation's constructor.
   * @throw GripperIllegalStateException  if the transport is already connected.
   * @throw GripperTransportException  if opening the connection fails.
   */
  virtual void connect() = 0;

  /**
   * Closes the connection to the gripper.
   *
   * Should not throw. Implementations may assume that the transport will not be used again after disconnect() has
   * been called.
   */
  virtual void disconnect() = 0;

  /**
   * Transmits the contents of @code buffer to the gripper. This will typically be the contents of a single gripper
   * communication packet.
   *
   * Blocks until the data has been transmitted (or at least transferred to the underlying I/O system).
   * @param buffer  data to be transferred to the gripper.
   * @throw GripperIllegalStateException  if the transport is not connected.
   * @throw GripperTransportException  if transmission fails due to an I/O error.
   */
  virtual void transmit(const std::vector<unsigned char> & buffer) = 0;

  /**
   * Makes sure all data given in the last call to transmit() are actually sent (in contrast to being stored in an
   * output buffer until more data is transmitted).
   *
   * This is an optional method; does nothing if a flush operation is not available on the underlying I/O system.
   * @throw GripperIllegalStateException  if the transport is not connected.
   * @throw GripperTransportException  if the operation fails due to an I/O error.
   */
  virtual void flush() = 0;

  /**
   * Receives an arbitrary amount of data from the gripper. This will typically be the contents, or part of the
   * contents, of a gripper communication packet.
   *
   * @param buffer  the output buffer. On exit, contains the received data, and its size represents the amount of
   *                received data.
   * @param blocking  specifies whether the method should block until at least one byte of data is available, or
   *                  should return immediately with an empty buffer if no data is available.
   */
  virtual void receive(std::vector<unsigned char> & buffer, bool blocking) = 0;

  /**
   * Constructs a transport from a URI-style specification.
   * @param uri URI to create the transport from. Currently, the following formats are supported (without the angle
   * brackets):
   *   - ipa_canopen:<CAN id> constructs a IPA-CANopen transport, e.g. ipa_canopen:100.
   *
   * @return the transport.
   *
   * @throw std::invalid_argument if an invalid URI is passed.
   */
  static TransportPtr createFromUri(const std::string & uri);

private:
  Transport(const Transport &);
  Transport &operator=(const Transport &);
};

}

#endif /* WEISS_WSG_GRIPPER_TRANSPORT_H_ */
