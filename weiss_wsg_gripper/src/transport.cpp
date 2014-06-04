/*
 * \file
 * \date 21.01.2014
 * \author Alexander Buchegger
 */
#include <weiss_wsg_gripper/transport.h>
#include <sstream>
#include <stdexcept>
#include <boost/algorithm/string.hpp>
#include <boost/make_shared.hpp>
#include <weiss_wsg_gripper/ipa_canopen_transport.h>

namespace weiss_wsg_gripper
{

Transport::Transport()
{
}

Transport::~Transport()
{
}

TransportPtr Transport::createFromUri(const std::string & uri)
{
  TransportPtr transport;
  std::vector<std::string> parts;
  boost::split(parts, uri, boost::is_any_of(":"));

  if (parts.size() == 2 && parts[0] == "ipa_canopen" && !parts[2].empty())
  {
    int can_id = atoi(parts[1].c_str());

    transport = boost::make_shared<IpaCanopenTransport>(can_id);
  }
  else
  {
    throw std::invalid_argument("weiss_wsg_gripper::Transport::createFromUri: "
        "Invalid URI (" + uri + ")");
  }
  return transport;
}

}
