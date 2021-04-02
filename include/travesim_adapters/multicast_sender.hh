#include <boost/asio.hpp>
#include <string>

#ifndef __MULTICAST_SENDER_H__
#define __MULTICAST_SENDER_H__

class MulticastSender {
  public:
    MulticastSender(const std::string multicast_address, const short multicast_port);

    ~MulticastSender();

    size_t send(const char* buffer, const size_t buffer_size);

  private:
    boost::asio::io_context io_context;
    boost::asio::ip::udp::socket* socket;
    boost::asio::ip::udp::endpoint endpoint;

};

#endif // __MULTICAST_SENDER_H__
