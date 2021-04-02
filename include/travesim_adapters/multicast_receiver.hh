#include <boost/asio.hpp>
#include <string>

#ifndef __MULTICAST_RECEIVER_H__
#define __MULTICAST_RECEIVER_H__

class MulticastReceiver {
  public:
    MulticastReceiver(const std::string multicast_address, const short multicast_port, const std::string listener_address);

    // Uses multicast address as listen address
    MulticastReceiver(const std::string multicast_address, const short multicast_port);

    size_t receive(std::array<char, 1024>* p_buffer);

  private:
    boost::asio::io_context io_context;
    boost::asio::ip::udp::socket socket;
    boost::asio::ip::udp::endpoint sender_endpoint;
    boost::asio::ip::udp::endpoint listener_endpoint;

    void configure_socket(const boost::asio::ip::address multicast_ip);
};

#endif // __MULTICAST_RECEIVER_H__
