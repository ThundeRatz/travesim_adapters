#include <boost/asio.hpp>
#include <string>

#ifndef __MULTICAST_RECEIVER_H__
#define __MULTICAST_RECEIVER_H__

namespace travesim {
namespace udp {

class MulticastReceiver {
  public:
    MulticastReceiver(const std::string multicast_address, const short multicast_port,
                      const std::string listener_address);

    // Uses multicast address as listen address
    MulticastReceiver(const std::string multicast_address, const short multicast_port);

    ~MulticastReceiver();

    size_t receive(char* buffer, const size_t buffer_size);

  private:
    boost::asio::io_context io_context;
    boost::asio::ip::udp::socket* socket;
    boost::asio::ip::udp::endpoint sender_endpoint;
    boost::asio::ip::udp::endpoint listener_endpoint;

    void create_socket(const boost::asio::ip::address multicast_ip);
};

} // namespace udp
} // namespace travesim

#endif  // __MULTICAST_RECEIVER_H__
