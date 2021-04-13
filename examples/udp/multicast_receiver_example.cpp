#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include "boost/bind.hpp"

#include "travesim_adapters/udp/multicast_receiver.hpp"

#define BUFFER_SIZE 1024U

const std::string listen_address_str = "0.0.0.0";
const std::string multicast_address_str = "239.255.0.1";
const short multicast_port = 30001;

char data_buff[BUFFER_SIZE];

int main(int argc, char* argv[]) {
    try {
        travesim::udp::MulticastReceiver my_receiver(multicast_address_str, multicast_port);

        size_t data_size = 0;

        for (long int i = 0;; i++) {
            data_size = my_receiver.receive(data_buff, BUFFER_SIZE);

            if (data_size > 0) {
                std::string received_msg(data_buff, data_size);
                std::cout << received_msg << std::endl;
            }

            if (i % 100000 == 0) {
                std::cout << "Loop count: " << i << std::endl;
            }
        }

    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}
