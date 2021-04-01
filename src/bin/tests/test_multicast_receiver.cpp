//
// receiver.cpp
// ~~~~~~~~~~~~
//
// Copyright (c) 2003-2008 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include "boost/bind.hpp"

#include "travesim_adapters/multicast_receiver.hh"

const std::string listen_address_str = "0.0.0.0";
const std::string multicast_address_str = "239.255.0.1";
const short multicast_port = 30001;

std::array<char, 1024> data_buff;

int main(int argc, char* argv[]) {
    try {
        boost::asio::io_context io_context;
        MulticastReceiver my_receiver = MulticastReceiver(io_context, multicast_address_str, multicast_port);

        size_t data_size = 0;

        for(long int i = 0;; i++) {
            data_size = my_receiver.receive(&data_buff);

            if (data_size > 0) {
                std::cout.write(data_buff.data(), data_size);
                std::cout << std::endl;
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
