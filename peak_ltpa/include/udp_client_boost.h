#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <boost/asio.hpp>

//using namespace boost::asio;
//using ip::udp;


namespace BoostSocketWrappers {

class UdpClientBoost {
public:
    UdpClientBoost(const std::string& ip_address, const int& port);
    ~UdpClientBoost();

    std::vector<unsigned char>         receive(const size_t& size);
    //void                               send(const std::string& message);

private:
    std::string                        ip_address_;
    int                                port_;

    boost::asio::io_context            io_context_;
    boost::asio::ip::udp::endpoint     remote_endpoint_;

public:
    boost::asio::ip::udp::socket       socket_;
};

} // namespace BoostSocketWrappers