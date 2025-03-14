#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <boost/asio.hpp>

//using namespace boost::asio;
//using ip::tcp;


namespace BoostSocketWrappers {

class TcpClientBoost {
public:
    TcpClientBoost(const std::string& ip_address, const int& port);
    ~TcpClientBoost();

    void                               connect();
    std::vector<unsigned char>         receive(const size_t& size);
    void                               send(const std::string& message);

private:
    std::string                        ip_address_;
    int                                port_;
    boost::asio::io_context            io_context_;
    boost::asio::ip::tcp::endpoint     remote_endpoint_;

public:
    boost::asio::ip::tcp::socket       socket_;
};

} // namespace BoostSocketWrappers