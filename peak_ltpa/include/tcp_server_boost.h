#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <boost/asio.hpp>

//using namespace boost::asio;
//using ip::tcp;


namespace BoostSocketWrappers {

class TcpServerBoost {
public:
    TcpServerBoost(const std::string& ip_address, const int& port);
    ~TcpServerBoost();

    std::vector<unsigned char>         receive(const size_t& size);
    void                               send(const std::string& message);

private:
    std::string                        ip_address_;
    int                                port_;

    boost::asio::io_context            io_context_;
    boost::asio::ip::tcp::acceptor     acceptor_;

public:
    boost::asio::ip::tcp::socket       socket_;
};

} // namespace BoostSocketWrappers