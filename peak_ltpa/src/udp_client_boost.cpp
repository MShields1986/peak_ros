#include "udp_client_boost.h"



namespace BoostSocketWrappers {

UdpClientBoost::UdpClientBoost(const std::string& ip_address, const int& port)
     : ip_address_(ip_address), 
       port_(port),
       io_context_(),
       socket_(io_context_, {boost::asio::ip::udp::v4(), (short unsigned int)port_})
{
}


UdpClientBoost::~UdpClientBoost() {
    socket_.close();
}


std::vector<unsigned char> UdpClientBoost::receive(const size_t& size) {
    boost::asio::streambuf buf;
    boost::asio::streambuf::mutable_buffers_type bufs = buf.prepare(size);

    boost::system::error_code error;

    size_t n = this->socket_.receive_from(bufs, remote_endpoint_, 0, error);

    if(error) {
        std::cout << "UdpClientBoost : Read error: " << error << std::endl;
    }

    buf.commit(n);

    std::vector<unsigned char> data(size);
    boost::asio::buffer_copy(boost::asio::buffer(data), buf.data());

    return data;
}

/*
void UdpClientBoost::send(const std::string& message) {
  const std::string msg = message;
  boost::asio::write(socket_, boost::asio::buffer(msg));
}
*/

} // namespace BoostSocketWrappers