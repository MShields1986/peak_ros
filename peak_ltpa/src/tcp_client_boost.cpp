#include "tcp_client_boost.h"



namespace BoostSocketWrappers {

TcpClientBoost::TcpClientBoost(const std::string& ip_address, const int& port)
     : ip_address_(ip_address), 
       port_(port),
       io_context_(),
       remote_endpoint_(boost::asio::ip::address::from_string(ip_address), port),
       socket_(io_context_)
{
}


TcpClientBoost::~TcpClientBoost() {
    socket_.close();
}


void TcpClientBoost::connect() {
    socket_.connect(remote_endpoint_);
}


std::vector<unsigned char> TcpClientBoost::receive(const size_t& size) {
    boost::asio::streambuf buf;
    boost::asio::streambuf::mutable_buffers_type bufs = buf.prepare(size); // bytes

    boost::system::error_code error;

    size_t n_tot = boost::asio::read(this->socket_, bufs, error);

    if(error) {
        std::cout << "TCPClientBoost : Read error: " << error << std::endl;
    }

    buf.commit(n_tot);

    std::vector<unsigned char> data(size);
    boost::asio::buffer_copy(boost::asio::buffer(data), buf.data());

    return data;
}


void TcpClientBoost::send(const std::string& message) {
  const std::string msg = message;
  boost::asio::write(socket_, boost::asio::buffer(msg));
}

} // namespace BoostSocketWrappers