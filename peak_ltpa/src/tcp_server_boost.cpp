#include "tcp_server_boost.h"



namespace BoostSocketWrappers {

TcpServerBoost::TcpServerBoost(const std::string& ip_address, const int& port)
     : ip_address_(ip_address), 
       port_(port),
       io_context_(),
       acceptor_(io_context_),
       socket_(io_context_)
{
    acceptor_.open(boost::asio::ip::tcp::v4());

    int one = 1;
    setsockopt(acceptor_.native_handle(), SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &one, sizeof(one));

    acceptor_.bind( boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port_) );
    acceptor_.listen();
    acceptor_.accept(socket_);
}


TcpServerBoost::~TcpServerBoost() {
    socket_.close();
}


std::vector<unsigned char> TcpServerBoost::receive(const size_t& size) {
    boost::asio::streambuf buf;
    boost::asio::streambuf::mutable_buffers_type bufs = buf.prepare(size);

    boost::system::error_code error;

    size_t n = boost::asio::read(this->socket_, bufs, error);

    if(error) {
        std::cout << "TCPServerBoost : Read error: " << error << std::endl;
    }

    buf.commit(n);

    std::vector<unsigned char> data(size);
    boost::asio::buffer_copy(boost::asio::buffer(data), buf.data());

    return data;
}


void TcpServerBoost::send(const std::string& message) {
  const std::string msg = message;
  boost::asio::write(socket_, boost::asio::buffer(msg));
}

} // namespace BoostSocketWrappers