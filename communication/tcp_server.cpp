#include "tcp_server.h"

namespace HybridAStar {
namespace Communication {
  session::session(tcp::socket& socket) : 
    socket_(std::move(socket)) {}

  void session::do_read()
  {
    auto self(shared_from_this());
    std::cout << "[===READING!===]" << std::endl;

    // reading until 
    socket_.async_read_some(boost::asio::buffer(data_),
        [this, self](boost::system::error_code ec, std::size_t length)
        {
          if (!ec)
          {
            std::cout << "length: " << length << std::endl;
            do_write(length);
          }
        }
    );
  }

  void session::do_write(std::size_t length)
  {
    auto self(shared_from_this());
    std::cout << "[===writing!===]" << std::endl;

    HybridAStar::Query q;
    q.ParseFromArray(data_, length);
    std::cout << q.id() << "|" << q.question() << std::endl;
    sleep(1);
    boost::asio::async_write(socket_, boost::asio::buffer(data_, length),
        [this, self](boost::system::error_code ec, std::size_t /*length*/)
        {
          if (!ec)
          {
            do_read();
          }
        });
  }

  server::server(boost::asio::io_context& io_context, short port)
    : acceptor_(io_context, tcp::endpoint(tcp::v4(), port))
  {
    do_accept();
  }

  void server::do_accept()
  {
    acceptor_.async_accept(
        [this](boost::system::error_code ec, tcp::socket socket)
        {
          if (!ec)
          {
            std::make_shared<session>(socket)->start();
          }

          do_accept();
        });
  }
} // namespace Communication
} // namespace HybridAStar