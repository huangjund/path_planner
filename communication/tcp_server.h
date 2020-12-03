#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>
#include <boost/asio.hpp>

#include "build/path.pb.h"

namespace HybridAStar {
namespace Communication {
  using boost::asio::ip::tcp;

  class session : public std::enable_shared_from_this<session> {
   public:
    session(tcp::socket&);
    void start() { do_read();}
   private:
    tcp::socket socket_;
    enum {max_length = 1024};
    std::array<char, max_length> data_;

    void do_read();
    void do_write(std::size_t length);

  };

  class server {
   public:
    server(boost::asio::io_context& io_context, short port);

   private:
    void do_accept();
    tcp::acceptor acceptor_;
  };
} // namepsace Communication
} // namespace HybridAStar