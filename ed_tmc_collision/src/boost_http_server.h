#ifndef boost_http_server_h_
#define boost_http_server_h_

#include <boost/beast/core.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <memory>


namespace boost {
namespace asio {
class io_context;
}
}

namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>


// Accepts incoming connections and launches the sessions
class listener : public std::enable_shared_from_this<listener>
{

    net::io_context& ioc_;
    tcp::acceptor acceptor_;
    std::shared_ptr<std::string const> doc_root_;

public:

    listener(net::io_context& ioc,
             tcp::endpoint endpoint,
             std::shared_ptr<std::string const> const& doc_root);

    void
    run();

    private:

    void
    do_accept();

    void
    on_accept(beast::error_code ec, tcp::socket socket);

};

#endif
