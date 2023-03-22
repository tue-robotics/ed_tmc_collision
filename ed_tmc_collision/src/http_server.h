#ifndef http_server_h_
#define http_server_h_

#include "boost_http_server.h"

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/address.hpp>

#include <memory>
#include <thread>
#include <string>
#include <vector>


namespace net = boost::asio; // from <boost/asio.hpp>


class HTTPServer
{

public:
    HTTPServer() = delete;

    HTTPServer(const std::string& address, const unsigned short port, const std::string& doc_root);

    virtual ~HTTPServer();

    void run(unsigned int threads=1);

    void stop();

private:
    // Configuration
    const net::ip::address address_;
    const unsigned short port_;
    const std::shared_ptr<const std::string> doc_root_;

    std::unique_ptr<net::io_context> ioc_;

    std::shared_ptr<listener> listener_;

    std::vector<std::thread> threads_;

};

#endif
