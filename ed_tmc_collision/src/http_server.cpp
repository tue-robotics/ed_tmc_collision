#include "http_server.h"

#include <boost/asio/ip/tcp.hpp>

using tcp = boost::asio::ip::tcp; // from <boost/asio/ip/tcp.hpp>

HTTPServer::HTTPServer(const std::string& address,
                       const unsigned short port,
                       const std::string& doc_root)
    : address_(net::ip::make_address(address)), port_(port), doc_root_(std::make_shared<const std::string>(doc_root))
{
}

HTTPServer::~HTTPServer()
{
    stop();
}

void HTTPServer::run(unsigned int threads)
{
    stop();
    ioc_ = std::make_unique<net::io_context>(threads);

    // Create and launch a listening port
    listener_ = std::make_shared<listener>(*ioc_, tcp::endpoint{address_, port_}, doc_root_);
    listener_->run();

    // Run the I/O service on the requested number of threads
    threads_.reserve(threads);
    for (unsigned int i = threads; i > 0; --i)
    {
        threads_.emplace_back(
        [this]
        {
            ioc_->run();
        });
    }
}

void HTTPServer::stop()
{
    if (ioc_)
        if (!ioc_->stopped())
            ioc_->stop();

    for (std::thread& thread : threads_)
    {
        if (thread.joinable())
            thread.join();
    }
    threads_.clear();
}
