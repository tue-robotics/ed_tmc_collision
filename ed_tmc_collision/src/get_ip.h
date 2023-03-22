#ifndef GET_IP_H_
#define GET_IP_H_

#ifndef _GNU_SOURCE
#define _GNU_SOURCE /* To get defns of NI_MAXSERV and NI_MAXHOST */
#endif

#include <ros/console.h>

#include <ifaddrs.h>
#include <netdb.h>
#include <string>
#include <stdexcept>
#include <sys/socket.h>
#include <vector>

std::string getIPAddress(const std::string& interface, const std::vector<int> families={AF_INET})
{
   struct ifaddrs *ifaddr;
   int family, s;
   char host[NI_MAXHOST];

   std::string ip;

   if (getifaddrs(&ifaddr) == -1)
   {
       throw std::runtime_error("getifaddrs");
   }

   /* Walk through linked list, maintaining head pointer so we can free list later. */
   for (struct ifaddrs *ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
   {
       if (ifa->ifa_addr == NULL)
           continue;

        family = ifa->ifa_addr->sa_family;
        bool family_match = false;
        for (const int test_family : families)
            if (family == test_family)
               family_match = true;

        if (!family_match)
            // Skip all other interface than defined
            continue;

        if (std::string(ifa->ifa_name) != interface)
           // Skip if name not matching
           continue;

        /* For an AF_INET* interface address, display the address. */
        s = getnameinfo(ifa->ifa_addr,
                       (family == AF_INET) ? sizeof(struct sockaddr_in) : sizeof(struct sockaddr_in6),
                       host,
                       NI_MAXHOST,
                       NULL,
                       0,
                       NI_NUMERICHOST);

        if (s != 0)
        {
            throw std::runtime_error(std::string("getnameinfo() failed: ") + gai_strerror(s));
        }
        ip = host;
    }

    freeifaddrs(ifaddr);

    return ip;
}

#endif
