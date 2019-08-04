#ifndef TCP_H
#define TCP_H

#include <string.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/poll.h>

int ip_port_info(const int sockfd, char *ip, int *port);

#endif // TCP_H
