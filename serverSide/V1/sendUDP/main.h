#ifndef MAIN_H
#define MAIN_H

typedef struct __attribute__((__packed__)) pair_t;
int recvPacks(int sockfd);
int sendPacks(int sockfd, pair_t pair);
int main(int argc, char *argv[]);


#endif // MAIN_H

