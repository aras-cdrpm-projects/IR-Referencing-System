#include <QCoreApplication>
#include <stdio.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <QUdpSocket>

#define PORT     50000
#define MAXLINE 1024

typedef struct __attribute__((__packed__)) pair_t {
    double d_1;
    double d_2;
    double d_3;
    double d_4;
} pair_t;

int sockfd;
char buffer[MAXLINE];
struct sockaddr_in servaddr, cliaddr;
int len, n;

int recvPacks(int sockfd){

    n = read(sockfd, (char *)buffer, sizeof (buffer));
    return n;
}
int sendPacks(int sockfd, pair_t &pair){
    n = send(sockfd, (struct pair_t*)&pair, sizeof(pair_t), 0);
    //n = write(sockfd, (struct pair_t*)&pair, (sizeof(pair_t)));
    return n;
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    //socket = new QUdpSocket(QWidget);

    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }
    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));
    servaddr.sin_family    = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

    pair_t *pair;
    pair = (pair_t *)buffer;
    int i=0;
    if ( bind(sockfd, (const struct sockaddr *)&servaddr,
            sizeof(servaddr)) < 0 )
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    while(i<200){
        recvPacks(sockfd);
        //sendPacks(sockfd, *pair);
        printf("d_1 = %f, d_2 = %f, d_3 = %f, d_4 = %f\n", pair->d_1, pair->d_2, pair->d_3, pair->d_4);
        i++;
    }
    close(sockfd);
    return a.exec();
    }
