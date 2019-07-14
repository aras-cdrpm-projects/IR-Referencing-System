#include <QtCore/QCoreApplication>
#include "myudp.h"


int main(int argc, char *argv[])

{
    QCoreApplication a(argc, argv);
    MyUDP client;
    client.WriteData();

    return a.exec();
}
