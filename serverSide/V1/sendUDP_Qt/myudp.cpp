#include "myudp.h"


MyUDP::MyUDP(QObject *parent) :QObject(parent)

{
    socket = new QUdpSocket(this);
    socket->bind(QHostAddress("192.168.1.5"),50000);
    connect(socket,SIGNAL(readyRead()),this,SLOT(readReady()));
}


void MyUDP::WriteData()

{
    QByteArray msg;
    msg.append("Hello!!!");
    socket->writeDatagram(msg,QHostAddress("192.168.1.5"),50000);
}


void MyUDP::readReady()

{
    QByteArray buffer;
    buffer.resize(socket->pendingDatagramSize());
    QHostAddress sender;
    quint16 port;
    socket->readDatagram(buffer.data(),buffer.size(),&sender,&port);
    qDebug()<<"Message From: " << sender.toString();
    qDebug()<<"With Port Number " << port;
    qDebug()<<"Message: " << buffer;

}
