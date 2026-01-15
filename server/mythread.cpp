#include "mythread.h"

myThread::myThread(QTcpSocket *s)
    // 调用基类QThread的构造函数
{
    socket =s;
}

void myThread::run()
{

    connect(socket,&QTcpSocket::readyRead,this,&myThread::clientInfoSlots);
    // 这里实现线程的执行逻辑
    // run()函数会在调用start()时在新线程中执行
}
void myThread::clientInfoSlots(){
    //qDebug()<<socket->readAll();
    QByteArray ba=socket->readAll();
    emit sendToWidget(ba);



}
