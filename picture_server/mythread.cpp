#include "mythread.h"
#include <QDataStream>



mythread::mythread(QTcpSocket *s)
{
    socket = s;
}

void mythread::run()
{
    connect(socket, &QTcpSocket::readyRead, this, &mythread::clientInfoSlots);
    exec();
    // 线程退出时清理socket
    socket->deleteLater();
}

void mythread::clientInfoSlots()
{
    // 从主线程移动过来的粘包处理逻辑
    buffer.append(socket->readAll());

    while (true) {
        if (expectedSize == 0) {
            if (buffer.size() < (int)sizeof(quint32)) {
                return;
            }

            QDataStream stream(buffer);
            stream.setVersion(QDataStream::Qt_5_15);
            stream >> expectedSize;
            buffer = buffer.mid(sizeof(quint32));
        }

        if (expectedSize > 0 && buffer.size() >= expectedSize) {
            QByteArray imageData = buffer.left(expectedSize);
            buffer = buffer.mid(expectedSize);

            // 发送完整帧给主线程显示
            emit sendToWidget(imageData);

            expectedSize = 0;

            if (buffer.size() > 0) {
                continue;
            } else {
                break;
            }
        } else {
            break;
        }
    }
}
