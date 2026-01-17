#ifndef MYTHREAD_H
#define MYTHREAD_H

#include <QObject>
#include <QThread>
#include <QTcpSocket>

class mythread : public QThread
{
    Q_OBJECT
public:
    explicit mythread(QTcpSocket *s);
    void run();

signals:
    void sendToWidget(QByteArray b);
public slots:
    void clientInfoSlots();

private:
    QTcpSocket *socket;
    quint32 expectedSize;  // 必须添加：每个线程独立的帧大小
    QByteArray buffer;     // 必须添加：每个线程独立的缓冲区
};

#endif // MYTHREAD_H
