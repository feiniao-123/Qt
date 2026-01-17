#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QImage>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    server = new QTcpServer(this);
    server->listen(QHostAddress::AnyIPv4,8000);

    connect(server, &QTcpServer::newConnection, [this](){
        QTcpSocket *socket = server->nextPendingConnection();
        socket->setParent(this);
        mythread *t = new mythread(socket);
        t->setParent(this);
        t->start();
        // 添加：线程结束时自动删除

        connect(t, &mythread::sendToWidget, this, &MainWindow::threadshowSlot);
    });
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::threadshowSlot(QByteArray s)
{
    // 简化的显示逻辑，只负责显示
    QImage image;
    if (image.loadFromData(s, "JPG")) {
        ui->cam->setPixmap(QPixmap::fromImage(image));
    }
}
