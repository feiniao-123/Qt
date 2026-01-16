#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    server = new QTcpServer(this);
    server->listen(QHostAddress::AnyIPv4,8000);

    connect(server, &QTcpServer::newConnection, [this](){
        socket = server->nextPendingConnection();
         socket->setParent(this);
        connect(socket, &QTcpSocket::readyRead, this, &MainWindow::onReadyRead);
    });
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onReadyRead()
{
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

            QImage image;
            if (image.loadFromData(imageData, "JPG")) {
                ui->cam->setPixmap(QPixmap::fromImage(image));
            }

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
