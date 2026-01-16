#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    socket =new QTcpSocket(this);
    connect(socket,&QTcpSocket::connected,[this](){
       QMessageBox::information(this,"连接提示","连接服务器成功");
        if (cap == nullptr) {
            cap = new cv::VideoCapture(0);

            if (cap->isOpened()) {
                cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

                timer = new QTimer(this);
                connect(timer, &QTimer::timeout, [this]() {  // 只捕获this
                    cv::Mat frame;
                    *cap >> frame;
                    if (!frame.empty()) {
                        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
                        ui->cam->setPixmap(QPixmap::fromImage(
                            QImage(frame.data, frame.cols, frame.rows,
                                   frame.step, QImage::Format_RGB888)));

                        sendFrame(frame);
                    }
                });
                timer->start(30);

            } else {
                delete cap;
                cap = nullptr;
            }
        }
    });
    //判断服务器连接失败
    connect(socket,&QTcpSocket::disconnected,[this](){
    QMessageBox::warning(this,"连接提示","连接服务器失败");
    });
}

MainWindow::~MainWindow()
{
    if (cap) {
        cap->release();
        delete cap;
    }
    if (timer) {
        timer->stop();
        delete timer;
    }
    delete ui;

}



void MainWindow::on_connectButton_clicked()
{
    QString ip =ui->ipEdit->text();
    QString port =ui->portEdit->text();
    socket->connectToHost(QHostAddress(ip),port.toShort());
}


void MainWindow::sendFrame(const cv::Mat &frame)
{// 客户端 sendFrame 函数修改：

        if (!socket || !socket->isOpen()) return;

        // 1. 转换为QImage
        QImage image(frame.data, frame.cols, frame.rows,
                     frame.step, QImage::Format_RGB888);

        // 2. 保存为JPEG字节流
        QByteArray jpegData;
        QBuffer buffer(&jpegData);
        buffer.open(QIODevice::WriteOnly);
        image.save(&buffer, "JPG");

        // 3. 发送数据：先发送4字节长度，再发送JPEG数据
        QByteArray block;
        QDataStream stream(&block, QIODevice::WriteOnly);
        stream << (quint32)jpegData.size();  // 只写大小
        block.append(jpegData);              // 追加纯JPEG数据

        socket->write(block);

}
