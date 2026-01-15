#include "widget.h"
#include "ui_widget.h"

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    socket =new QTcpSocket;  //创建socket对象，这个对象是一个指针，指向QTcpSocket这个实体

    //判断服务器连接成功，如果成功，服务器对象会发出信号
    connect(socket,&QTcpSocket::connected,[this](){
        QMessageBox::information(this,"连接提示","连接服务器成功");
        this->hide();
        Chat *c =new Chat(socket);
        c->show();
    });
    //判断服务器连接失败
    connect(socket,&QTcpSocket::disconnected,[this](){
        QMessageBox::warning(this,"连接提示","连接服务器失败");
    });
}

Widget::~Widget()
{
    delete ui;
}


void Widget::on_cancelButton_clicked()
{
    this->close();
}

void Widget::on_connectButton_clicked()
{
    //首先获取ip地址和端口号
    QString ip = ui->iplineEdit->text();
    QString port =ui->portlineEdit->text();

    //连接服务器
    socket->connectToHost(QHostAddress(ip),port.toShort());


}
