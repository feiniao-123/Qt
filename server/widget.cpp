#include "widget.h"
#include "ui_widget.h"

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    server = new QTcpServer;


    //服务器的ip地址和端口号，随时准备收到连接
    server->listen(QHostAddress::AnyIPv4,8000);


    //客户端发起连接之后，我们的服务端接受到这个信号，有信号就可以回调，我们就可以使用回调
    connect(server,&QTcpServer::newConnection,this,&Widget::newClientHandler);

    //客户端发送信息之后，我们的服务端传过来信息
//    connect(socket,&QTcpSocket::readyRead,this,&Widget::clientInfoSlot);
    //不可以再这里写这个回调，因为回调收到消息的一方是socket（在客户端中，我们把要改变的消息通过socket写了下来），但是这里之前还没有
    //创建过socket，之所以上面的回调可以用server是因为我们把server声明在最前面了
}

Widget::~Widget()
{
    delete ui;
}

void Widget::newClientHandler(){
    QTcpSocket *socket =server->nextPendingConnection();
    ui->iplineEdit->setText(socket->peerAddress().toString());
    ui->portlineEdit->setText(QString::number(socket->peerPort()));

//    connect(socket,&QTcpSocket::readyRead,this,&Widget::clientInfoSlot);
    //写到这里
    myThread *t =new myThread(socket);
    t->start();   //开始线程
    connect(t,&myThread::sendToWidget,this,&Widget::threadshowSlot);

}


//void Widget::clientInfoSlot(){
    //那我们这里似乎也不能用socket了，因为刚才的socket是局部变量
    //那我们应该把它设置为全局变量吗？这样多个信息传来就会乱套
    //方法：使用sender（）——获取信号的发出者；
//    QTcpSocket *s =(QTcpSocket *) sender();
//    ui->lineEdit->setText(QString(s->readAll()));

//}

void Widget::threadshowSlot(QByteArray b){
    ui->lineEdit->setText(QString(b));
}
