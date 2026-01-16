#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QTcpServer>
#include <QTcpSocket>
#include <QDebug>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onReadyRead();

private:
    Ui::MainWindow *ui;
    QTcpServer *server;
    QTcpSocket *socket= nullptr;
    quint32 expectedSize = 0;  // 期望接收的数据大小
    QByteArray buffer;         // 数据缓冲区
};
#endif // MAINWINDOW_H
