#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTcpSocket>
#include <QMessageBox>
#include <QTimer>
#include <opencv2/opencv.hpp>
#include <QBuffer>


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
    void sendFrame(const cv::Mat &frame);
    ~MainWindow();



private slots:
    void on_connectButton_clicked();

private:
    Ui::MainWindow *ui;
     QTcpSocket *socket;
     QTimer *timer = nullptr;
     cv::VideoCapture *cap = nullptr;  // 成员变量
};
#endif // MAINWINDOW_H
