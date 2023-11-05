#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// Standard C++ includes
#include <queue>
#include <bits/stdc++.h>

// QT C++ includes
#include <QDir>
#include <QFile>
#include <QString>
#include <QIODevice>
#include <QTcpSocket>
#include <QTcpServer>
#include <QFileDialog>
#include <QMainWindow>
#include <QTextStream>
#include <QMessageBox>

namespace Ui {
class MainWindow;
}
/*
class SocketInterface {

public:
    SocketInterface();

    // Common Socket Methods
    virtual void init();
    virtual void setUp();

    // Client virtual methods
    virtual void connected();
    virtual void disconnected();
    virtual void bytesWritten(qint64 bytes);
    virtual void readyRead();
    virtual void sendCMD();

    // Server Virtual Methods
    virtual void newConnection();


};
*/
// Main Window Class
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    void updateCMDWidget();
    void textToWidgets(QString text, int widget);
    ~MainWindow();

private slots:
    void on_action_Open_CMD_file_triggered();
    void startTaskOperations();
    void on_UpCmdButton_clicked();
    void on_DownCmdButton_clicked();
    void on_QuitCmdButton_clicked();
    bool isRealTimeMode();
    void printCommands();
    void on_StartCmdButton_clicked();
    void on_realTimeMode_clicked();

    // Client Socket
    void clientInit();
    void sendCMD();
    void connected();
    void disconnected();
    void bytesWritten(qint64 bytes);
    void readyRead();

    // Server Socket
    void serverInit();
    void newConnection();
    void onReadyRead();


private:
    Ui::MainWindow *ui;
    QTcpServer *serverSocket;
    QTcpSocket *clientSocketObj;
    //SocketInterface *telemServer;
    //SocketInterface *clientSocketCMD;
    bool isRealTime;
};
/*
// TCP-Client Class
class ClientSocket : public SocketInterface, public QObject{

    Q_OBJECT

public:
    explicit ClientSocket(QObject *parent = 0);
    //void setUp(MainWindow *inputWindowObj);
    void init();
    void sendCMD();

signals:
public slots:
    void connected();
    void disconnected();
    void bytesWritten(qint64 bytes);
    void readyRead();
    MainWindow getMainWindow();

private:
    //MainWindow *mainWindowObj;
    QTcpSocket *clientSocketObj;

};

// TCP-Server Class
class ServerSocket : public SocketInterface, public QObject {

    Q_OBJECT

public:

    explicit ServerSocket(QObject *parent = 0);
    //void setUp(MainWindow *inputWindowObj);
    void init();
    MainWindow getMainWindow();

signals:

public slots:
    void newConnection();

private:
    //MainWindow *mainWindowObj;
    QTcpServer *serverSocket;

};
*/
#endif // MAINWINDOW_H
