#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// Standard C++ includes
#include <queue>
#include <bits/stdc++.h>

// QT C++ includes
#include <QDir>
#include <QFile>
#include <QTimer>
#include <QString>
#include <QIODevice>
#include <QTcpSocket>
#include <QTcpServer>
#include <QFileDialog>
#include <QMainWindow>
#include <QTextStream>
#include <QMessageBox>

// QT second window header
#include "networkconfig.h"

namespace Ui {
class MainWindow;
}

// Main Window Class
class MainWindow : public QMainWindow {

    Q_OBJECT

public:
    // Helper Methods
    explicit MainWindow(QWidget *parent = 0);
    void updateCMDWidget();
    void textToWidgets(QString text, int widget);
    ~MainWindow();

    // Timer
    QTimer *timer;
    void timerHandler();

private slots:
    // Private UI methods
    void on_action_Open_CMD_file_triggered();
    void startTaskOperations();
    void on_UpCmdButton_clicked();
    void on_DownCmdButton_clicked();
    void on_QuitCmdButton_clicked();
    bool isRealTimeMode();
    void printCommands();
    void on_StartCmdButton_clicked();
    void on_realTimeMode_clicked();
    void on_action_Network_Config_triggered();

    // Client Socket Methods
    void clientInit();
    void sendCMD();
    void connected();
    void disconnected();
    void bytesWritten(qint64 bytes);
    void readyRead();

    // Server Socket Methods
    void serverInit();
    void newConnection();
    void onReadyRead();

private:
    Ui::MainWindow *ui;

    // Heartbeat Variables for RC connection
    int heartBeat = 0;
    int heartBeatFailures;

    // Controls Real-Time Mode
    bool isRealTime;

    // Internal objects that control
    // network communications with the RC car/robot
    NetworkConfig *networkOptions;
    QTcpServer *serverSocket;
    QTcpSocket *clientSocketObj;

};

#endif // MAINWINDOW_H
