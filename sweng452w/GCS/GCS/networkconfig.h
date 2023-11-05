#ifndef NETWORKCONFIG_H
#define NETWORKCONFIG_H

#include <QDialog>

namespace Ui {
class NetworkConfig;
}

class NetworkConfig : public QDialog
{
    Q_OBJECT

public:
    explicit NetworkConfig(QWidget *parent = 0);
    ~NetworkConfig();
    // SETTERS
    void setClientIP(QString inputClientAddr);
    void setClientPort(quint16 inputClientPort);
    void setServerPort(quint16 intputServerPort);
    void setRCIPUpdate(bool update);
    void setRCPortUpdate(bool update);
    void setServerPortUpdate(bool update);
    // GETTERS
    QString getClientIP();
    quint16 getClientPort();
    quint16 getServerPort();
    bool isRCUpdated();
    bool isRCPortUpdated();
    bool isServerPortUpdated();

private slots:
    void on_mountConfigButton_clicked();

    void on_ServerPort_valueChanged(int arg1);

    void on_RCPort_valueChanged(int arg1);

private:
    Ui::NetworkConfig *ui;
    // IP address variables
    QString clinetIPAdrr;
    quint16 clientPort;
    quint16 serverPort;
    bool isRCUpdatedVar;
    bool isServerPortVar;
    bool isRCPortVar;
};

#endif // NETWORKCONFIG_H
