#ifndef NETWORKCONFIG_H
#define NETWORKCONFIG_H

#include <QDialog>

namespace Ui {
class NetworkConfig;
}

// Allows user to change the IP address and Ports
// to connect to RC car
class NetworkConfig : public QDialog
{
    Q_OBJECT

public:
    // Constructor and Deconstructor
    explicit NetworkConfig(QWidget *parent = 0);
    ~NetworkConfig();

    // SETTER Methods
    void setClientIP(QString inputClientAddr);
    void setClientPort(quint16 inputClientPort);
    void setServerPort(quint16 intputServerPort);
    void setRCIPUpdate(bool update);
    void setRCPortUpdate(bool update);
    void setServerPortUpdate(bool update);

    // GETTER Methods
    QString getClientIP();
    quint16 getClientPort();
    quint16 getServerPort();
    bool isRCUpdated();
    bool isRCPortUpdated();
    bool isServerPortUpdated();

private slots:
    // UI methods
    void on_mountConfigButton_clicked();
    void on_ServerPort_valueChanged(int arg1);
    void on_RCPort_valueChanged(int arg1);

private:
    Ui::NetworkConfig *ui;

    // IP address variables
    QString clinetIPAdrr;
    quint16 clientPort;
    quint16 serverPort;

    // Checks if new values have been inputted
    bool isRCUpdatedVar;
    bool isServerPortVar;
    bool isRCPortVar;
};

#endif // NETWORKCONFIG_H
