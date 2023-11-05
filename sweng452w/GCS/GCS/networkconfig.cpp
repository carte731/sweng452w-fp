#include "networkconfig.h"
#include "ui_networkconfig.h"

NetworkConfig::NetworkConfig(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::NetworkConfig)
{
    ui->setupUi(this);

    // Default IP Address
    this->clinetIPAdrr = "10.0.0.223";
    this->clientPort = 3390;
    this->serverPort = 9050;
    this->isRCUpdatedVar = false;
    this->isServerPortVar = false;
    this->isRCPortVar = false;

}

NetworkConfig::~NetworkConfig()
{
    delete ui;
}

// SETTERS
void NetworkConfig::setClientIP(QString inputClientAddr){
    this->clinetIPAdrr = inputClientAddr;
}

void NetworkConfig::setClientPort(quint16 inputClientPort){
    this->clientPort = inputClientPort;
}

void NetworkConfig::setServerPort(quint16 intputServerPort){
    this->serverPort = intputServerPort;
}

void NetworkConfig::setRCIPUpdate(bool update){
    this->isRCUpdatedVar = update;
}

void NetworkConfig::setRCPortUpdate(bool update){
    this->isRCPortVar = update;
}

void NetworkConfig::setServerPortUpdate(bool update){
    this->isServerPortVar = update;
}

// GETTERS
QString NetworkConfig::getClientIP(){
    return(this->clinetIPAdrr);
}

quint16 NetworkConfig::getClientPort(){
    return(this->clientPort);
}

quint16 NetworkConfig::getServerPort(){
    return(this->serverPort);
}

bool NetworkConfig::isRCUpdated(){
    return(this->isRCUpdatedVar);
}

bool NetworkConfig::isRCPortUpdated(){
    return(this->isRCPortVar);
}

bool NetworkConfig::isServerPortUpdated(){
    return(this->isServerPortVar);
}

// METHODS
void NetworkConfig::on_mountConfigButton_clicked()
{
    if(!ui->RCIPAddr->text().isEmpty()){
        this->setClientIP(ui->RCIPAddr->text());
        this->setRCIPUpdate(true);
    }

    if(this->isRCPortUpdated()){
        this->setClientPort(ui->RCPort->value());
    }

    if(this->isServerPortUpdated()){
        this->setServerPort(ui->ServerPort->value());
    }

}

void NetworkConfig::on_ServerPort_valueChanged(int arg1)
{
    this->setServerPortUpdate(true);
}

void NetworkConfig::on_RCPort_valueChanged(int arg1)
{
    this->setRCPortUpdate(true);
}
