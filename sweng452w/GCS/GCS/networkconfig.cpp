#include "networkconfig.h"
#include "ui_networkconfig.h"

// Constructor for new window
NetworkConfig::NetworkConfig(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::NetworkConfig)
{
    ui->setupUi(this);

    // Default IP and Port Addresses for RC
    //this->clinetIPAdrr = "10.0.0.223";
    this->clinetIPAdrr = "10.0.0.145";
    this->clientPort = 3390;

    // Default GCS server port
    this->serverPort = 9000;

    // Tracks if values have been updated in the window
    this->isRCUpdatedVar = false;
    this->isServerPortVar = false;
    this->isRCPortVar = false;

}

// Deconstuctor
NetworkConfig::~NetworkConfig()
{
    delete ui;
}

// SETTER Methods

// Sets the client RC IP address
void NetworkConfig::setClientIP(QString inputClientAddr){
    this->clinetIPAdrr = inputClientAddr;
}

// Sets the client RC Port address
void NetworkConfig::setClientPort(quint16 inputClientPort){
    this->clientPort = inputClientPort;
}

// Sets the GCS server port address
void NetworkConfig::setServerPort(quint16 intputServerPort){
    this->serverPort = intputServerPort;
}

// Used for tracking if the RC IP address has been updated from the defaults
void NetworkConfig::setRCIPUpdate(bool update){
    this->isRCUpdatedVar = update;
}

// Used for tracking if the RC Port address has been updated from the defaults
void NetworkConfig::setRCPortUpdate(bool update){
    this->isRCPortVar = update;
}

// Used for tracking if the GCS Port address has been updated from the defaults
void NetworkConfig::setServerPortUpdate(bool update){
    this->isServerPortVar = update;
}

// GETTER Methods

// Gets RC IP address
QString NetworkConfig::getClientIP(){
    return(this->clinetIPAdrr);
}

// Gets RC Port address
quint16 NetworkConfig::getClientPort(){
    return(this->clientPort);
}

// Gets GCS server port address
quint16 NetworkConfig::getServerPort(){
    return(this->serverPort);
}

// Checks if RC IP address has been change from defaults
bool NetworkConfig::isRCUpdated(){
    return(this->isRCUpdatedVar);
}

// Checks if RC Port address has been change from defaults
bool NetworkConfig::isRCPortUpdated(){
    return(this->isRCPortVar);
}

// Checks if GCS port address has been change from defaults
bool NetworkConfig::isServerPortUpdated(){
    return(this->isServerPortVar);
}

// UI-METHODS
// If there are any changes, mount them to the object variables
void NetworkConfig::on_mountConfigButton_clicked()
{
    // If input parameters are different than defaults,
    // then update - otherwise, just ignore.
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

// If the RC or GCS ports have been changed.
// Then the two functions change the boolen value (isChanged) to true.
// Used for mounting new port addresses in the method above.
void NetworkConfig::on_ServerPort_valueChanged(int arg1)
{
    this->setServerPortUpdate(true);
}

// Same as the function above.
void NetworkConfig::on_RCPort_valueChanged(int arg1)
{
    this->setRCPortUpdate(true);
}
