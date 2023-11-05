#include "mainwindow.h"
#include "ui_mainwindow.h"

// Heartbeat for RC connection
int heartBeat = 0;

// Command queue
std::queue<QString> commandQueue;

//SocketInterface::SocketInterface() {}

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow){
    ui->setupUi(this);

    networkOptions = new NetworkConfig(this);
    this->clientInit();
    this->serverInit();
    this->isRealTime = false;

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::textToWidgets(QString text, int widget){
    // 0: Command execution widget
    // 1: Telementry widget
    switch(widget){
        case(0):
            ui->CommandExeWidget->addItem(text);
            break;
        case(1):
            ui->TelemetryWidget->addItem(text);
            break;
    }
}

void MainWindow::on_action_Open_CMD_file_triggered()
{
    // File explorer wild-card filters
    QString filter = "All Files (*.*) ;; Command Files (*.CEF)";

    // Opens the file explorer at the home path
    QString filePath = QFileDialog::getOpenFileName(this, "Opening Command Execution File.", QDir::homePath(), filter);
    QFile cmdFile(filePath);

    // Error checking in command execution file
    if(!cmdFile.open(QFile::ReadOnly | QFile::Text)){
        QMessageBox::warning(this, "File I/O Error", "Error opening command file");
    }

    // Opening file stream for processing
    QTextStream fileIn(&cmdFile);
    QString fileText;

    // Streaming off file line-by-line and saving
    // the command to a queue.
    while(!fileIn.atEnd()){
        fileText = QString(fileIn.readLine());
        commandQueue.push(fileText);
    }

    if(!this->isRealTimeMode()){
        this->printCommands();
    } else {
        this->printCommands();
        this->startTaskOperations();
    }

}

void MainWindow::startTaskOperations(){

    // Goes through the queue and sends all commands
    while(!commandQueue.empty()) {
        //this->clientSocketCMD->sendCMD();
        this->sendCMD();
        this->printCommands();
    }

    // Clears out any remaining tasks
    // A clean-up operation
    ui->CommandExeWidget->clear();

}

void MainWindow::printCommands(){
    // Clears out old data
    ui->CommandExeWidget->clear();

    // Copies main priority queue
    std::queue<QString> cQCopy(commandQueue);

    // Scans through queue and prints the results to the display
    while (!cQCopy.empty()) {
        QString aCMD = cQCopy.front();
        // Add translation unit check here !!!
        ui->CommandExeWidget->addItem(aCMD);
        cQCopy.pop();
    }
}

void MainWindow::updateCMDWidget(){
    this->printCommands();
}

void MainWindow::on_UpCmdButton_clicked()
{
    commandQueue.push("UP");

    this->printCommands();

    if(this->isRealTimeMode()){
        this->on_StartCmdButton_clicked();
    }
}

void MainWindow::on_DownCmdButton_clicked()
{
    commandQueue.push("DOWN");

    this->printCommands();

    if(this->isRealTimeMode()){
        this->on_StartCmdButton_clicked();
    }
}

void MainWindow::on_QuitCmdButton_clicked()
{
    commandQueue.push("QUIT");

    this->printCommands();

    if(this->isRealTimeMode()){
        this->on_StartCmdButton_clicked();
    }
}

bool MainWindow::isRealTimeMode(){
    return(this->isRealTime);
}

void MainWindow::on_StartCmdButton_clicked(){
    if(!this->isRealTimeMode()){
        this->startTaskOperations();
    } else {
        //this->clientSocketCMD->sendCMD();
        this->sendCMD();
        this->printCommands();
    }
}

void MainWindow::on_realTimeMode_clicked()
{
    this->isRealTime = not this->isRealTime;


    if(this->isRealTime){
        ui->StartCmdButton->setEnabled(not this->isRealTime);
        ui->TelemetryWidget->addItem("Real-Time Mode enabled - enter commands for movement.");
    } else {
        //this->isRealTime = true;
        //ui->StartCmdButton->setEnabled(true);
        ui->StartCmdButton->setEnabled(not this->isRealTime);
        ui->TelemetryWidget->addItem("Real-Time Mode disabled - queue commands and press 'Queue Mode Start' when ready");
    }
}

void MainWindow::on_action_Network_Config_triggered()
{
    networkOptions = new NetworkConfig(this);
    networkOptions->show();
    networkOptions->exec();
    //networkOptions->cl

    int serverUpdate = 0;
    int RCUpdate = 0;

   if(networkOptions->isRCUpdated()){
        ui->TelemetryWidget->addItem("New RC IP Adrress assigned: " + networkOptions->getClientIP());
        RCUpdate++;
    }
   if(networkOptions->isRCPortUpdated()){
        ui->TelemetryWidget->addItem("New RC Port assigned: " +  QString::number(networkOptions->getClientPort()));
        RCUpdate++;
    }

   if(networkOptions->isServerPortUpdated()){
        ui->TelemetryWidget->addItem("New GCS Port assigned: " +  QString::number(networkOptions->getServerPort()));
        serverUpdate++;
    }

   if(RCUpdate > 0){
        this->clientInit();
        networkOptions->setRCIPUpdate(false);
        networkOptions->setRCPortUpdate(false);
   }

   if(serverUpdate > 0){
        this->serverInit();
        networkOptions->setServerPortUpdate(false);
   }

}

// QT-CLIENT SOCKET CLASS

//ClientSocket::ClientSocket(QObject *parent) : SocketInterface(), QObject(parent){}

//void ClientSocket::setUp(MainWindow *inputWindowObj){
//    this->mainWindowObj = inputWindowObj;
//    this->init();
//}

void MainWindow::clientInit(){
    // Creating TCP-Socket object
    clientSocketObj = new QTcpSocket(this);

    // Connecting abstract methods to socket parent class
    connect(clientSocketObj, SIGNAL(connected()), this, SLOT(connected()));
    connect(clientSocketObj, SIGNAL(disconnected()), this, SLOT(disconnected()));
    connect(clientSocketObj, SIGNAL(readyRead()), this, SLOT(readyRead()));
    connect(clientSocketObj, SIGNAL(bytesWritten(qint64)), this, SLOT(bytesWritten(qint64)));
}

void MainWindow::sendCMD(){
    // Output to the telemetry window
    //Ui ui = this->mainWindowObj->getUI();
    //clientSocketObj->flush();
    clientSocketObj->abort();

    ui->TelemetryWidget->addItem("Connecting to RC Car...");
    //this->mainWindowObj->textToWidgets("Connecting to RC Car...", 1);

    // Connecting to RC car server
    clientSocketObj->connectToHost(networkOptions->getClientIP(), networkOptions->getClientPort());

    // Error checking the connection to host
    if(!clientSocketObj->waitForDisconnected(100))
    {
        ui->TelemetryWidget->addItem("Error: " + clientSocketObj->errorString());
        //this->mainWindowObj->textToWidgets("Error: " + clientSocketObj->errorString(), 1);
    }

     clientSocketObj->flush();

}

void MainWindow::connected(){
    //Ui ui = this->mainWindowObj->getUI();

    ui->TelemetryWidget->addItem("Connected to RC car server...");
    //this->mainWindowObj->textToWidgets("Connected to RC car server...", 1);

    if(!commandQueue.empty()){
        QString aCMD = commandQueue.front();
        commandQueue.pop();
        this->clientSocketObj->write(aCMD.toUtf8().constData());

        //this->mainWindowObj->updateCMDWidget();
        this->printCommands();
    } else {
        ui->TelemetryWidget->addItem("No commands in command-queue, input movement or quit command(s)");
        //this->mainWindowObj->textToWidgets("No commands in command-queue, input movement or quit command(s)", 1);
    }

}

void MainWindow::disconnected()
{
    ui->TelemetryWidget->addItem("Disconnected from RC car server...");
    //this->mainWindowObj->textToWidgets("Disconnected from RC car server...", 1);
}

void MainWindow::bytesWritten(qint64 bytes)
{
    ui->TelemetryWidget->addItem("Command sent: " + bytes);
    //this->mainWindowObj->textToWidgets("Command sent: " + bytes, 1);
}

void MainWindow::readyRead()
{
   ui->TelemetryWidget->addItem("Received msg from RC: " + clientSocketObj->readAll());
   clientSocketObj->flush();
   //this->mainWindowObj->textToWidgets("Received msg from RC: " + clientSocketObj->readAll(), 1);
}

// QT-SERVER SOCKET CLASS

//ServerSocket::ServerSocket(QObject *parent) : SocketInterface(), QObject(parent) {}


//void ServerSocket::setUp(MainWindow *inputWindowObj){
//    this->mainWindowObj = inputWindowObj;
//    this->init();
//}

void MainWindow::serverInit(){
    serverSocket = new QTcpServer(this);

    connect(serverSocket, SIGNAL(newConnection()), this, SLOT(newConnection()));
    //serverSocket->isListening();

    if(!serverSocket->listen(QHostAddress::Any, networkOptions->getServerPort()))
    {
        ui->TelemetryWidget->addItem("Telemetry server could not start!");
        //this->mainWindowObj->textToWidgets("Telemetry server could not start!", 1);
    } else {
        ui->TelemetryWidget->addItem("Telemetry server started!");
        //this->mainWindowObj->textToWidgets("Telemetry server started!", 1);
    }
}

void MainWindow::newConnection()
{
    QTcpSocket *connSocket = serverSocket->nextPendingConnection();
    connect(connSocket, SIGNAL(readyRead()), this, SLOT(onReadyRead()));
    connSocket->flush();
    //socket->write("hello client\r\n");
    //connSocket->flush();

    ui->TelemetryWidget->addItem("Connected to RC Car - awaiting payload...");

    connSocket->waitForBytesWritten();
/*
    if(connSocket->bytesAvailable()){
        QString inputData = connSocket->readAll();
        if(inputData != "HeatBeat"){
            ui->TelemetryWidget->addItem(inputData);
            //this->mainWindowObj->textToWidgets(inputData, 1);
        } else {
            heartBeat++;
        }

    }
*/
    //connSocket->close();
    connSocket->flush();
}

void MainWindow::onReadyRead()
{
    QTcpSocket* sender = static_cast<QTcpSocket*>(QObject::sender());
    sender->flush();
    //QByteArray datas = sender->readAll();
    //if(sender->bytesAvailable()){
        QString inputData = sender->readAll();
        if(inputData != "HeartBeaT"){
            ui->TelemetryWidget->addItem(inputData);
            //this->mainWindowObj->textToWidgets(inputData, 1);
        } else {
            heartBeat++;
        }
    //}

    //sender->close();
    sender->flush();

    //ui->TelemetryWidget->addItem(sender->readAll());

    //for (QTcpSocket* socket : _sockets) {
    //    if (socket != sender)
    //        socket->write(QByteArray::fromStdString(sender->peerAddress().toString().toStdString() + ": " + datas.toStdString()));
    //}

     //sender->close();
}
