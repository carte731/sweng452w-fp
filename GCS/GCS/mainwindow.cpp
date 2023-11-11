#include "mainwindow.h"
#include "ui_mainwindow.h"

// User inputed command queue
std::queue<QString> commandQueue;

// Main constructor
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow){
    ui->setupUi(this);

    // Configures network
    // Telemetry server and command client output
    networkOptions = new NetworkConfig(this);
    this->clientInit();
    this->serverInit();
    this->isRealTime = false;

    // Create Timer for heartbeat detection
    // If a heartbeat isn't receive after four minutes of intial connection.
    // A heartbeat error is printed to telemetry screen
    this->timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::timerHandler);
}

// Deconstructor
MainWindow::~MainWindow()
{
    delete ui;
}

// Used for testing
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

// UI method: Allows user to select a command execution file.
// This will load up the commands in the queue
void MainWindow::on_action_Open_CMD_file_triggered()
{
    // File explorer wild-card filters
    QString filter = "All Files (*.*) ;; Command Files (*.CEF)";

    // Opens the file explorer at the home path
    QString filePath = QFileDialog::getOpenFileName(this, "Opening Command Execution File.", QDir::homePath(), filter);

    // If the user didn't select anything,
    // don't do anything
    if(filePath.length() > 0){

        // File I/O
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

        // Reads the file and loads each command into
        // the queue. The RC car checks if the command is
        // invalid (translation method in RC program).
        while(!fileIn.atEnd()){
            fileText = QString(fileIn.readLine());
            commandQueue.push(fileText);
        }

        // If it's Real-Time mode then,
        // just execute the new inputted command.
        // Other-wise add the command to the queue.
        if(!this->isRealTimeMode()){
            this->printCommands();
        } else {
            this->printCommands();
            this->startTaskOperations();
        }
    } else {
        ui->TelemetryWidget->addItem("No Command file selected");
    }

}

// Executes the commands in the command queue
void MainWindow::startTaskOperations(){

    // Goes through the queue and sends all commands
    while(!commandQueue.empty()) {
        // Sends command to RC car
        this->sendCMD();

        // Refreshes the command queue widget window
        this->printCommands();
    }

    // Clears out any remaining tasks
    // A clean-up operation
    ui->CommandExeWidget->clear();

}

// Prints command in the command execution window
void MainWindow::printCommands(){
    // Clears out old data
    ui->CommandExeWidget->clear();

    // Copies main priority queue
    std::queue<QString> cQCopy(commandQueue);

    // Scans through queue and prints the results to the display
    while (!cQCopy.empty()) {
        QString aCMD = cQCopy.front();
        ui->CommandExeWidget->addItem(aCMD);
        cQCopy.pop();
    }
}

// Updates the command window - mostly used for testing
void MainWindow::updateCMDWidget(){
    this->printCommands();
}

// UI Method: If you press the up command,
// it adds the command to the command queue.
// If it's in real-time mode - it executes the commmand.
void MainWindow::on_UpCmdButton_clicked()
{
    // Adds command to command queue
    commandQueue.push("UP");

    // Updates the command execution window widget
    this->printCommands();

    // If it's real-time mode, execute the command now
    if(this->isRealTimeMode()){
        this->on_StartCmdButton_clicked();
    }
}


// UI Method: If you press the down command,
// it adds the command to the command queue.
// If it's in real-time mode - it executes the commmand.
void MainWindow::on_DownCmdButton_clicked()
{
    // Same as the up command

    commandQueue.push("DOWN");

    this->printCommands();

    if(this->isRealTimeMode()){
        this->on_StartCmdButton_clicked();
    }
}

// UI Method: If the quit command is entered.
// The RC car/robot will stop operations
void MainWindow::on_QuitCmdButton_clicked()
{
    // Same as the up/down commands

    commandQueue.push("QUIT");

    this->printCommands();

    if(this->isRealTimeMode()){
        this->on_StartCmdButton_clicked();
    }
}

// Getter method for checking if
// real-time mode is activated
bool MainWindow::isRealTimeMode(){
    return(this->isRealTime);
}

// UI Method: if real-time mode isn't actived
// Then this will execute all the commands in the
// command queue.
// This button is ghosted out in real-time mode.
// The else statement is there for a "just in case"
// situation.
void MainWindow::on_StartCmdButton_clicked(){
    if(!this->isRealTimeMode()){
        this->startTaskOperations();
    } else {
        this->sendCMD();
    }
}

// UI-Method: If real-time mode is actived, the start command is ghosted.
// Any commands inputted into the GCS will automatically be sent to the
// RC car/robot.
void MainWindow::on_realTimeMode_clicked()
{
    // This allows for a boolean toggle
    // It will flip to the opposite value
    // every-time the button is pressed.
    this->isRealTime = not this->isRealTime;

    // If the real-time mode is active, the start button
    // is ghosted (can't be pressed) and is unghosted if
    // real-time mode is not active.
    // Current mode is printed on the telemetry/status widget window
    if(this->isRealTime){
        ui->StartCmdButton->setEnabled(not this->isRealTime);
        ui->TelemetryWidget->addItem("Real-Time Mode enabled - enter commands for movement.");
    } else {
        ui->StartCmdButton->setEnabled(not this->isRealTime);
        ui->TelemetryWidget->addItem("Real-Time Mode disabled - queue commands and press 'Queue Mode Start' when ready");
    }
}

// UI-Method: If the network config is selected in the file dropdown menu.
// It allows the user to change the IP adress the GCS will connect to.
void MainWindow::on_action_Network_Config_triggered()
{
    // Create a new network object and assign it to the main objects
    // private variables.
    networkOptions = new NetworkConfig(this);
    // This makes the network-config window to open.
    networkOptions->show();
    networkOptions->exec();

    // Checks track of any change.
    // If the user doesn't change anything
    // then it doesn't update the main objects client or server objects
    int serverUpdate = 0;
    int RCUpdate = 0;

    // Checks if the RC car/robot IP address has changed
    if(networkOptions->isRCUpdated()){
        ui->TelemetryWidget->addItem("New RC IP Address assigned: " + networkOptions->getClientIP());
        RCUpdate++;
    }

   // Checks if the RC car/robot Port address has changed
   if(networkOptions->isRCPortUpdated()){
        ui->TelemetryWidget->addItem("New RC Port assigned: " +  QString::number(networkOptions->getClientPort()));
        RCUpdate++;
    }

   // Checks if the GCS server port has changed
   if(networkOptions->isServerPortUpdated()){
        ui->TelemetryWidget->addItem("New GCS Port assigned: " +  QString::number(networkOptions->getServerPort()));
        serverUpdate++;
    }

   // If either elements of the RC car/robot IP or Port
   // have been altered, then reinitialized the client socket object
   if(RCUpdate > 0){
        this->clientInit();
        networkOptions->setRCIPUpdate(false);
        networkOptions->setRCPortUpdate(false);
   }

   // If the GCS server port has changed, then reinitialized the server object
   if(serverUpdate > 0){
        this->serverInit();
        networkOptions->setServerPortUpdate(false);
   }

}

// QT-CLIENT SOCKET (Used for sending commands to the RC Car/Robot)

// Initializes the object, connect and abstract methods for QT TCP socket object
void MainWindow::clientInit(){
    // Creating QT-TCP Socket object
    clientSocketObj = new QTcpSocket(this);

    // Connecting abstract methods to socket parent class
    connect(clientSocketObj, SIGNAL(connected()), this, SLOT(connected()));
    connect(clientSocketObj, SIGNAL(disconnected()), this, SLOT(disconnected()));
    connect(clientSocketObj, SIGNAL(readyRead()), this, SLOT(readyRead()));
    connect(clientSocketObj, SIGNAL(bytesWritten(qint64)), this, SLOT(bytesWritten(qint64)));
}

// Sends input commands to the RC Car/Robot once connected
void MainWindow::sendCMD(){
    // Kills old connection
    clientSocketObj->abort();

    // Info for telemetry/status window
    ui->TelemetryWidget->addItem("Connecting to RC Car...");

    // Connecting to RC car server
    clientSocketObj->connectToHost(networkOptions->getClientIP(), networkOptions->getClientPort());

    // Error checking the connection to host
    if(!clientSocketObj->waitForDisconnected(100)){
        ui->TelemetryWidget->addItem("Command TimeOut: " + clientSocketObj->errorString());
    }

    // Flush the client buffer
    clientSocketObj->flush();

}

// If a connection is made to the RC car/robot
// then this method is activated automatically by QT
void MainWindow::connected(){
    ui->TelemetryWidget->addItem("Connected to RC car server...");

    // Empties out command queue and
    // sends commands to RC
    if(!commandQueue.empty()){
        QString aCMD = commandQueue.front();
        commandQueue.pop();
        this->clientSocketObj->write(aCMD.toUtf8().constData());

        this->printCommands();
    } else {
        ui->TelemetryWidget->addItem("No commands in command-queue, input movement or quit command(s)");
    }

}

// Once commands have been sent, the connection is closed for the next
// round of commands to be sent.
void MainWindow::disconnected()
{
    ui->TelemetryWidget->addItem("End of command exection frame for RC car server...");
}

// Prints the out bound commands
void MainWindow::bytesWritten(qint64 bytes)
{
    ui->TelemetryWidget->addItem("Command sent: " + bytes);
}

// Prints out any confirmation message from RC
void MainWindow::readyRead()
{
   ui->TelemetryWidget->addItem("Received msg from RC: " + clientSocketObj->readAll());
   clientSocketObj->flush();
}

// QT-SERVER SOCKET

// Initializes the GCS server for any inbound telemetry data
void MainWindow::serverInit(){
    // Creates instance of QT-TCP server socket object for GCS
    serverSocket = new QTcpServer(this);

    // Connecting abstract methods to socket parent class
    connect(serverSocket, SIGNAL(newConnection()), this, SLOT(newConnection()));

    // Listens for any inbound data from external sources
    if(!serverSocket->listen(QHostAddress::Any, networkOptions->getServerPort()))
    {
        ui->TelemetryWidget->addItem("Telemetry server could not start!");
    } else {
        ui->TelemetryWidget->addItem("Telemetry server started!");
    }
}

// If an external source connects to the GCS server,
// the connection is captured and inbound data set-up here
void MainWindow::newConnection()
{
    // Assigns inbound connection to local temp socket object
    QTcpSocket *connSocket = serverSocket->nextPendingConnection();
    connect(connSocket, SIGNAL(readyRead()), this, SLOT(onReadyRead()));
    connSocket->flush();

    ui->TelemetryWidget->addItem("Connected to RC Car - awaiting payload...");

    // Waits for inbound data from socket object
    connSocket->waitForBytesWritten();

    // Flush the data every-time it's received
    connSocket->flush();
}

// Processes data from inbound data
void MainWindow::onReadyRead()
{
    // Bind data to socket
    QTcpSocket* sender = static_cast<QTcpSocket*>(QObject::sender());

    sender->flush();
    // If bytes are available, process them
    if(sender->bytesAvailable()){
        // Read all the data, it's not a heartbeat
        // print it.
        QString inputData = sender->readAll();
        if(inputData != "HeartBeaT"){
            ui->TelemetryWidget->addItem(inputData);
        } else {
            // Tracks heartbeats, allows one minute between heartbeats
            // otherwise the handler takes over
            this->heartBeat++;
            this->heartBeatFailures = 0;
            // Starts a 1 minute timer - if the timer reaches zero then
            // a heartbeat failure is logged.
            // After four failues, the connection is considered dead
            this->timer->start(60000);
        }

    }

    // Flush the buffer after every telemetry input from RC
    sender->flush();
}

// The handler for the QT-Timer, used with heartbeats
void MainWindow::timerHandler(){
    // If the timer fails 4 times in a row, the connection is considered dead
    this->heartBeatFailures++;
    // four consecutive failues, this is printed.
    if(this->heartBeatFailures >= 4){
        ui->TelemetryWidget->addItem("HEARTBEAT FAILURE...");
    }
}
