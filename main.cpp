#include "mainwindow.h"
#include <QApplication>

#include "SerialPort.h"

int main(int argc, char *argv[])
{
    SerialPort * _serialPort = new SerialPort;
    
    _serialPort->SerialPortInit();

    return 0;
}
