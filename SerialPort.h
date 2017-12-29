#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <sys/stat.h>
#include <termios.h>
#include <sys/types.h>
#include <QString>
#include <QDebug>

#define WIFI_SERIAL_NUM_1   "/dev/extboard/tty_WiFi"
//#define WIFI_SERIAL_NUM_1   "/dev/ttyX_0"

class SerialPort
{
public:
    SerialPort();

    QString m_devname;
    int fd;
    int m_speed;
    int m_databits;
    int m_stopbits;
    int m_parity;
    bool bNextStep;
    bool bCardSerialPort;
    bool CardWorkStartFlag;                     //开始工作标识位

    SerialPort * _serialPort;          //串口类

    void SerialPortInit();
    void SerialPortScend();
    bool Open(const QString &dev);
    bool SetSpeed(const int baudrate);
    bool SetParity(int databits, int parity,int stopbits);
    bool SendDataAndCheck(const char *send,char *recv,int time);
};

#endif // SERIALPORT_H
