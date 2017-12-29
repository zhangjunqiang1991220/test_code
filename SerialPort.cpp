#include "SerialPort.h"

int speed_arr[] = { B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200};
int name_arr[] = { 115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200};

SerialPort::SerialPort()
{
    qDebug()<<"......................SerialPort";
    bNextStep = false;

//    m_devname.clear();
    fd = -1;
//    m_speed = 115200;
//    m_databits = 8;
//    m_parity = 'n';
//    m_stopbits = 1;
}

void SerialPort::SerialPortInit()
{
    qDebug()<<".............................SerialPortInit";
    int ret = 0,WaitTime = 0;
    QString qSendData,qCompareData;

    qSendData.clear();
    qCompareData.clear();
    ret = Open (WIFI_SERIAL_NUM_1);

    SetParity (8,'N',1);
    SetSpeed (57600);

    SendDataAndCheck("AT+VER\r\n","AT+VER",500);

    sleep(3);
    if(ret == true)
    {
          CardWorkStartFlag = TRUE;
          bNextStep = SendDataAndCheck("+++","a",500);
          if(bNextStep == true)
          {
              bNextStep = false;
              bNextStep = SendDataAndCheck("a","+ok",500);

              if(bNextStep == true)
              {
                  bNextStep = false;
                  bNextStep = SendDataAndCheck("AT+VER\r\n","AT+VER",500);

//                  usleep(500000);
//                  SendDataAndCheck("AT+RELD\r\n","AT+RELD",5000);
                  if(bNextStep == true)
                  {
                      bNextStep = false;
                      bNextStep = SendDataAndCheck("AT+WMODE=AP\r\n","AT+WMODE=AP",1000);

                      if(bNextStep == true)
                      {
                          bNextStep = false;
                          bNextStep = SendDataAndCheck("AT+WAP=11BGN,TGOOD_test_server,Auto\r\n","AT+WAP=11BGN,TGOOD_test_server,Auto",1000);

                          if(bNextStep == true)
                          {
                              bNextStep = false;
                              bNextStep = SendDataAndCheck("AT+WAKEY=WPA2PSK,AES,TGOOD300001\r\n","AT+WAKEY=WPA2PSK,AES,TGOOD300001",1000);

//                              SendDataAndCheck("AT+NETP=TCP,Server,8899,10.10.100.100\r\n","AT+NETP",1500);
//                              SendDataAndCheck("AT+NETP\r\n","AT+NETP",1500);

                              SendDataAndCheck("AT+LANN=10.10.100.254,255.0.0.0\r\n","AT+LANN",1500);

                              SendDataAndCheck("AT+LANN\r\n","AT+LANN",1500);

                              SendDataAndCheck("AT+WANN\r\n","AT+WANN",1500);

//                              SendDataAndCheck("AT+EPHY\r\n","AT+EPHY",1500);

                              if(bNextStep == true)
                              {
                                  bNextStep = false;
                                  bNextStep = SendDataAndCheck("AT+Z\r\n","AT+Z",(1000 * 5));

                                  sleep(2);
                                  printf("============================");
                                  SerialPortScend();

                                  return;
                              }
                          }
                      }
                  }
              }
//              bNextStep = SendDataAndCheck("AT+ENTM\r\n","AT+ENTM",500);
          }
    }
}

void SerialPort::SerialPortScend()
{
    bNextStep = false;

    SendDataAndCheck("AT+VER\r\n","AT+VER",(1000 * 3));

    bNextStep = SendDataAndCheck("+++","a",500);
    if(bNextStep == true)
    {
        bNextStep = false;
        bNextStep = SendDataAndCheck("a","+ok",500);
        if(bNextStep == true)
        {
            bNextStep = false;
            bNextStep = SendDataAndCheck("AT+VER\r\n","AT+VER",500);

            if(bNextStep == true)
            {
                bNextStep = false;
                bNextStep = SendDataAndCheck("AT+WMODE=AP\r\n","AT+WMODE=AP",1000);

                if(bNextStep == true)
                {
                    bNextStep = false;
                    bNextStep = SendDataAndCheck("AT+WMODE\r\n","AT+WMODE",1000);

                    SendDataAndCheck("AT+EPHY\r\n","AT+EPHY",1500);

                    if(bNextStep == true)
                    {
                        bNextStep = false;
                        bNextStep = SendDataAndCheck("AT+ENTM\r\n","AT+ENTM",500);

                        return;
                    }
                }
            }
        }
    }
    bNextStep = SendDataAndCheck("AT+ENTM\r\n","AT+ENTM",500);
}

bool SerialPort::Open(const QString &name)
{
    struct termios tio;
    qDebug()<<"To open "<<name;
    if ((fd = open(name.toAscii().data(),O_RDWR|O_NOCTTY|O_NONBLOCK)) > 0)
    {
        qDebug()<<"----------------------------opened";
        struct termios init_settings,new_settings;
        tcgetattr(fd,&init_settings);
        new_settings = init_settings;
        new_settings.c_lflag &= ~(ICANON|ISIG);
        new_settings.c_lflag &= ~ECHO;
        new_settings.c_iflag &= ~ICRNL;
        new_settings.c_iflag &= ~INLCR;
        new_settings.c_iflag &= ~IXON;
        new_settings.c_iflag &= ~IXANY;
        new_settings.c_iflag &= ~IXOFF;
        new_settings.c_iflag &= ~IGNBRK;
        new_settings.c_oflag &= ~ONLCR;
        new_settings.c_oflag &= ~OCRNL;
        if (tcsetattr(fd, TCSANOW, &new_settings) != 0)
        {
            perror("SetupSerial faild!");
        }
        else
        {
            qDebug() << QString ("Open %1 success! fd=%2").arg(name).arg(fd);
            m_devname = name;
            return TRUE;
        }
    }
    qDebug()<<"open faile";
    qDebug() << QString ("Open %1 faild!").arg(name);
    return FALSE;
}

bool SerialPort::SetParity(int databits,int parity,int stopbits)
{
    struct termios options;
    if (tcgetattr(fd, &options) != 0)
    {
        perror("SetupSerial 1");
        return (FALSE);
    }
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            fprintf(stderr, "Unsupported data size\n");
            return (FALSE);
    }
    switch (parity)
    {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB;
            options.c_iflag &= ~INPCK;
            break;
        case 'o':
        case 'O':
            options.c_cflag |= (PARODD | PARENB);
            options.c_iflag |= INPCK;
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK;
            break;
        case 'S':
        case 's':
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            fprintf(stderr, "Unsupported parity\n");
            return (FALSE);
    }

    switch (stopbits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            fprintf(stderr, "Unsupported stop bits\n");
            return (FALSE);
    }
    /* Set input parity option */
    if (parity != 'n')
    {
        options.c_iflag |= INPCK;
    }
    tcflush(fd, TCIFLUSH);

//  options.c_iflag &= ~(IXON | IXOFF | IXANY);

    options.c_cc[VTIME] = 1; // 1 seconds
    options.c_cc[VMIN] = 0;
    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
        perror("SetupSerial faild!");
        return (FALSE);
    }
    m_stopbits = stopbits;
    m_databits = databits;
    m_parity   = parity;
    return (TRUE);
}

bool SerialPort::SetSpeed(const int speed)
{
    unsigned int i;
    int status;
    struct termios Opt;
    tcgetattr(fd, &Opt);
    for (i= 0; i < sizeof(speed_arr) / sizeof(int); i++)
    {
        if (speed == name_arr[i])
        {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);
            status = tcsetattr(fd, TCSANOW, &Opt);
            if (status != 0)
            {
                perror("tcsetattr fd1");
                return FALSE;
            }
            tcflush(fd, TCIOFLUSH);
        }
    }
    m_speed = speed;
    return TRUE;
}

bool SerialPort::SendDataAndCheck(const char *send,char *compare,int time)
{
    int nbytes = -1;
    unsigned char m_RecvBuf[128];

    memset(m_RecvBuf,0x00,sizeof(m_RecvBuf));
    nbytes = write(fd,send,strlen(send));
    printf("[send] = %s\n",send);


    usleep(time * 1000);
    nbytes = read(fd,m_RecvBuf, 100);
    printf("[recv] = %s\n",m_RecvBuf);

    if(strncmp(compare,(const char *)m_RecvBuf,strlen(compare)) == 0)
    {
        return true;
    }

    return false;
}
