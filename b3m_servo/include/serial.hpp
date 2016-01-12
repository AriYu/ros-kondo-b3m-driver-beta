#ifndef SERIAL_H_
#define SERIAL_H_

#include <iostream>
#include <string>
#include <exception>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <system_error>

class SerialPort{
public:
  SerialPort(std::string portName, int baudRate)
  {
    fd_ = open(portName.c_str(), O_RDWR | O_NOCTTY);
    if(fd_ < 0)
    {
exit(-1);
    }
    // 現在のシリアルポートの設定を退避させる
    tcgetattr(fd_, &oldtio_);
    // 新しいポートの設定の構造体をクリア
    memset(&newtio_, 0, sizeof(newtio_));
    
    // シリアル通信の設定
    newtio_.c_cc[VMIN] = 1;
	newtio_.c_cc[VTIME] = 0;
	newtio_.c_cflag = baudRate | CS8 | CREAD | CLOCAL;
	newtio_.c_iflag = IGNBRK | IGNPAR;
	newtio_.c_oflag = 0; // rawモード
	newtio_.c_lflag = 0; // 非カノニカル入力

    // ポートのクリア
    tcflush(fd_, TCIFLUSH);
    // ポートの設定を有効にする
    tcsetattr(fd_, TCSANOW, &newtio_);
  }
  ~SerialPort()
  {
    tcsetattr(fd_, TCSANOW, &oldtio_);
    close(fd_);
  }
public:
  int fd_;
 private:
  struct termios oldtio_, newtio_;
  
};

#endif
