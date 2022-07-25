#ifndef RS485_DRIVER_H
#define RS485_DRIVER_H
/*************************************************************************
 * @ Brief: 串口通讯程序封装
 * @ Author: Lazyshawn
 * @ Email: hitszshaohu@gmail.com
 * @ Data: 220720
 * @ Reference:
 * 串口通信示例:
 * zhou-yuxin.github.io/articles/2016/Linux%20串口操作（C++实现同步发、异步收的串口类）/index.html
 * Linux 串口通信介绍:
 * https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
 * 串口阻塞模式介绍:
 * https://stackoverflow.com/a/26006680
 * CRC 校验: http://www.ip33.com/crc.html
 * 进制转换计算器: https://lostphp.com/hexconvert/
*************************************************************************/

#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <chrono>
/* *** 串口通讯头文件 *** */
// C library headers
#include <stdio.h>
#include <string.h>  // bzero()
// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // Unic standard func: write(), read(), close()

// 十六进制与十进制转换
template <typename T>
void hex2dec(uint8_t *hex, T &dec, int hexLen = 4) {
  if (hexLen != 4) {
    std::cout << "hex2dec need 4 byte." << std::endl;
    throw hexLen;
  }
  // 反转字节序
  uint8_t endianInv[hexLen];
  for (int i=0; i<hexLen; ++i) {
    endianInv[i] = hex[hexLen-1-i];
  }
  dec = *(T*)endianInv;
}
// Note: 负数使用浮点数操作
int hex2int(uint8_t *hexData, int hexLen = 4);
float hex2float(uint8_t *hexData, int hexLen = 4);

class SerialPort {
private:
  // 打开串口获得的文件描述符
  int nFd;
  // 串口名称: /dev/ttyUSB0
  std::string ttyName;
  // Linux串口配置的结构体
  struct termios ttyOpt;
  // 传输延时(s): 串口阻塞时的等待时间
  float msDelay = 500;

public:
  SerialPort(std::string ttyName_, int baud, int dataBits, int parMode,
             int stopBits, float sDelay_ = 500);
  ~SerialPort();
  // 串口初始化
  // 校验模式(NOEMS)
  int port_init(std::string ttyName, int baud, int dataBits, int parMode,
                int stopBits);
  // 串口通讯
  int write_port(uint8_t* tx, int txLen);
  int read_port(uint8_t* rx, int rxLen);

  int tranBaud(int baud);
  int tranDataB(int dataBits);
};

class RS485Device {
private:
  // 通信串口
  SerialPort *serial;
  // 从机地址
  uint8_t slaveID = 0x01;
  // 传输数据
  uint8_t rxData[1024], txData[1024];
  // 功能码
  static uint8_t rCoil, rMulReg, wMulReg;

public:
  RS485Device(uint8_t ID = 0x01);
  RS485Device(std::string ttyName, int baud, int dataBits, int parMode,
              int stopBits, uint8_t ID = 0x01);
  ~RS485Device();

  /*************************************************************************
   * @ Brief: 打开通信串口
   * @ Param: 设备名称、从机ID、波特率、数据位、校验模式、停止位
   * @ Return: 函数执行结果
  *************************************************************************/
  int connect(std::string ttyName, int baud, int dataBits, int parMode,
              int stopBits);
  std::vector<uint8_t> readMulReg(uint16_t addr, uint16_t num);
  /*************************************************************************
   * @ Brief: 写连续寄存器
   * @ Param: 起始寄存器地址、寄存器个数、待写入数据
   * @ Return: 从机返回的数据
  *************************************************************************/
  std::vector<uint8_t> writeMulReg(uint16_t addr, std::vector<uint8_t> wData);
  // CRC-16 校验
  uint16_t get_crc16(uint8_t *data, int len);
};

#endif
