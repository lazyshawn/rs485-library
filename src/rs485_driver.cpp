#include "rs485_driver.h"

int hex2int(uint8_t *hexData, int hexLen) {
  int decData = 0, byteVal;
  for (int i=0; i<hexLen; ++i) {
    byteVal = hexData[hexLen-1 -i] << (8*i);
    decData += byteVal;
  }
  return decData;
}

// Note: 负数使用浮点数操作
float hex2float(uint8_t *hexData, int hexLen) {
  if (hexLen != 4) {
    std::cout << "hex2float need 4 byte." << std::endl;
    throw hexLen;
  }
  uint8_t endianInv[hexLen];
  for (int i=0; i<hexLen; ++i) {
    endianInv[i] = hexData[hexLen-1-i];
  }
  float floatData = *(float*)&endianInv;
  return floatData;
}

SerialPort::SerialPort(std::string ttyName_, int baud, int dataBits,
                       int parMode, int stopBits, float sDelay_)
    : ttyName(ttyName_), msDelay(sDelay_) {
  port_init(ttyName, baud, dataBits, parMode, stopBits);
}

SerialPort::~SerialPort() {
  if (nFd >= 0) {
    ::close(nFd);
    nFd = -1;
  }
}

// 串口初始化
int SerialPort::port_init(std::string ttyName, int baud, int dataBits,
                          int parMode, int stopBits) {
  /* *** 打开串口 *** */
  // TODO: 遍历串口，直到打开
  // O_RDWR: 读写方式打开; O_NOCTTY: 串口数据不当成Linux命令; O_NDELAY: 非阻塞
  nFd = ::open(ttyName.c_str(), O_RDWR|O_NOCTTY|O_NDELAY);
  if (nFd < 0) {
    printf("Error %i from open: %s\n", errno, strerror(errno));
  }
  // 复制原先串口的配置
  if(tcgetattr(nFd, &ttyOpt) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  /* *** 串口参数设置 *** */
  // bzero(&serialSetting, sizeof(serialSetting));
  // 设置终端为原始模式，该模式下全部的输入数据以字节为单位被处理
  cfmakeraw(&ttyOpt);
  // 设置输入输出速率(波特率)
  int baudT = tranBaud(baud);
  cfsetispeed(&ttyOpt,baudT);
  cfsetospeed(&ttyOpt,baudT);
  // 设置数据位
  int csize = tranDataB(dataBits);
  // 用数据位掩码清空数据位设置
  ttyOpt.c_cflag &= ~CSIZE;
  ttyOpt.c_cflag |= csize;
  // 设置校验位
  if (parMode == 0) {
    // 无校验位
    ttyOpt.c_cflag &= ~PARENB;
  } else if (parMode == PARODD || parMode == ~PARODD){
    // 奇校验/偶校验
    ttyOpt.c_cflag |= PARENB;
    ttyOpt.c_cflag |= parMode;
  } else {
    std::cout << "Wrong party mode: 0/PARODD/~PARODD" << std::endl;
    throw ttyOpt.c_cflag;
  }
  // 设置停止位
  if (stopBits == 2) {
    ttyOpt.c_cflag |= CSTOPB;
  } else if (stopBits == 1) {
    ttyOpt.c_cflag &= ~CSTOPB;
  } else {
    std::cout << "Wrong stop bits: 0/1/2" << std::endl;
  }
  // 设置read()函数的超时设置:
  // https://www.cnblogs.com/electron/p/3451114.html
  // 超时时间 -- 1:100ms
  ttyOpt.c_cc[VTIME] = 1;
  // 每次接收数据长度的字节数
  ttyOpt.c_cc[VMIN] = 0;

  // 清空终端未完毕的输入/输出请求及数据
  tcflush(nFd,TCIFLUSH);
  // 立即激活新配置
  if (tcsetattr(nFd, TCSANOW, &ttyOpt) != 0) {
    perror("tcsetattr Error!\n");
    return -1;
  }
  return nFd;
}

// 串口通讯
int SerialPort::write_port(uint8_t* tx, int txLen) {
  /* *** 向串口发送 *** */
  int writeRet = write(nFd, tx, txLen);
  if (writeRet == txLen) {
    return writeRet;
  } else {
    tcflush(nFd, TCOFLUSH);
    return -1;
  }
}

int SerialPort::read_port(uint8_t* rx, int rxLen) {
  int byteToRcv = rxLen, byteRcv = 0, byteRead = 0;
  auto start = std::chrono::steady_clock::now();

  bzero(rx, rxLen);
  // Ref: https://stackoverflow.com/questions/18082073/
  while (byteToRcv > 0) {
    byteRead = read(nFd, rx, byteToRcv);
    if (byteRead > 0) {
      byteToRcv -= byteRead;
      rx += byteRead;
      byteRcv += byteRead;
    } else {
      auto end = std::chrono::steady_clock::now();
      auto duration = end - start;
      auto sec = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
      // 超时处理
      if (sec.count() > msDelay) return byteRead;
    }
  }

  return byteRead;
}

int SerialPort::tranBaud(int baud) {
  int map[][2] = {{2400, B2400},    {4800, B4800},   {9600, B9600},
                  {19200, B19200},  {38400, B38400}, {57600, B57600},
                  {115200, B115200}};
  for (int i = 0; i < sizeof(map) / sizeof(int) / 2; i++)
    if (map[i][0] == baud)
      return map[i][1];
  return -1;
}

int SerialPort::tranDataB(int dataBits) {
  int map[][2] = {{5, CS5}, {6, CS6}, {7, CS7}, {8, CS8}};
  for (int i = 0; i < sizeof(map) / sizeof(int) / 2; i++)
    if (map[i][0] == dataBits)
      return map[i][1];
  return -1;
}

// 功能码初始化
uint8_t RS485Device::rCoil = 0x01;
uint8_t RS485Device::rMulReg = 0x03;
uint8_t RS485Device::wMulReg = 0x10;

RS485Device::RS485Device(uint8_t ID) : slaveID(ID) {}
RS485Device::RS485Device(std::string ttyName, int baud, int dataBits,
                         int parMode, int stopBits, uint8_t ID)
    : slaveID(ID) {
  connect(ttyName, baud, dataBits, parMode, stopBits);
}

RS485Device::~RS485Device() {
}

int RS485Device::connect(std::string ttyName, int baud, int dataBits,
                         int parMode, int stopBits) {
  serial = new SerialPort(ttyName, baud, dataBits, parMode, stopBits);
  return 0;
}

// 读连续寄存器
std::vector<uint8_t> RS485Device::readMulReg(uint16_t addr, uint16_t num) {
  const int cmdLen = 8, rxLen = 5+2*num;
  std::vector<uint8_t> regMsg = {};
  uint8_t hNum = num>>8, lNum = num&0x00FF, hAddr = addr>>8, lAddr = addr&0x00FF;
  uint8_t cmd[cmdLen] = {slaveID, rMulReg, hAddr, lAddr, hNum, lNum};
  // 计算 CRC 校验位
  uint16_t crc = get_crc16(cmd, 6);
  cmd[6] = crc & 0x00FF;
  cmd[7] = crc>>8;
  // 发送串口指令
  serial->write_port(cmd, cmdLen);
  serial->read_port(rxData, rxLen);
  for (int i=0; i<rxLen; ++i) {
    regMsg.emplace_back(rxData[i]);
    // printf("%02X  ", regMsg[i]);
  }
  // std::cout << std::endl;
  return regMsg;
}

std::vector<uint8_t> RS485Device::writeMulReg(uint16_t addr,
                                              std::vector<uint8_t> wData) {
  uint8_t byteNum = wData.size(), regNum = byteNum / 2;
  uint8_t hAddr = addr >> 8, lAddr = addr & 0x00FF;
  std::vector<uint8_t> cmd = {slaveID, wMulReg, hAddr, lAddr, 0, regNum, byteNum};
  std::vector<uint8_t> slaveRet;
  int rxLen = 8;
  // 待写入数据
  for (int i=0; i<wData.size(); ++i) {
    cmd.emplace_back(wData[i]);
  }
  int cmdLen = cmd.size()+2;
  uint8_t cmdArr[cmdLen];
  for (int i=0; i<cmdLen-2; ++i) {
    cmdArr[i] = cmd[i];
  }
  // CRC 校验
  uint16_t crc = get_crc16(cmdArr, cmdLen-2);
  cmdArr[cmdLen-2] = crc & 0x00FF;
  cmdArr[cmdLen-1] = crc>>8;
  // 发送串口指令
  serial->write_port(cmdArr, cmdLen);
  serial->read_port(rxData, rxLen);
  for (int i=0; i<rxLen; ++i) {
    slaveRet.emplace_back(rxData[i]);
  }
  return slaveRet;
}

// CRC-16 校验
// @param : *data-数据帧首地址(最高位); len-数据Bit位数;
// @return: uint16_t-两字节校验码(高位在左)。
uint16_t RS485Device::get_crc16(uint8_t *data, int len) {
  // 预置 CRC 寄存器
  uint16_t crc = 0xFFFF, lsb;

  for (int i = 0; i < len; ++i) {
    // 高8位与CRC寄存器异或
    crc ^= data[i];
    // 逐位移出并检测
    for (int j = 0; j < 8; ++j) {
      lsb = crc & 0x0001;
      crc = crc >> 1;
      // 最低位检测
      if (lsb != 0) crc ^= 0xA001;
    }
  }
  return crc;
}

