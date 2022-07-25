#include "rs485_driver.h"

int main(int argc, char** argv) {
  uint8_t txData[] = {0x01, 0x03, 0x01, 0x00, 0x00, 0x0C, 0x44, 0x33};
  uint8_t zero[] = {0x01, 0x10, 0x03, 0x20, 0x00, 0x02, 0x04,
                    0x00, 0x00, 0x00, 0x0A, 0x65, 0x40};
  std::vector<uint8_t> regRet;

  RS485Device sensor;

  sensor.connect("/dev/ttyUSB0", 19200, 8, 0, 1);
  regRet = sensor.writeMulReg(0x0320, {0x00, 0x00, 0x00, 0x01});

  for (int i=0; i<regRet.size(); ++i) {
    printf("%02X ", regRet[i]);
  }
  std::cout << std::endl;

  return 0;
}

