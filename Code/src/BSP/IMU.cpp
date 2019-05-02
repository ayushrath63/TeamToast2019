#include "IMU.hpp"
#include "usart.h"

void IMU::init()
{
  uint8_t buf[2];
  buf[0] = 0x6A; //User control reg
  buf[1] = 0x10; //I2c disable bit

  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&spiHandle_, buf, sizeof(buf), 100);
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  buf[0] = 0x6B; //power mgmt register
  buf[1] = 0x80; //reset
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&spiHandle_, buf, sizeof(buf), 100);
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  buf[0] = 0x68; //signal path
  buf[1] = 0x07; //reset
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&spiHandle_, buf, sizeof(buf), 100);
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  buf[0] = 0x6B; //signal path
  buf[1] = 0x03; //gyro pll select
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&spiHandle_, buf, sizeof(buf), 100);
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  //IMU -> read WHOAMI reg
  uint8_t addr, data;
  addr = 0x75 | 0x80; // WHOAMI | read flag
  data = 0;
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&spiHandle_, &addr, sizeof(addr), 100);
  HAL_SPI_Receive(&spiHandle_, &data, sizeof(data), 100);
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  if(data != 0x70)
  {
    Error_Handler();
  }
  
  //Notes:
  /*
  Registers 19 to 24 – Gyro Offset Registers
  25 -> sample rate divider
  26-> config
  */
  //Configuration
  //set sample rate to 1khz
  buf[0] = 0x19; //Sample rate divider
  buf[1] = 0x00; //1KHz
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&spiHandle_, buf, sizeof(buf), 100);
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  //setup dlpf
  buf[0] = 0x1A; //Config
  buf[1] = 0x01; //DLPF -> 184Hz
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&spiHandle_, buf, sizeof(buf), 100);
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  
  //set scale
  buf[0] = 0x1A; //Gyro Config
  buf[1] = 0x00; // 0x18; //FS -> 2000dps
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&spiHandle_, buf, sizeof(buf), 100);
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
}

int8_t IMU::read()
{
  uint8_t addrH = (0x47) | 0x80; // Gyro Z high byte | read flag
  uint8_t addrL = (0x48) | 0x80; // Gyro Z low byte | read flag
  int16_t gzData;
  uint8_t dataH, dataL;
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&spiHandle_, &addrH, sizeof(addrH), 100);
  HAL_SPI_Receive(&spiHandle_, &dataH, sizeof(dataH), 100);
  HAL_SPI_Transmit(&spiHandle_, &addrL, sizeof(addrL), 100);
  HAL_SPI_Receive(&spiHandle_, &dataL, sizeof(dataL), 100);
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  // char buf[32];
  // sprintf(buf, "H:%d, L:%d\r\n", dataH, dataL);
  // print((uint8_t*)buf);
  //Low byte broken :(
  gzData = (int8_t)(dataH);//((int16_t)dataH << 8) | (dataL);
  return gzData;
}