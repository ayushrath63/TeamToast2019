
#ifndef __IMU_H__
#define __IMU_H__


#include "gpio.h"
#include "spi.h"
class IMU
{
public:
    explicit IMU(SPI_HandleTypeDef hspi) : spiHandle_(hspi)
    {}
    void init();
    int8_t read();
private:
    SPI_HandleTypeDef spiHandle_;
};

#endif /* __IMU_H__ */