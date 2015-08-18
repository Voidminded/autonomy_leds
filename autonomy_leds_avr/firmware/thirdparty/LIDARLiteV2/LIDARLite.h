#include <stdint.h>

extern "C"
{
  #include <util/delay.h>
  #include "i2c.h"
}


class LIDARLite
{
  public:
      LIDARLite();
      void begin(uint8_t configuration = 0, bool fasti2c = false,
        bool showErrorReporting = false, uint8_t LidarLiteI2cAddress = 0x62);
      void configure(uint8_t configuration = 0, uint8_t LidarLiteI2cAddress = 0x62);
      void beginContinuous(bool modePinLow = true, uint8_t interval = 0x04, uint8_t numberOfReadings = 0xff, uint8_t LidarLiteI2cAddress = 0x62);
      // void fast(int8_t = 0x62);
      uint16_t distance(bool stablizePreampFlag = true, bool takeReference = true, uint8_t LidarLiteI2cAddress = 0x62);
      uint16_t distanceContinuous(uint8_t LidarLiteI2cAddress = 0x62);
      // void scale(int8_t, int8_t = 0x62);
      // int16_t velocity(int8_t = 0x62);
      // int16_t signalStrength(int8_t = 0x62);
      // void correlationRecordToArray(int16_t*,int16_t = 256, int8_t = 0x62);
      // void correlationRecordToSerial(int8_t = '\n', int16_t = 256, int8_t = 0x62);
      // uint8_t changeAddress(int8_t, bool = false, int8_t = 0x62);
      // void changeAddressMultiPwrEn(int16_t , int16_t* , uint8_t* , bool = false);
      bool write(uint8_t myAddress, uint8_t myValue, uint8_t LidarLiteI2cAddress = 0x62);
      void read(uint8_t myAddress, uint8_t numOfBytes, uint8_t* arrayToSave, bool monitorBusyFlag, uint8_t LidarLiteI2cAddress = 0x62);
  private:
      static bool errorReporting;
};
