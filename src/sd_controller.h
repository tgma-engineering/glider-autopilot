#ifndef SD_CONTROLLER_H_
#define SD_CONTROLLER_H_

#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

class SdController {
public:
    static const uint8_t kSckGpio = 18;
    static const uint8_t kMisoGpio = 5;
    static const uint8_t kMosiGpio = 10;
    static const uint8_t kSsGpio = 9;

    int8_t setup(const String& file_name);
    int8_t append(const String& str);

private:
    SPIClass spi_;
    String file_name_;
};

#endif  // SD_CONTROLLER_H_