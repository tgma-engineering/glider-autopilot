#include "sd_controller.h"

int8_t SdController::setup(const String& file_name) {
    if (file_name[0] == '/') {
        file_name_ = file_name;
    } else {
        file_name_ = "/" + file_name;
    }

    Serial.println("Setup SPI ...");
    spi_.begin(kSckGpio, kMisoGpio, kMosiGpio, kSsGpio);

    Serial.println("Setup SD ...");
    if (!SD.begin(kSsGpio, spi_)) {
        Serial.println("Error: SD Setup failed");
        return -1;
    }
    return 0;
}

int8_t SdController::append(const String& str) const {
    if (SD.usedBytes() >= SD.totalBytes()) {
        Serial.println("Warning: SD Card is full. No data will be logged");
        return -1;
    }

    File file_ = SD.open(file_name_, FILE_APPEND, true);  // Creates file if none exists
    if (file_) {
        // Write data
        if (!file_.print(str)) {
            Serial.println("Warning: Writing failed");
            file_.close();
            return -1;
        }
    } else {
        Serial.println("Warning: Failed to open file. No data will be logged");
        return -1;
    }

    file_.close();
    return 0;
}