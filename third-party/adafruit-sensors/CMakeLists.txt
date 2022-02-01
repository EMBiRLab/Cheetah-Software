add_library (I2CDevice STATIC I2CDevice.h I2CDevice.cpp)
add_library (Adafruit_ADS1X15 STATIC I2CDevice.h Adafruit_ADS1X15.h Adafruit_ADS1X15.cpp)
add_library (Adafruit_INA260 STATIC I2CDevice.h Adafruit_INA260.h Adafruit_INA260.cpp)