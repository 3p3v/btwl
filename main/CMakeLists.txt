idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES sim800l 
                             driver
                             esp_timer
                             esp_system
                             hal
                             I2Cdev
                             hmc5883lUni
                             bbFormats
                             gps
                             MPU6050
                             MPU6050Parser
                             SHT30
                             jsonData
                             lid
                             boxMessage)
