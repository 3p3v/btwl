set(COMPONENT_REQUIRES "bbFormats" "driver") #"TinyEKF")
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "SHT30Readings.cpp" "SHT30Parser.cpp" "SHT30.cpp" "SHT30Esp8266.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
