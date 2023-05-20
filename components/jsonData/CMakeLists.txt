# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES "cJSON" "gps" "MPU6050Parser" "SHT30")
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "JsonData.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
