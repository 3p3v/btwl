# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES "minmea" "bbFormats" "driver") #"TinyEKF")
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "gps.cpp" "gpsESP.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
