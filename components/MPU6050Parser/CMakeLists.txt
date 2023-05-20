# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES "MPU6050" "bbFormats")
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "MPU6050Parser.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
