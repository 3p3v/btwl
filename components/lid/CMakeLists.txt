# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES driver)
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "LidController.cpp" "Lid.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()