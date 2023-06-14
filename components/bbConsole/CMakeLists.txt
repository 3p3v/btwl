# idf_component_register(SRCS "Sim800l.cpp" "Sim800lBB.cpp" "Sim800lESP.cpp"
#                     INCLUDE_DIRS "./include/"
#                     REQUIRES driver)

# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES driver nvs_flash)
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "Console.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
