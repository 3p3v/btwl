add_library(gps STATIC ${CMAKE_CURRENT_SOURCE_DIR}/../gps.cpp) # CMAKE_CURRENT_SOURCE_DIR - current directory
target_include_directories(gps PUBLIC 
	"../include/"
)
add_library(gps_tests OBJECT gps.test.cpp)

#add_library(minmea STATIC  ../../minmea/minmea.c ../../minmea/include/minmea.h)
#target_include_directories(minmea PUBLIC 
#	"../../minmea/include"
#)

target_link_libraries(gps_tests PRIVATE Catch2::Catch2)
target_link_libraries(gps_tests PRIVATE fff)
target_link_libraries(gps_tests PRIVATE gps)
target_link_libraries(gps PUBLIC common)
target_link_libraries(gps PUBLIC minmea)
target_link_libraries(gps_tests PUBLIC minmea)
target_link_libraries(gps PUBLIC bbformats)