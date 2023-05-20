add_library(jsondata STATIC ${CMAKE_CURRENT_SOURCE_DIR}/../JsonData.cpp)
target_include_directories(jsondata PUBLIC 
	"../include/"
)
add_library(jsondata_tests OBJECT jsonData.test.cpp)

target_link_libraries(jsondata_tests PRIVATE Catch2::Catch2)
target_link_libraries(jsondata_tests PRIVATE fff)
target_link_libraries(jsondata_tests PRIVATE jsondata)
target_link_libraries(jsondata PUBLIC cjson)
target_link_libraries(jsondata PUBLIC gps)
target_link_libraries(jsondata PUBLIC mpu6050parser)