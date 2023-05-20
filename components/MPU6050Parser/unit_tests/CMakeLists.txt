add_library(mpu6050parser STATIC ${CMAKE_CURRENT_SOURCE_DIR}/../MPU6050Parser.cpp)
target_include_directories(mpu6050parser PUBLIC 
	"../include/"
)
add_library(mpu6050parser_tests OBJECT mpu6050parser.test.cpp)

target_link_libraries(mpu6050parser_tests PRIVATE Catch2::Catch2)
target_link_libraries(mpu6050parser_tests PRIVATE fff)
target_link_libraries(mpu6050parser_tests PRIVATE mpu6050parser)
target_link_libraries(mpu6050parser PUBLIC common)
target_link_libraries(mpu6050parser PUBLIC bbformats)