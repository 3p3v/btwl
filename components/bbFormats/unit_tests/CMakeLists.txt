add_library(bbformats STATIC ${CMAKE_CURRENT_SOURCE_DIR}/../BBFormats.cpp)
target_include_directories(bbformats PUBLIC 
	"../include/"
)
add_library(bbformats_tests OBJECT bbformats.test.cpp)

target_link_libraries(bbformats_tests PRIVATE Catch2::Catch2)
target_link_libraries(bbformats_tests PRIVATE fff)
target_link_libraries(bbformats_tests PRIVATE bbformats)