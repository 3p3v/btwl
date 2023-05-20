add_library(cjson STATIC ${CMAKE_CURRENT_SOURCE_DIR}/../cJSON.c ${CMAKE_CURRENT_SOURCE_DIR}/../cJSON_Utils.c)
target_include_directories(cjson PUBLIC 
	"../include/"
)