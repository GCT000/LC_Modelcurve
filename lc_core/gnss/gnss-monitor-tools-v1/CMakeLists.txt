aux_source_directory (. SRC_LIST)

include_directories (include)

set(CMAKE_C_FLAGS "-std=gnu99")
add_definitions(-D_DEBUG -D_CRT_SECURE_NO_DEPRECATE)
#LINK_LIBRARIES(m)


add_library(DMonitor SHARED ${SRC_LIST})

target_link_libraries(DMonitor pthread)

target_link_libraries(DMonitor m)
