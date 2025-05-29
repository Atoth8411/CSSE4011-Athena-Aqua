cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(cam_streaming)

# cmake_minimum_required(VERSION 3.16)
# project(cam_streaming)

# idf_component_register(SRCS "main.c" "other_sources.c"
#                        INCLUDE_DIRS "."
#                        REQUIRES esp_camera driver esp_timer log freertos)

# # Instead of building an executable, build a static lib:
# add_library(cam_streaming_lib STATIC ${COMPONENT_SRCS})

# target_include_directories(cam_streaming_lib PUBLIC ${COMPONENT_ADD_INCLUDEDIRS})

# cmake_minimum_required(VERSION 3.16)

# # This line locates the ESP-IDF build system and sets variables/functions like idf_component_register()
# include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# project(cam_streaming)

# idf_component_register(
#     SRCS "main.c" "other_source_files.c"
#     INCLUDE_DIRS "."
#     REQUIRES esp_camera driver esp_timer log freertos
# )