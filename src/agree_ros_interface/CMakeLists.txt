file(GLOB THIS_APP_SOURCES *.c *.cpp )
file(GLOB THIS_APP_HEADERS *.h h)

include_directories($ENV{HOME}/esmacat_rt/esmacat_master_high_performance)
include_directories($ENV{HOME}/esmacat_core/include)
include_directories($ENV{HOME}/esmacat_slave_drivers/drivers)

get_filename_component(THIS_APP_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message(STATUS "Example application, ${THIS_APP_NAME}, is ready to build")

add_executable(${THIS_APP_NAME} ${THIS_APP_SOURCES} ${THIS_APP_HEADERS})
add_dependencies(${THIS_APP_NAME} agree_esmacat_pkg_generate_messages_cpp)
target_link_libraries(${THIS_APP_NAME} ${catkin_LIBRARIES})

install(TARGETS ${THIS_APP_NAME} DESTINATION ./bin)


