file(GLOB INTERFACE_HEADERS *.h)
file(GLOB INTERFACE_SOURCES *.cpp *.c)


add_executable(interface_node ${INTERFACE_SOURCES} ${INTERFACE_HEADERS})

target_link_libraries(interface_node ${catkin_LIBRARIES})

install(TARGETS interface_node DESTINATION ./bin)

