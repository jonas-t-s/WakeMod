add_library(logging "")
set_target_properties(logging PROPERTIES LINKER_LANGUAGE C)

set(ULOG_PORT_INC_DIR ${CMAKE_CURRENT_LIST_DIR})

add_subdirectory(segger)
add_subdirectory(ulog)

target_link_libraries(logging PUBLIC segger_rtt ulog)

target_sources(logging
    PRIVATE
        logging.c
    PUBLIC
        logging.h
)

target_include_directories(logging PUBLIC ${CMAKE_CURRENT_LIST_DIR})
