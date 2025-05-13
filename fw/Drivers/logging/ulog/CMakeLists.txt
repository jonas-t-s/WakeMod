add_library(ulog "")
set_target_properties(ulog PROPERTIES LINKER_LANGUAGE C)

target_sources(ulog
    PRIVATE
        ulog.c
    PUBLIC
        ulog.h
)

target_include_directories(ulog PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_include_directories(ulog PUBLIC ${ULOG_PORT_INC_DIR})
