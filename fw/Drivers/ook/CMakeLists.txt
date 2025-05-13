add_library(ook_driver "")
set_target_properties(ook_driver PROPERTIES LINKER_LANGUAGE C)

target_sources(ook_driver
    PRIVATE
        ook.c
    PUBLIC
        ook.h
)

target_include_directories(ook_driver PUBLIC ${CMAKE_CURRENT_LIST_DIR})
