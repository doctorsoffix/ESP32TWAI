add_library(usermod_tftcore INTERFACE)

target_sources(usermod_tftcore INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/tftcore.c
)

target_include_directories(usermod_tftcore INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(usermod INTERFACE usermod_tftcore)
