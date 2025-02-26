add_library(pid_pico STATIC src/pid.c)

target_link_libraries(pid_pico pico_stdlib)

target_include_directories(pid_pico PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
