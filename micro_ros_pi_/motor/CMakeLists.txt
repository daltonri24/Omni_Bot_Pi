add_library(motor_pico STATIC src/motor.c)

target_link_libraries(motor_pico pico_stdlib hardware_pwm)

target_include_directories(motor_pico PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
