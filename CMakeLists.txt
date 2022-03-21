cmake_minimum_required(VERSION 3.18)

project(mpu9250)
 
include_directories(
    include
)
 
set(SOURCES 
    src/mpu9250/mpu9250.c 
    example/main.c
)

add_executable(mpu9250 ${SOURCES})
