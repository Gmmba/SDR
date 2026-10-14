cmake_minimum_required(VERSION 3.16)
project(PlutoSDR CXX)

set(CMAKE_BUILD_TYPE Debug)

option(INSTALL_DEPS "Установить зависимости" ON)
option(PLUTO_TIMESTAMP "Build pluto with timestamp" ON)

find_package(SoapySDR REQUIRED)

set(MAIN_SOURCE_FILES
    main.cpp
)

# Добавляем исполняемый файл
add_executable(main ${MAIN_SOURCE_FILES})

target_link_libraries(main ${SoapySDR_LIBRARIES})