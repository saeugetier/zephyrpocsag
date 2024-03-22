cmake_minimum_required(VERSION 3.22)
set(BOARD nrf52832_mdk)

find_package(Zephyr)
project(zephyr_pocsag)

target_sources(app PRIVATE src/main.c)

add_subdirectory(drivers)
