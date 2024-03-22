cmake_minimum_required(VERSION 3.20.4)

if(CONFIG_RADIO_FSK_SX1276)
include_directories(.)
message(ERROR "Test")
target_sources(app PRIVATE sx1276_fsk.c)
endif()