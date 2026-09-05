cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nrf_epd_minimal_draw)

target_sources(app PRIVATE src/main.c)
