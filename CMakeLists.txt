# SPDX-License-Identifier: AGPL-3.0-or-later

list(APPEND ZEPHYR_EXTRA_MODULES
  ${CMAKE_CURRENT_SOURCE_DIR}/max30102_module
  )

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(alyce)

target_sources(app PRIVATE
  src/main.c
  )
