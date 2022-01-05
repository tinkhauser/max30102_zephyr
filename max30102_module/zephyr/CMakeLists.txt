# Makefile - MAX30102 heart rate sensor
#
# SPDX-License-Identifier: AGPL-3.0-or-later

if(CONFIG_MAX30102)
    # Add max30102.h to the set of global include paths.
    zephyr_include_directories(.)

    zephyr_library()
    zephyr_library_sources(
        max30102.c
        )
endif()
