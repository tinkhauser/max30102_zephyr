#
# Copyright (c) 2022 Jacob Tinkhauser
# Copyright (c) 2022 EVERGREEN FUND 501(c)3
#
# SPDX-License-Identifier: AGPL-3.0-or-later

# Point to this project's root directory.
set(MAX_WORK_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "Max30102 driver root directory")

# Subdirectories
add_subdirectory(max30102)

# Include headers
zephyr_include_directories(max30102)
