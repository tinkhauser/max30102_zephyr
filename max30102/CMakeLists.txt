# 
# Copyright (c) 2022 Jacob Tinkhauser
# Copyright (c) 2022 EVERGREEN FUND 501(c)3
#
# SPDX-License-Identifier: AGPL-3.0-or-later

zephyr_library()

zephyr_library_sources(max30102.c)
zephyr_library_sources_ifdef(CONFIG_MAX30102_TRIGGER max30102_trigger.c)