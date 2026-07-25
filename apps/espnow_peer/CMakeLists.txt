# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(waveshare_espnow_peer)

set(ESPNOW_BLOB
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/espressif/libespnow.a)

if(NOT EXISTS ${ESPNOW_BLOB})
  message(FATAL_ERROR "Missing ESP-NOW library: ${ESPNOW_BLOB}")
endif()

# This Zephyr checkout predates hal_espressif's ESP-NOW blob integration.
# Keep the compatible upstream C6 library local to this application.
zephyr_link_libraries(${ESPNOW_BLOB})

# The older HAL also provides g_espnow_user_oui as a compatibility stub, while
# the upstream ESP-NOW blob now contains the same constant itself.
zephyr_ld_options(-Wl,--allow-multiple-definition)

target_sources(app PRIVATE
               src/main.c
               src/rgb_indicator.c)
target_sources_ifdef(CONFIG_ESPNOW_NODE_ROLE_MASTER app PRIVATE
                     src/ti_uart_link.c)
