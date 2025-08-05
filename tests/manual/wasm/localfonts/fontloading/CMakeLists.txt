# Copyright (C) 2023 The Qt Company Ltd.
# SPDX-License-Identifier: BSD-3-Clause
cmake_minimum_required(VERSION 3.16)

enable_language(CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

message(Current Build Type: $<CONFIG>)
message(CMAKE_CONFIGURATION_TYPES=${CMAKE_CONFIGURATION_TYPES})

find_package(QT NAMES Qt6 REQUIRED COMPONENTS Core)
find_package(Qt6 REQUIRED COMPONENTS Gui Widgets)

qt_add_executable(fontloading
        main.cpp
)

target_link_libraries(fontloading PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
)

add_custom_command(
    TARGET fontloading POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_SOURCE_DIR}/fontloading.html
            ${CMAKE_CURRENT_BINARY_DIR}/fontloading.html)
#add_custom_target(html DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/fontloading.html)
#add_dependencies(fontloading html)
