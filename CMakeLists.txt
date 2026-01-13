cmake_minimum_required(VERSION 3.16)
project(RobotAPI CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# MocapApi import target
add_library(MocapApi::MocapApi SHARED IMPORTED)
set_target_properties(MocapApi::MocapApi PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/third_party/MocapApi/lib/MocapApi.dll"
    IMPORTED_IMPLIB "${CMAKE_CURRENT_SOURCE_DIR}/third_party/MocapApi/lib/MocapApi.lib"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/third_party/MocapApi/include"
)

add_executable(robot_api_demo2
    robot_api_demo2.cpp
)

target_include_directories(robot_api_demo2 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/MocapApi/include
)

# Link to the DLL target so the demo uses the generated import library (robot_api.lib)
target_link_libraries(robot_api_demo2
    PRIVATE
        robot_api_shared
        MocapApi::MocapApi
)

add_custom_command(TARGET robot_api_demo2 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/third_party/MocapApi/lib/MocapApi.dll"
        $<TARGET_FILE_DIR:robot_api_demo2>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:robot_api_shared>
        $<TARGET_FILE_DIR:robot_api_demo2>
)

add_subdirectory(librobot)

add_library(robot_api_shared SHARED
    mcp_robot.cpp
    mcp_robot.h
    handle_manager.hpp
    robot_api.cpp
    robot_api.h
)

target_include_directories(robot_api_shared
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/librobot/include
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/MocapApi/include
)

target_link_libraries(robot_api_shared
    PUBLIC
        librobot
        MocapApi::MocapApi
)

target_compile_definitions(robot_api_shared
    PRIVATE ROBOT_API_SHARED_BUILD
    PUBLIC ROBOT_API_SHARED)

set_target_properties(robot_api_shared PROPERTIES
    OUTPUT_NAME "robot_api"
)

add_custom_command(TARGET robot_api_shared POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/third_party/MocapApi/lib/MocapApi.dll"
        $<TARGET_FILE_DIR:robot_api_shared>
)

add_executable(robot_api_demo
    robot_api_demo.cpp
)

target_include_directories(robot_api_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/MocapApi/include
)

target_link_libraries(robot_api_demo
    PRIVATE
        robot_api_shared
        MocapApi::MocapApi
)

add_custom_command(TARGET robot_api_demo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/third_party/MocapApi/lib/MocapApi.dll"
        $<TARGET_FILE_DIR:robot_api_demo>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:robot_api_shared>
        $<TARGET_FILE_DIR:robot_api_demo>
)

add_executable(robot_api_demo3
    robot_api_demo3.cpp
)

target_include_directories(robot_api_demo3 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/MocapApi/include
)

target_compile_definitions(robot_api_demo3 PRIVATE GLM_ENABLE_EXPERIMENTAL)

target_link_libraries(robot_api_demo3
    PRIVATE
        robot_api_shared
        MocapApi::MocapApi
)

add_custom_command(TARGET robot_api_demo3 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/third_party/MocapApi/lib/MocapApi.dll"
        $<TARGET_FILE_DIR:robot_api_demo3>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:robot_api_shared>
        $<TARGET_FILE_DIR:robot_api_demo3>
)
