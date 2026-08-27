cmake_minimum_required(VERSION 3.16)
project(g1_voice_interaction LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(CURL REQUIRED)

set(UNITREE_SDK_ROOT "" CACHE PATH "Unitree SDK2 installation or source root")
set(UNITREE_THIRDPARTY_ROOT "${UNITREE_SDK_ROOT}/thirdparty" CACHE PATH
    "Unitree SDK2 third-party dependency root")
find_path(UNITREE_SDK_INCLUDE_DIR
    NAMES unitree/robot/channel/channel_factory.hpp
    PATHS
        "${UNITREE_SDK_ROOT}/include"
        "${UNITREE_SDK_ROOT}"
    NO_DEFAULT_PATH)
find_library(UNITREE_SDK_LIBRARY
    NAMES unitree_sdk2
    PATHS
        "${UNITREE_SDK_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}"
        "${UNITREE_SDK_ROOT}/lib"
    NO_DEFAULT_PATH)
find_library(DDS_LIBRARY
    NAMES ddsc
    PATHS "${UNITREE_THIRDPARTY_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}"
    NO_DEFAULT_PATH)
find_library(DDSXX_LIBRARY
    NAMES ddscxx
    PATHS "${UNITREE_THIRDPARTY_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}"
    NO_DEFAULT_PATH)

if(NOT UNITREE_SDK_INCLUDE_DIR OR NOT UNITREE_SDK_LIBRARY OR
   NOT EXISTS "${UNITREE_THIRDPARTY_ROOT}/include/ddscxx/dds/topic/TopicTraits.hpp" OR
   NOT DDS_LIBRARY OR NOT DDSXX_LIBRARY)
    message(FATAL_ERROR
        "Unitree SDK2 not found. Set -DUNITREE_SDK_ROOT=<SDK root> "
        "or use the original SDK workspace.")
endif()

set(G1_VOICE_SOURCES
    example/g1/audio/g1_audio_client_test.cpp
    example/g1/voice_chat/app_config.cpp
    example/g1/voice_chat/asr_processor.cpp
    example/g1/voice_chat/command_router.cpp
    example/g1/voice_chat/llm_client.cpp
    example/g1/voice_chat/robot_controller.cpp
    example/g1/voice_chat/voice_assistant.cpp)

add_executable(g1_audio_client_test ${G1_VOICE_SOURCES})
target_include_directories(g1_audio_client_test PRIVATE
    "${UNITREE_SDK_INCLUDE_DIR}"
    "${UNITREE_THIRDPARTY_ROOT}/include"
    "${UNITREE_THIRDPARTY_ROOT}/include/ddscxx"
    "${CMAKE_CURRENT_SOURCE_DIR}/example/g1")
target_link_libraries(g1_audio_client_test PRIVATE
    "${UNITREE_SDK_LIBRARY}"
    "${DDSXX_LIBRARY}"
    "${DDS_LIBRARY}"
    CURL::libcurl
    Threads::Threads)
target_compile_options(g1_audio_client_test PRIVATE -Wall -Wextra -Wpedantic)

add_executable(g1_voice_logic_test
    example/g1/voice_chat/voice_logic_test.cpp
    example/g1/voice_chat/asr_processor.cpp
    example/g1/voice_chat/command_router.cpp)
target_include_directories(g1_voice_logic_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/example/g1")
target_compile_options(g1_voice_logic_test PRIVATE -Wall -Wextra -Wpedantic)
