cmake_minimum_required(VERSION 3.10)
project(ac3dec VERSION 1.2.0 LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

option(AC3DEC_BUILD_TESTS "Build the test programs" ON)

set(AC3DEC_SOURCES
    src/ac3_tables.c
    src/ac3_frame.c
    src/ac3_bitalloc.c
    src/ac3_block.c
    src/ac3_imdct.c
    src/ac3_downmix.c
    src/eac3_dec.c
    src/eac3_tables.c
    src/ac3_decode.c
    src/dolby_e.c
    src/dolby_e_parse.c)

add_library(ac3dec STATIC ${AC3DEC_SOURCES})
target_include_directories(ac3dec PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
if(NOT MSVC)
    target_link_libraries(ac3dec PUBLIC m)
endif()

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(ac3dec PRIVATE -Wall -Wextra)
endif()

add_executable(ac3dec_example example_decoder.c)
target_link_libraries(ac3dec_example PRIVATE ac3dec)

add_executable(dolby_e_example dolby_e_example.c)
target_link_libraries(dolby_e_example PRIVATE ac3dec)

if(AC3DEC_BUILD_TESTS)
    enable_testing()

    add_executable(test_api tests/test_api.c)
    target_link_libraries(test_api PRIVATE ac3dec)
    add_test(NAME api COMMAND test_api ${CMAKE_CURRENT_SOURCE_DIR}/tests/data)

    add_executable(test_imdct tests/test_imdct.c)
    target_include_directories(test_imdct PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(test_imdct PRIVATE ac3dec)
    add_test(NAME imdct COMMAND test_imdct)
endif()

