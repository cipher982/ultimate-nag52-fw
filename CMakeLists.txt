cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
add_compile_options(-fno-ipa-sra -fno-partial-inlining)
project(fw)
