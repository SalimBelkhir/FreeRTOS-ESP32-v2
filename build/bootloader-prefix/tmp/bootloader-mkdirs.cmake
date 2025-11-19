# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/pc/esp32/esp-idf/components/bootloader/subproject"
  "/home/pc/esp32/esp-idf/rtos/build/bootloader"
  "/home/pc/esp32/esp-idf/rtos/build/bootloader-prefix"
  "/home/pc/esp32/esp-idf/rtos/build/bootloader-prefix/tmp"
  "/home/pc/esp32/esp-idf/rtos/build/bootloader-prefix/src/bootloader-stamp"
  "/home/pc/esp32/esp-idf/rtos/build/bootloader-prefix/src"
  "/home/pc/esp32/esp-idf/rtos/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/pc/esp32/esp-idf/rtos/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/pc/esp32/esp-idf/rtos/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
