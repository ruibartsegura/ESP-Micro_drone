# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/rui/esp-idf/components/bootloader/subproject"
  "/home/rui/uni/TFG/ESP-Micro_drone/code/esp_code/build/bootloader"
  "/home/rui/uni/TFG/ESP-Micro_drone/code/esp_code/build/bootloader-prefix"
  "/home/rui/uni/TFG/ESP-Micro_drone/code/esp_code/build/bootloader-prefix/tmp"
  "/home/rui/uni/TFG/ESP-Micro_drone/code/esp_code/build/bootloader-prefix/src/bootloader-stamp"
  "/home/rui/uni/TFG/ESP-Micro_drone/code/esp_code/build/bootloader-prefix/src"
  "/home/rui/uni/TFG/ESP-Micro_drone/code/esp_code/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/rui/uni/TFG/ESP-Micro_drone/code/esp_code/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/rui/uni/TFG/ESP-Micro_drone/code/esp_code/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
