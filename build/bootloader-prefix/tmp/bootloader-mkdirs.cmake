# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/frameworks/esp-idf-v4.4.3/components/bootloader/subproject"
  "C:/Users/DELL/ESP32-ESP-IDF-BME280-Web-Server-main/build/bootloader"
  "C:/Users/DELL/ESP32-ESP-IDF-BME280-Web-Server-main/build/bootloader-prefix"
  "C:/Users/DELL/ESP32-ESP-IDF-BME280-Web-Server-main/build/bootloader-prefix/tmp"
  "C:/Users/DELL/ESP32-ESP-IDF-BME280-Web-Server-main/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/DELL/ESP32-ESP-IDF-BME280-Web-Server-main/build/bootloader-prefix/src"
  "C:/Users/DELL/ESP32-ESP-IDF-BME280-Web-Server-main/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/DELL/ESP32-ESP-IDF-BME280-Web-Server-main/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
