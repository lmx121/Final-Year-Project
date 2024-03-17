# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/frameworks/esp-idf-v5.1.1/components/bootloader/subproject"
  "C:/LUOMINGXI/project/ESP/8lvgl/spi_lcd_touch/build/bootloader"
  "C:/LUOMINGXI/project/ESP/8lvgl/spi_lcd_touch/build/bootloader-prefix"
  "C:/LUOMINGXI/project/ESP/8lvgl/spi_lcd_touch/build/bootloader-prefix/tmp"
  "C:/LUOMINGXI/project/ESP/8lvgl/spi_lcd_touch/build/bootloader-prefix/src/bootloader-stamp"
  "C:/LUOMINGXI/project/ESP/8lvgl/spi_lcd_touch/build/bootloader-prefix/src"
  "C:/LUOMINGXI/project/ESP/8lvgl/spi_lcd_touch/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/LUOMINGXI/project/ESP/8lvgl/spi_lcd_touch/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/LUOMINGXI/project/ESP/8lvgl/spi_lcd_touch/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
