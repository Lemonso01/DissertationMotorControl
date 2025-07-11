# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/mnt/e/DissMotCont/MotorControl/build/_deps/picotool-src"
  "/mnt/e/DissMotCont/MotorControl/build/_deps/picotool-build"
  "/mnt/e/DissMotCont/MotorControl/build/_deps"
  "/mnt/e/DissMotCont/MotorControl/build/pico-sdk/src/rp2350/boot_stage2/picotool/tmp"
  "/mnt/e/DissMotCont/MotorControl/build/pico-sdk/src/rp2350/boot_stage2/picotool/src/picotoolBuild-stamp"
  "/mnt/e/DissMotCont/MotorControl/build/pico-sdk/src/rp2350/boot_stage2/picotool/src"
  "/mnt/e/DissMotCont/MotorControl/build/pico-sdk/src/rp2350/boot_stage2/picotool/src/picotoolBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/mnt/e/DissMotCont/MotorControl/build/pico-sdk/src/rp2350/boot_stage2/picotool/src/picotoolBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/mnt/e/DissMotCont/MotorControl/build/pico-sdk/src/rp2350/boot_stage2/picotool/src/picotoolBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
