# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# compile ASM with /usr/bin/arm-none-eabi-gcc
ASM_DEFINES = -DLIB_BOOT_STAGE2_HEADERS=1 -DPICO_32BIT=1 -DPICO_BOARD=\"pico2\" -DPICO_BUILD=1 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_RP2350=1

ASM_INCLUDES = -I/home/goncalo_gcosta/pico/pico-sdk/src/rp2350/boot_stage2/asminclude -isystem /home/goncalo_gcosta/pico/pico-sdk/src/rp2350/hardware_regs/include -isystem /home/goncalo_gcosta/pico/pico-sdk/src/rp2_common/hardware_base/include -isystem /home/goncalo_gcosta/pico/pico-sdk/src/common/pico_base_headers/include -isystem /mnt/e/DissMotCont/MotorControl/build/generated/pico_base -isystem /home/goncalo_gcosta/pico/pico-sdk/src/boards/include -isystem /home/goncalo_gcosta/pico/pico-sdk/src/rp2350/pico_platform/include -isystem /home/goncalo_gcosta/pico/pico-sdk/src/rp2_common/pico_platform_compiler/include -isystem /home/goncalo_gcosta/pico/pico-sdk/src/rp2_common/pico_platform_panic/include -isystem /home/goncalo_gcosta/pico/pico-sdk/src/rp2_common/pico_platform_sections/include -isystem /home/goncalo_gcosta/pico/pico-sdk/src/rp2_common/hardware_dcp/include -isystem /home/goncalo_gcosta/pico/pico-sdk/src/rp2350/hardware_structs/include -isystem /home/goncalo_gcosta/pico/pico-sdk/src/rp2_common/hardware_rcp/include -isystem /home/goncalo_gcosta/pico/pico-sdk/src/rp2350/boot_stage2/include

ASM_FLAGS = -mcpu=cortex-m33 -mthumb -march=armv8-m.main+fp+dsp -mfloat-abi=softfp -mcmse -g -O3 -DNDEBUG

