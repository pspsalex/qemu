# Patches

* ADC:
  * Initial implementation

* PSRAM (and DPORT):
  * Link SPI bus to PSRAM to allow the simulation to read the SPI frequency
  * Use same host memory region for both CPUs

* GPIO:
  * Allow using all 48 GPIO pins
  * Trigger interrupts on GPIO IN changes
  * Implement behavior for some GPIO registers

* ~~SD:~~
  * ~~Update base SD driver implementation from qemu main repo~~

* SPI:
  * Enable DMA
  * Fix indexing bug in esp32_spi_txrx_buffer

* ESP32 Machine:
  * Enable ADC and PSRAM
  * Allow machine to be derive into M5Paper

* M5Paper Machine:
  * Initial implementation, derived from ESP32 Machine

* IT8951E:
  * Initial implementation

* GT911:
  * Initial implementation

* M5Paper Buttons:
  * Initial implementation


# Configure the project

```bash
mkdir build
cd build
../configure --target-list=xtensa-softmmu --enable-gcrypt --enable-debug --disable-sanitizers --disable-strip --disable-user --disable-capstone --disable-vnc --prefix=/opt/esp-qemu --cc="ccache gcc"
```


# Prepare the files
## SD Card Image
```bash
# create 256MB "sd-image.bin" SD card image
dd if=/dev/zero of=sd-image.bin bs=1M count=256

# create FAT32 filesystem
mkfs.vfat sd-image.bin
```

## E-Fuse
```
# dump efuse from an ESP32
espefuse.py dump --file_name esp32-efuse-block.bin

# merge the dumps in one file
cat esp32-efuse-block*.bin > esp32-efuse.bin
```

You can use the the following sample efuse file if you don't have an ESP32 handy:
```bash
cat <<EOL | xxd -r - esp32-efuse.bin
00000000: 0000 0000 586e 5a7e b994 e200 00a2 0000  ....XnZ~........
00000010: 3303 0000 0000 1000 0400 0000 0000 0000  3...............
00000020: 0000 0000 0000 0000 0000 0000 0000 0000  ................
00000030: 0000 0000 0000 0000 0000 0000 0000 0000  ................
00000040: 0000 0000 0000 0000 0000 0000 0000 0000  ................
00000050: 0000 0000 0000 0000 0000 0000 0000 0000  ................
00000060: 0000 0000 0000 0000 0000 0000 0000 0000  ................
00000070: 0000 0000 0000 0000 0000 0000            ............
EOL
```

More info on using `espufse.py` can be found in the [IDF documentation](https://docs.espressif.com/projects/esptool/en/latest/esp32/espefuse/dump-cmd.html).


# Run the stuff
```bash
# Set rotation to 90 if the app is running in portrait mode
./build/qemu-system-xtensa -m 4M  -machine m5paper -drive file=merged_qemu.bin,if=mtd,format=raw -global driver=timer.esp32.timg,property=wdt_disable,value=true  -serial mon:stdio  -drive id=mysd,if=sd,format=raw,file=sd-image.bin,bus=0,unit=1 -drive id=efuse,if=none,format=raw,file=esp32-efuse.bin -global driver=it8951e,property=rotation,value=0
```


# Allow debugging of the emulated app:
Starts the emulator paused (`-S`), with support for remote gdb on TCP port 1234 (`-s`):
```bash
./build/qemu-system-xtensa -m 4M  -machine m5paper -drive file=merged_qemu.bin,if=mtd,format=raw -global driver=timer.esp32.timg,property=wdt_disable,value=true  -serial mon:stdio  -drive id=mysd,if=sd,format=raw,file=sd-image.bin,bus=0,unit=1 -drive id=efuse,if=none,format=raw,file=esp32-efuse.bin -global driver=it8951e,property=rotation,value=0 -S -s
```

Start the debugger:
```bash
~/Proiecte/Kits/xtensa-esp-elf-gdb/bin/xtensa-esp32-elf-gdb firmware.elf
```

Then use gdb. Examples:
```
b whatever # set breakpoint at whatever function
rwatch *0x3ff48000 # watch for read access at specified address (breaks execution)
b file.c:1234 # set breakpoint at specific place in source code
target remote :1234 # connect to qemu
c # start execution (qemu was started paused)
```


#  Build with platformio / pio
```bash
# Enter your platformio project folder
cd project

# Build the software
pio run

# Merge the compiled files into a merged_qemu.bin file. Adapt the parameters 
# flash size, speed, mode) to your board
cd .pio/build/m5stack-fire
~/.platformio/packages/tool-esptoolpy/esptool.py --chip esp32 merge_bin -o merged_qemu.bin --flash_mode dio --flash_freq 40m --flash_size 16MB --fill-flash-size 16MB   0x1000 bootloader.bin   0x8000 partitions.bin   0x10000 firmware.bin
```

Note: if you have `esptool.py` in your path, you can use that as well, or any other tool that can generate the merged image.


# QEMU coding rules check
```bash
# Compare current working directory to latest commit in the branch
git diff HEAD | scripts/checkpatch.pl --no-signoff -q -

# Compare current working directory to previous commit in the branch
git diff HEAD | scripts/checkpatch.pl --no-signoff -q -
```
