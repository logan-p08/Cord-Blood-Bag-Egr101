DUAL PLATE SENSOR SWAP V3 - PLATFORMIO / VS CODE

The original uploaded dual_plate_sensor_swap_v3.ino is preserved byte-for-byte
in original/dual_plate_sensor_swap_v3/dual_plate_sensor_swap_v3.ino.

src/main.cpp contains that same code, with only #include <Arduino.h> and a
blank line prepended. No original lines, settings, comments or control logic
were changed. All function declarations needed for the conversion were
already present. PlatformIO compiles src/main.cpp; the original copy is
outside src and is not compiled a second time.

OPEN AND BUILD
1. Extract the ZIP.
2. Open dual_plate_sensor_swap_v3.code-workspace in VS Code, or use File >
   Open Folder to open the folder that contains platformio.ini.
3. Use your PlatformIO Core terminal. If you use the PlatformIO IDE extension,
   open its PlatformIO terminal. Run these commands from the project folder:

   pio run
   pio run --target upload
   pio device monitor --baud 9600

The first command builds, the second uploads to a connected board, and the
third opens Serial Monitor. Exit the monitor with Ctrl+C before uploading.
The first build needs internet access to install the AVR platform, Arduino
framework and the four Adafruit libraries specified in platformio.ini.
Wire and EEPROM are supplied by the Arduino AVR framework.

The VS Code workspace also includes Build, Upload and Serial Monitor tasks.
These require pio on your terminal PATH. If a task reports that pio is not
found, run the command in the PlatformIO IDE extension's PlatformIO terminal
or make your PlatformIO Core installation available on PATH.

NANO BOOTLOADER
The target is the classic Arduino Nano (ATmega328P), matching this sketch.
nano_new is the default. If your Nano uses ATmega328P (Old Bootloader), run:

   pio run -e nano_old
   pio run -e nano_old --target upload

You can also change default_envs = nano_new to default_envs = nano_old in
platformio.ini. Both environments compile exactly the same src/main.cpp.

If several serial devices are connected, list them with pio device list.
Pass --upload-port YOUR_PORT when uploading, or --port YOUR_PORT for the
serial monitor, replacing YOUR_PORT with the listed device path.

VALIDATION
The preserved .ino and main.cpp after its added header were compared directly
with the uploaded file and are byte-for-byte identical. The INI, workspace
JSON, function declarations and ZIP integrity were checked. PlatformIO Core
is not available in the preparation environment, so a full build was not run.
No hardware upload or runtime testing was performed. Run pio run locally to
verify compilation for your selected Nano environment.
