Teensy 3.0 Project Template
===========================

Purpose
-------

An easy starting point for a Teensy 3.0 project which might not fit inside the
arduino build environment.


Reasons to Use
--------------

- You need to modify the teensy core
- You don't love Java IDE's
- You love Make
- Because


Using
-----

1. Put your code in main.cpp
2. Build your code ```make```
3. Upload your code ```make upload```


Make Targets
------------

- `make` alias for `make hex`
- `make build` compiles everything and produces a .elf
- `make hex` converts the elf to an intel hex file
- `make post_compile` opens the launcher with the correct file
- `make upload` uploads the hex file to a teensy board
- `make reboot` reboots the teensy


Where everything came from
--------------------------

- The `/teensy3` sub-folder is taken from [The Teensy 3.0 Cores](https://github.com/PaulStoffregen/cores/tree/master/teensy3)
- The `/teensy3/tools/` sub-folder is taken from [Teensyduino](http://www.pjrc.com/teensy/td_download.html)
- The `main.cpp` file is moved, unmodified from `/teensy3/main.cpp`
- The `Makefile` file is moved, modified from `/teensy3/Makefile`

Modifications to `Makefile` include
- Change tools directory
- Calculate target name from current directory
- Prettify rule output
- Do not upload by default, only build
