Teensy 3.X Project Template
===========================

DOC - C4 model
-------

System Context:  

![alt text](http://www.plantuml.com/plantuml/png/NP5DRnD138Rl-ojCJgcK3z9IMd5g26seIDke2U8OJNQSxD3vM8qzILU8_uw7ZLXW3Yll-F7hUUqROXKvELkyCRwqIIDKp3Lz68-ZEexsXgkqJOIn39xHywWCRbnRvNFDa5XRintFtWsN-UhhmsBi5356kThCf48PZt5KP_s23Lk4LIDv1pc9hmpQg7rK3dOXmigh5uIvSWXr3D-nvA9OOgJW-xNeDnW7q6k3daI_GVJoG3BIj7TM5AtuvcD8NglOzCk9SAtilfCL57uK8AVb-xH6zDG8smPmCJhF7QPH9-C3M2C2q-S74ECiBtNIwv_okxNGyeLL6KKXrqv6ro0JhM1oVlqwUVyMYA-kBYvXsy1H7P0g5V4_cI_OR8EAMgebh9NEEhD471oyd79mb8t09R0gUKZoKrP8uCGmemOEsUB1Q8HK3q27en-0nHqF89fznQ2y1XGqIkkVXPndjDrm_tv17P48AJScWOKx-UnM4HBNYQaTBRNEJsGdrCqMBPOSJQaiaDbxPRCziM4E23OSJwL_-DDKTPGy0UyuuwlblN2owCzFtpPtYwVf-lvnlbdSpcyVF_NFYXlqEZdx2m00)

Containers:  

![alt text](http://www.plantuml.com/plantuml/png/PP5FRnen4CNl-occd422fIHD4_MKbYOqAlcZGjLB9MJM0sl5VrQUSUYgwdVlcB8WDPUTzNZUx_dPryGgSdAsU6DyPPD6g9aR-Z0UHxKhjuRhj4w4iGgUqNDP1JTkhF9PCoBMrgp7qtUZfzpwVZyVEqMCKLghgIYKyHZB9lCBDcmHZbtGHcsZSX0si4JqrC9vUGOR4zrEHIoAHIiarwVbxN88FVd2eeeos9FLfBm2EMyDaxEhNvFtRu7uylBy0jOjxDGBKYs4tk28-8hjEgYeHIc_ZTAPCKt4mS7pOGzsaXKkW5N58v95LIE12qboQ-00JGmlHYEaPWWwxFmGB6vu2D5iQmRbDQ2CHh4jFebAgzZsguaOdTBr1l2x083JBVHfuTKpojI-mamvx6MDB4WYDaaSnBb28cbiPHjYfaitCK58BBKKq_-mzrA5amlO49eClVE6IOGXMCgvvPyafKwsGn-S1FUdABwXxJZ7KoP2ogYd_MLqHuHDY7j0-TDFbRMlQ1dLCGuFsf7-9UXcJY_L9UZczaRvSXTV7d-iRkUF7vTt3xFL_6Psy_2vFoYkqUlax5y0)

Components:



Forked repo.

Purpose
-------

An easy starting point for a Teensy 3.X project which might not fit inside the
arduino build environment.


Reasons to Use
--------------

- You need to modify the teensy core
- You don't love Java IDE's
- You love Make
- Because


Setup
-----

Install the Teensy udev rule: `sudo cp tools/49-teensy.rules /etc/udev/rules.d/`

Then unplug your Teensy and plug it back in.


Using
-----

1. Put your code in `src/main.cpp`
2. Put any libraries you need in `libraries`
3. Set the TEENSY variable in `Makefile` according to your teensy version
4. Build your code ```make```
5. Upload your code ```make upload```


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

- The `teensy3` sub-folder is taken from [The Teensy 3 Cores](https://github.com/PaulStoffregen/cores/tree/master/teensy3)
- The `tools` sub-folder is taken from [Teensyduino](http://www.pjrc.com/teensy/td_download.html)
- The `src/main.cpp` file is moved, unmodified from `teensy3/main.cpp`
- The `Makefile` file is moved, modified from `teensy3/Makefile`
- The `49-teensy.rules` file is taken from [PJRC's udev rules](http://www.pjrc.com/teensy/49-teensy.rules)

Modifications to `Makefile` include
- Add support for arduino libraries
- Change tools directory
- Calculate target name from current directory
- Prettify rule output
- Do not upload by default, only build
