## How to use on arch linux based distro

### Install ARM toolchain
`sudo pacman -S arm-none-eabi-gcc arm-none-eabi-binutils arm-none-eabi-newlib arm-none-eabi-gdb`

### Install the VS Code extensions
• Arm Assembly (dan-c-underwood)
• Cortex-Debug (marus25)
• Cortex-Debug: Device Support Pack - STM32F1 (marus25)

### Test it
Open example file in VS Code, go to Run and Debug and press Flash (F5)