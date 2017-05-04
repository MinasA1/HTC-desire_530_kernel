PATH=${PATH}:~/A16/toolchains/arm-eabi-4.8/bin/
KERNEL_DIR=$PWD
export USE_CCACHE=1
export CCACHE_DIR=~/android/cache
export ARCH=arm
export SUBARCH=arm
make a16_defconfig ARCH=arm CROSS_COMPILE=arm-eabi-
make menuconfig
make -j5 ARCH=arm CROSS_COMPILE=arm-eabi-
cp arch/arm/boot/zImage-dtb ~/A16/Outbuild/boot.img-kernel
find . -name "*.ko" -exec cp {} ~/A16/Outbuild/modules \;
