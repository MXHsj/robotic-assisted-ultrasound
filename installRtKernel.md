# Real time kernel installation tutorial

This is a markdown file.
*Reference:* <https://index.ros.org/doc/ros2/Tutorials/Building-Realtime-rt_preempt-kernel-for-ROS-2/>

use <kbd>ctrl</kbd>+<kbd>alt</kbd>+<kbd>T</kbd> to open a terminal

    mkdir ~/kernel
    cd kernel
    wget https://mirrors.edge.kernel.org/pub/linux/kernel/v4.x/linux-4.16.18.tar.gz

you can also find other linux version in <https://mirrors.edge.kernel.org/pub/linux/kernel/v4.x/>
dowmload patch-4.16.18-rt12.patch.gz to `/kernel` from <http://cdn.kernel.org/pub/linux/kernel/projects/rt/4.16/>

    tar -xzf linux-4.16.18.tar.gz 
    mv linux-4.16.18 linux-4.16.18-rt12
    gunzip patch-4.16.18-rt12.patch.gz 
    cd linux-4.16.18-rt12/
    patch -p1 < ../patch-4.16.18-rt12.patch 

check with command line: `uname -srm`
you will see something like: *Linux 4.15.0-66-generic x86_64*
change the linux version of the following line to what you just get.

    cp /boot/config-4.15.0-66-generic .config
    sudo apt-get install bison
    sudo apt-get install flex
    yes '' | make oldconfig
    sudo apt install libncurses5-dev build-essential libssl-dev ccache
    make menuconfig

choose under “Processor Type and Features” — “Preemption Model” — “Fully Preemptible kernel (RT)”

use up and down to select
use left and right to switch between "select" and "exit"
use enter to confirm

    make -j8

**you can use make -j4 if you have 4-cores CPU. This may takes a long time.**

This may takes more than 1 hour to finish. Be patient and don't just use your battery.

    sudo make modules_install
    sudo make install
