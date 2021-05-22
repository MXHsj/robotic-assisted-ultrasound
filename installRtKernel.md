## Real time kernel installation tutorial
- author: Xihan Ma
- date: 05-22-2021
---

### Option 1 (suggested to try first) - install older version of real-time kernel (**rt10**)
find guide [here](https://frankaemika.github.io/docs/installation_linux.html) in **Setting up the real-time kernel** section

### Option 2 - install newer version of real-time kernel (**rt44**)
follow this guide if option 1 doesn't work

*Reference:* <https://index.ros.org/doc/ros2/Tutorials/Building-Realtime-rt_preempt-kernel-for-ROS-2/>

```sh
mkdir ~/kernel
cd ~/kernel
```

We can go with a browser to https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/ and see if the version is there. You can download it from the site and move it manually from /Downloads to the /kernel folder, or download it using wget by right clicking the link using “copy link location”.
```sh
wget https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/linux-5.4.78.tar.gz
tar -xzf linux-5.4.78.tar.gz
```

download rt_preempt patch matching the Kernel version we just downloaded over at http://cdn.kernel.org/pub/linux/kernel/projects/rt/5.4/

```sh
wget http://cdn.kernel.org/pub/linux/kernel/projects/rt/5.4/older/patch-5.4.78-rt44.patch.gz
gunzip patch-5.4.78-rt44.patch.gz
cd linux-5.4.78/
patch -p1 < ../patch-5.4.78-rt44.patch
```

We simply want to use the config of our Ubuntu installation, so we get the Ubuntu config with
```sh
cp /boot/config-5.4.0-54-generic .config
```
Open Software & Updates. in the Ubuntu Software menu tick the ‘Source code’ box
We need some tools to build kernel, install them with
```sh
sudo apt-get build-dep linux
sudo apt-get install libncurses-dev flex bison openssl libssl-dev dkms libelf-dev libudev-dev libpci-dev libiberty-dev autoconf fakeroot
```
To enable all Ubuntu configurations, we simply use
```sh
yes '' | make oldconfig
```
Then we need to enable rt_preempt in the kernel.
```sh
make menuconfig
```
and set the following

- Enable CONFIG_PREEMPT_RT
 -> General Setup
  -> Preemption Model (Fully Preemptible Kernel (Real-Time))
   (X) Fully Preemptible Kernel (Real-Time)

Save and exit menuconfig. 

build the kernel and generate debian packages. (10-30min)
```sh
make -j `nproc` deb-pkg
sudo dpkg -i ../*.deb
```
reboot the system, when entering grub interface, select advanced options for ubuntu -> real-time kernel to boot

 verify the kernel version
```sh
uname -r
```

finally
```sh
sudo addgroup realtime
sudo usermod -a -G realtime $(whoami)
```

Afterwards, add the following limits to the realtime group in **/etc/security/limits.conf**:
```
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400
```

### Debugging
1. when building the kernel: `recipe for target 'deb-pkg' failed`
> - check building dependencies
> - check this [answer](https://superuser.com/questions/925079/compile-linux-kernel-deb-pkg-target-without-generating-dbg-package)
> - use `make -j1 deb-pkg` to see the intermediate error message
