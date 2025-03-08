# Instalasi Kinect

## Install Libfreect

1. Install pre-requisite packages
```bash
sudo apt-get install git cmake build-essential libusb-1.0-0-dev
sudo apt-get install freeglut3-dev libxmu-dev libxi-dev
```

2. Fetch and build the libfreenect driver
```bash
git clone https://github.com/OpenKinect/libfreenect
cd libfreenect
mkdir build
cd build
cmake -L .. # -L lists all the project options
make
```

3. Install Udev rules
Simply place this file in /etc/udev/rules.d
```bash
# 51-kinect.rules
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666"
# ATTR{product}=="Xbox NUI Audio"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666"
# ATTR{product}=="Xbox NUI Camera"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666"

# Kinect for Windows
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c2", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02be", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02bf", MODE="0666"
```

4. Install libfreenect System-Wide:
After building libfreenect, you need to install it so that its components are accessible system-wide. From the build directory of libfreenect, execute:
```bash
sudo make install
```

5. Update the Shared Library Cache:
After installation, ensure that the system recognizes the new library by updating the shared library cache:
```bash
sudo ldconfig
```

6. Copy the repo
```bash
cd ~/Documents/GitHub/jolie_ws/src
git clone https://github.com/fadlio/kinect_ros2
```

7.  Install any missing ROS packages
```bash
rosdep install --from-paths src --ignore-src -r -y
```

8. Nonaktifkan Autosuspend USB (jika diperlukan)
- Edit file /etc/default/grub:
```bash
sudo nano /etc/default/grub
```
- Temukan baris GRUB_CMDLINE_LINUX_DEFAULT dan tambahkan usbcore.autosuspend=-1 di dalam tanda kutip. Contoh:
```bash
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.autosuspend=-1"
```

- Simpan perubahan dan perbarui konfigurasi GRUB:
```bash
sudo update-grub
```

- Reboot sistem:
```bash
sudo reboot
```

9. Verify that the Kinect is detected
```bash
lsusb
dmesg | grep -i kinect
freenect-glview
```