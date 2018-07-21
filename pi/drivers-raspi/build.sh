#!/bin/bash

if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root (use sudo)" 1>&2
   exit 1
fi

set -x

# TODO(ensonic): get version from git
ver="0.1"

scriptdirpath=$(dirname $(realpath $0))

apt-get -y install raspberrypi-kernel
apt-get -y install raspberrypi-kernel-headers
apt-get -y install -t stretch dkms debhelper

# locate currently installed kernels (may be different to running kernel if
# it's just been updated)
kernels=$(ls /lib/modules | sed "s/^/-k /")
#kernels=$(uname -r)

# https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=832558
# "Version 2.2.0.3-5 introduced $debian_build_arch as part of the then
# invoked mv command, which fails."
sed -i.bak -e 's/Architecture: all/Architecture: DEBIAN_BUILD_ARCH/' \
  /etc/dkms/template-dkms-mkdeb/debian/control

rm /etc/dkms/template-dkms-mkdeb/debian/dirs

function build_module {
  src=$1
  mod=$2

  if [[ -e /usr/src/$mod-$ver || -e /var/lib/dkms/$mod/$ver ]]; then
    dkms remove -m $mod -v $ver --all
    rm -rf /usr/src/$mod-$ver
  fi
  mkdir -p /usr/src/$mod-$ver
  cp -a $src/* /usr/src/$mod-$ver/
  dkms add -m $mod -v $ver
  dkms build $kernels -m $mod -v $ver
  dkms mkdeb $kernels -m $mod -v $ver --source-only --legacy-postinst=0
  mv /var/lib/dkms/$mod/$ver/deb/* /home/pi/deb/
  dkms remove -m $mod -v $ver --all
  rm -rf /usr/src/$mod-$ver
}

mkdir -p /home/pi/deb
build_module "sound" "aiy-voicebonnet-soundcard"
build_module "aiy" "aiy"
build_module "leds" "leds-ktd202x"
build_module "pwm" "pwm-soft"

cd arch/arm/boot/dts/overlays
dtc  -W no-unit_address_vs_reg -@ -O dtb aiy-visionbonnet-overlay.dts \
  -o /boot/overlays/aiy-visionbonnet.dtbo
dtc  -W no-unit_address_vs_reg -@ -O dtb aiy-voicebonnet-overlay.dts \
  -o /boot/overlays/aiy-voicebonnet.dtbo

sed -i \
  -e "s/^dtparam=audio=on/#\0/" \
  -e "s/^dtparam=spi=on/#\0/" \
  -e "s/^#\(dtparam=i2c1=on\)/\1/" \
  -e "s/^#\(dtparam=i2s=on\)/\1/" \
  /boot/config.txt
sed -i \
  -e "s/dtoverlay=googlevoicehat-soundcard/# \0/" \
  -e "s/dtoverlay=i2s-mmap/# \0/" \
  /boot/config.txt
grep -q "pwm-soft" /etc/modules || \
  echo "pwm-soft" >> /etc/modules
# Enable Camera
grep -q "start_x=1" /boot/config.txt || \
  echo "start_x=1" >> /boot/config.txt
grep -q "gpu_mem=128" /boot/config.txt || \
  echo "gpu_mem=128" >> /boot/config.txt
