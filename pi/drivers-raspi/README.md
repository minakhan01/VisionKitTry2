Source code for the voiceHAT kernel drivers.

## Installing

Run
``` shell
make deploy
```
to send the source to the Raspberry Pi. On the Raspberry Pi run:
``` shell
cd ~/drivers-raspi
sudo ./build.sh
sudo dpkg -i drivers-raspi.deb
```
to install the drivers.

