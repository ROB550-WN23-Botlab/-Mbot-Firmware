cd /home/pi/mbot-firmware/build
make
cd ..
sudo picotool load -f /home/pi/mbot-firmware/build/src/mbot.uf2
sudo picotool reboot
cd /home/pi/botlab-w23/build/bin
./timesync & ./pico_shim