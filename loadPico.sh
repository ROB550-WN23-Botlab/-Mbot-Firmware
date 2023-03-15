cd build
make
cd ..
sudo picotool load -f build/src/mbot.uf2
sudo picotool reboot
