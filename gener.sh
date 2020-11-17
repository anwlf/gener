sudo insmod gener.ko gpio=17,24,27
sudo chmod 777 /dev/gen*
echo 100000000:5000000 > /dev/gen17
echo 20000:10000 > /dev/gen24
echo 300000:150000 > /dev/gen27
