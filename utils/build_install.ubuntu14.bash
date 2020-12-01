#!/bin/bash -vxe

dir=$(dirname $0)/../

cd $dir/src/drivers/
rm Makefile
ln -s Makefile.ubuntu14 Makefile
make clean
make 
sudo insmod rtcar.ko
sleep 1
sudo chmod 666 /dev/rt*
echo 0 > /dev/rtmotoren0
