# clone repo
git clone https://github.com/daniel-santos/mcp2210-linux.git
cd mcp2210-linux

# compile and install
KERNELDIR=/usr/src/kernels/4.11.12-100.fc24.x86_64 make
vi user/settings.h
sudo KERNELDIR=/usr/src/kernels/4.11.12-100.fc24.x86_64 make modules_install
#sudo rmmod mcp2210
sudo modprobe mcp2210
sudo ./user/mcp2210_bind.sh

# load config
LD_LIBRARY_PATH=`pwd`/user user/mcp2210-util set config 31

LD_LIBRARY_PATH=`pwd`/user user/mcp2210-util encode > config.dat
# determine file size
$size=$(stat --printf="%s" config.dat)
sudo LD_LIBRARY_PATH=`pwd`/user user/mcp2210-util -i config.dat eeprom write addr=0 size=$size 

LD_LIBRARY_PATH=`pwd`/user user/mcp2210-util test config

