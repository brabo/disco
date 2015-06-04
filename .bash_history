wget http://downloads.sourceforge.net/project/openocd/openocd/0.8.0/openocd-0.8.0.tar.gz?r=http%3A%2F%2Fsourceforge.net%2Fprojects%2Fopenocd%2Ffiles%2Fopenocd%2F0.8.0%2F&ts=1418665767&use_mirror=softlayer-ams
ls
mv openocd-0.8.0.tar.gz?r=http:%2F%2Fsourceforge.net%2Fprojects%2Fopenocd%2Ffiles%2Fopenocd%2F0.8.0%2F openocd-0.8.0.tar.gz
ls
tar xvzf openocd-0.8.0.tar.gz 
wget https://alock.googlecode.com/files/alock-svn-94.tar.bz2
tar xvjf alock-svn-94.tar.bz2 
cd alock-svn-94/
ls
cat README.txt 
./configure --help
./configure --with-passwd
ls
make
sudo make install
cd ..
wget http://gcc.cybermirror.org/releases/gcc-4.9.2/gcc-4.9.2.tar.gz
wget http://ftp.gnu.org/gnu/gdb/gdb-7.8.1.tar.gz
wget http://ftp.gnu.org/gnu/binutils/binutils-2.24.tar.gz
ls
tar xvzf binutils-2.24.tar.gz 
ls
tar xvzf gcc-4.9.2.tar.gz 
wget ftp://sourceware.org/pub/newlib/newlib-2.1.0.tar.gz
ls ../arm
ls ../arm/bin
cd src
ls
tar xvzf gdb-7.8.1.tar.gz 
ls
wget http://ftpmirror.gnu.org/mpfr/mpfr-3.1.2.tar.xz
wget http://ftpmirror.gnu.org/gmp/gmp-6.0.0a.tar.xz
wget http://ftpmirror.gnu.org/mpc/mpc-1.0.2.tar.gz
ls
tar xf mpc-1.0.2.tar.gz 
ls
ls mpc-1.0.2
tar xvf mpfr-3.1.2.tar.xz 
ls
tar xvf gmp-6.0.0a.tar.xz 
ls
wget ftp://gcc.gnu.org/pub/gcc/infrastructure/isl-0.12.2.tar.bz2
wget ftp://gcc.gnu.org/pub/gcc/infrastructure/cloog-0.18.1.tar.gz
ls -lah
tar xvjf isl-0.12.2.tar.bz2 
ls
tar xvzf cloog-0.18.1.tar.gz 
ls
rm mpc-1.0.2.tar.gz.1
rm mpc-1.0.2.tar.gz.2
ls
tar xvzf newlib-2.1.0.tar.gz 
ls
tar xvvf Sublime\ Text\ 2.0.2\ x64.tar.bz2 
ls
chown -R brabo:brabo Sublime\ Text\ 2
ls -lah
vim /home/brabo/.bashrc 
mv Sublime\ Text\ 2 ST
ls
cd ..
mkdir build
cd build/
cd ..
mkdir arm
cd build/
../src/binutils-2.24/configure --target=arm-none-eabi --prefix=/home/brabo/arm/ --enable-interwork --enable-multilib
make
make install
ls
cd ..
rm -rf build
cd src/binutils-2.24/
mkdir build
cd build
../configure --target=arm-none-eabi --prefix=/home/brabo/arm/ --enable-interwork --enable-multilib
make
make install
cd
vim .bashrc
. .bashrc
echo $PATZH
echo $PATH
export PATH=/home/brabo/arm/bin:/usr/local/bin:/usr/bin:/bin:/usr/local/games:/usr/games
cd src/newlib-2.1.0/
cd ../gcc-4.9.2/
mkdir build
cd build/
../configure --target=arm-none-eabi --prefix=$MYTOOLS --enable-interwork --enable-multilib --enable-languages="c,c++" --with-newlib --with-headers= ../../newlib-2.1.0
../configure --target=arm-none-eabi --prefix=/home/brabo/arm --enable-interwork --enable-multilib --enable-languages="c,c++" --with-newlib --with-headers=../../newlib-2.1.0/newlib/libc/include/
cd ..
ln -s ../mpfr-3.1.2 mpfr
ln -s ../gmp-6.0.0 gmp
ln -s ../mpc-1.0.2 mpc
ln -s ../isl-0.12.2 isl
ln -s ../cloog-0.18.1 cloog
cd build/
../configure --target=arm-none-eabi --prefix=/home/brabo/arm --enable-interwork --enable-multilib --enable-languages="c,c++" --with-newlib --with-headers=../../newlib-2.1.0/newlib/libc/include/
make
make clean
ls
rm -rf *
../configure --target=arm-none-eabi --prefix=/home/brabo/arm --disable-threads --enable-interwork --enable-multilib --enable-languages="c,c++" --with-newlib --with-headers=../../newlib-2.1.0/newlib/libc/include/
make
make clean
rm -rf *
../configure --target=arm-none-eabi --prefix=/home/brabo/arm --with-system-zlib --disable-threads --enable-interwork --enable-multilib --enable-languages="c,c++" --with-newlib --with-headers=../../newlib-2.1.0/newlib/libc/include/
make
make clean
rm -rf *
../configure --target=arm-none-eabi --prefix=/home/brabo/arm --disable-threads --enable-interwork --enable-multilib --enable-languages="c,c++" --with-newlib --with-headers=../../newlib-2.1.0/newlib/libc/include/
make
make clean
rm -rf *
../configure --target=arm-none-eabi --prefix=/home/brabo/arm --with-system-zlib --disable-threads --enable-interwork --enable-multilib --enable-languages="c,c++" --with-newlib --with-headers=../../newlib-2.1.0/newlib/libc/include/
make
make install
cd src
git clone git@github.com:libopencm3/libopencm3.git
ssh-keygen
cat ~/.ssh/id_rsa.pub 
git clone git@github.com:libopencm3/libopencm3.git
git clone git@github.com:texane/stlink.git
ls stlink/
cd stlink/
./autogen.sh 
./configure
vim Makefile.am 
./configure
make clean
ls
cd ..
rm -rf stlink/
git clone git@github.com:texane/stlink.git
cd stlink/
./autogen.sh 
vim Makefile.am 
./autogen.sh 
make clean
./configure
make
vim /home/brabo/.bashrc 
st-util
cd src/stlink/
ls
st-util
cd ..
ls
cd gdb-7.8.1/
cd ..
cp -R gcc-4.9.2 testgcc
cd testgcc/build/ 
cd src
ls
cd newlib-2.1.0/
mkdir build
cd build/
../configure --target=arm-none-eabi --prefix=/home/brabo/arm --enable-interwork --enable-multilib
make
cd
cd ~/src/newlib-2.1.0/build
vim /home/brabo/src/newlib-2.1.0/build/arm-none-eabi/libgloss/arm/Makefile 
make
vim /home/brabo/src/newlib-2.1.0/build/arm-none-eabi/libgloss/arm/Makefile 
make clean; make
cd /home/brabo/src/newlib-2.1.0/build/arm-none-eabi/libgloss/arm/cpu-init
ls ../../../../libgloss/arm/../config/default.mh
ls ../../libgloss/arm/../config/default.mh
ls ../../../libgloss/arm/../config/default.mh
ls ../libgloss/arm/../config/default.mh
ls ../../../../../libgloss/arm/../config/default.mh
vim Makefile 
make
cd ../../../
cd ..
make
cd /home/brabo/src/newlib-2.1.0/build/arm-none-eabi/thumb/libgloss/arm/cpu-init
ls ../../../../../../libgloss/arm/../config/default.mh
vim Makefile 
make
cd ../../../../
cd ..
make
cd /home/brabo/src/newlib-2.1.0/build/arm-none-eabi/fpu/libgloss/arm/cpu-init
ls ../../../../../../libgloss/arm/../config/default.mh
vim Makefile 
make
cd ../../../../..
make
make install
cd ../../gcc-4.9.2/
cd build/
ls
make install
cd ../../gdb-7.8.1/
mkdir build
cd build/
../configure --target=arm-none-eabi --prefix=/home/brabo/arm --enable-interwork --enable-multilib
make
aptitude search term
make clean
ls
rm -rf *
../configure --target=arm-none-eabi --prefix=/home/brabo/arm --enable-interwork --enable-multilib
make
make install
ls ~/arm
ls ~/arm/bin
cd ../..
ls
mkdir locm3
cd locm3/
tar xvzf locm3.tar.gz 
cd libopencm3/
make clean; TARGET=stm32/f1 make
make clean; TARGET=stm32/f1 make clean; TARGET=stm32/f1 makemake
make clean; TARGET=stm32/f1 DESTDIR=/home/brabo/arm make
make install
DESTDIR=/home/brabo/arm make install
ls /home/brabo/arm
ls /home/brabo/arm/arm-none-eabi/
ls /home/brabo/arm/arm-none-eabi/lib
cd src
cd openocd-0.8.0/
mkdir build
cd build/
../configure --help
../configure --enable-ti-icdi --enable-buspirate --enable-sysfsgpio --enable-buspirate --enable-ftdi --enable-stlink
../configure --enable-ti-icdi --enable-buspirate --enable-sysfsgpio --enable-buspirate --enable-ftdi --enable-stlink --program-prefix=/home/brabo/arm
make
make install
vim Makefile 
make install
../configure --enable-ti-icdi --enable-buspirate --enable-sysfsgpio --enable-buspirate --enable-ftdi --enable-stlink --program-prefix=/home/brabo/arm
make clean
ls
rm -rf *
../configure --enable-ti-icdi --enable-buspirate --enable-sysfsgpio --enable-buspirate --enable-ftdi --enable-stlink --program-prefix=/home/brabo/arm
make install
make clean
rm -rf *
../configure --enable-ti-icdi --enable-buspirate --enable-sysfsgpio --enable-buspirate --enable-ftdi --enable-stlink --program-prefix=/home/brabo/arm
vim Makefile 
make
make install
vim Makefile 
make clean
rm -rf *
../configure --enable-ti-icdi --enable-buspirate --enable-sysfsgpio --enable-buspirate --enable-ftdi --enable-stlink --prefix=/home/brabo/arm
make
make install
arm-none-eabi-gdb 
cd src
git clone git@github.com:libopencm3/libopencm3.git
