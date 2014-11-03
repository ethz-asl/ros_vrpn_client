# /bin/bash
wget ftp://ftp.cs.unc.edu/pub/packages/GRIP/vrpn/old_versions/vrpn_07_28.zip
unzip vrpn_07_28.zip
cd vrpn
mkdir build
cd build
cmake ..
make
sudo make install
cd ../..

# you might want to remove these files, or might not.
#rm -rf vrpn
#rm -f vrpn_07_28.zip

