# libxtrx
High level XTRX API

## Build instructions :

### Install base requirements
```
   $ sudo apt-get install build-essential libqcustomplot-dev libusb-1.0-0-dev libsoapysdr-dev soapysdr-module-lms7
```

### Build liblms7002m
```
   $ git clone https://github.com/xtrx-sdr/liblms7002m
   $ cd liblms7002m
   $ mkdir build
   $ cd build
   $ cmake ..
   $ make
   $ sudo make install
   $ cd ..
```

### Build liblms7002m
```
   $ git clone https://github.com/xtrx-sdr/libusb3380
   $ cd libusb3380
   $ mkdir build
   $ cd build
   $ cmake ..
   $ make
   $ sudo make install
   $ cd ..
```

### Build libxtrxll
```
   $ git clone https://github.com/xtrx-sdr/libxtrxll
   $ cd libxtrxll
   $ mkdir build
   $ cd build
   $ cmake ..
   $ make
   $ sudo make install
   $ cd ..
```

### Build libxtrxdsp
```
   $ git clone https://github.com/xtrx-sdr/libxtrxdsp
   $ cd libxtrxdsp
   $ mkdir build
   $ cd build
   $ cmake ..
   $ make
   $ sudo make install
   $ cd ..
```

### Build libxtrx
```
   $ git clone https://github.com/xtrx-sdr/libxtrx
   $ cd libxtrx
   $ mkdir build
   $ cd build
   $ cmake ..
   $ make
   $ sudo make install
   $ cd ..
```
