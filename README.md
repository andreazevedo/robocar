# Robocar

A self-driving car built with raspberry pi and C++ 17.

## Hardware

Coming soon.

## Software

### Installation

#### Dependencies
##### 1) cmake 3.1+
```bash
sudo apt install gcc g++
```

##### 2) gcc 7.4+
```bash
sudo apt install gcc
```

##### 3) googletest (optional - only for running tests)
```bash
$ wget https://github.com/google/googletest/archive/release-1.10.0.tar.gz && \
    tar zxf release-1.10.0.tar.gz && \
    rm -f release-1.10.0.tar.gz && \
    cd googletest-release-1.10.0 && \
    cmake . && \
    make -j && \
    sudo make install
```

### Build
```bash
$ cmake .
$ make
```

### Run tests (optional)
```bash
$ cmake . -DBUILD_TESTS=ON
$ make
$ make test
```

### Run
The binary will be in the `bin/` directory:
```bash
$ bin/robocar
```
