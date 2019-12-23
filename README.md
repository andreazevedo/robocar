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

##### 4) OpenCV
```bash
# First install dependencies
$ sudo apt-get -y install build-essential cmake pkg-config \
    libjpeg-dev libtiff-dev libjasper-dev libpng12-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev \
    libatlas-base-dev gfortran

# Build opencv from source
$ wget -O opencv.tar.gz https://github.com/opencv/opencv/archive/4.1.2.tar.gz && \
    wget -O opencv_contrib.tar.gz https://github.com/opencv/opencv_contrib/archive/4.1.2.tar.gz && \
    tar zxf opencv.tar.gz && \
    tar zxf opencv_contrib.tar.gz && \
    mv opencv-4.1.2/ opencv && \
    mv opencv_contrib-4.1.2/ opencv_contrib && \
    cd opencv && \
    mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
        -D ENABLE_NEON=ON \
        -D ENABLE_VFPV3=ON \
        -D BUILD_TESTS=OFF \
        -D OPENCV_ENABLE_NONFREE=ON \
        -D INSTALL_PYTHON_EXAMPLES=OFF \
        -D BUILD_EXAMPLES=OFF .. &&
    make -j4 &&
    sudo make install &&
    sudo ldconfig &&
    sudo apt-get update

# Clean up downloads
$ cd ~/
$ rm opencv.tar.gz && rm opencv_contrib.tar.gz
$ rm -r opencv && rm -r opencv_contrib

# Reboot
$ sudo reboot
```


#### 5) Tensforflow Lite
Before running the following, make sure you are at the root of this project.
```bash
# Make sure you are at robocar root before running this.
$ sudo apt-get -y install build-essential && \
    cd third_party && \
    git clone https://github.com/tensorflow/tensorflow && \
    cd tensorflow && \
    ./tensorflow/lite/tools/make/download_dependencies.sh && \
    ./tensorflow/lite/tools/make/build_rpi_lib.sh
```
Note: Tensorflow folder is in .gitignore to avoid having it checked-in this repo. In the future we might consider having it as submodule.

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
