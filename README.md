# Robocar

A self-driving car built with raspberry pi and C++ 17.

## Videos

Below are some other videos of the robocar working in practice
 - [Robocar autonomously driving in a complex track](https://youtu.be/XUoTdrrCPvY)
 - [Robocar on a running track](https://youtu.be/WzhDoDy-ALc)

## Hardware

#### Vehicle

The following table contains the basic components necessary for the car.

| Item              | Approx. Price | Link                                                                                                    |
|-------------------|---------------|---------------------------------------------------------------------------------------------------------|
| Raspberry Pi      | $100          | [amazon](https://www.amazon.com/gp/product/B07V5JTMV9/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1) |
| Car chassi        | $25           | [amazon](https://www.amazon.com/gp/product/B07F759T89/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1) |
| Actuator          | $7            | [amazon](https://www.amazon.com/gp/product/B01DG61YRM/ref=ppx_yo_dt_b_asin_title_o01_s01?ie=UTF8&psc=1) |
| Wires             | $5            | [amazon](https://www.amazon.com/gp/product/B01LZF1ZSZ/ref=ppx_yo_dt_b_asin_title_o09_s00?ie=UTF8&psc=1) |
| Camera            | $8            | [amazon](https://www.amazon.com/gp/product/B07QNSJ32M/ref=ppx_yo_dt_b_asin_title_o07_s00?ie=UTF8&psc=1) |
| USB Power Supply  | $12           | [amazon](https://www.amazon.com/gp/product/B07V5FPNNZ/ref=ppx_yo_dt_b_asin_title_o06_s00?ie=UTF8&psc=1) |
| Power bank        | $35           | [amazon](https://www.amazon.com/gp/product/B078S6LH8L/ref=ppx_yo_dt_b_asin_title_o05_s00?ie=UTF8&psc=1) |

#### Others

| Item                    | Approx. Price | Link                                                                                                    |
|-------------------------|---------------|---------------------------------------------------------------------------------------------------------|
| Additional wires        | $10           | [amazon](https://www.amazon.com/gp/product/B07CWP7HPT/ref=ppx_yo_dt_b_asin_title_o02_s00?ie=UTF8&psc=1) |
| Soldering Iron          | $15           | [amazon](https://www.amazon.com/gp/product/B06XZ31W3M/ref=ppx_yo_dt_b_asin_title_o02_s00?ie=UTF8&psc=1) |
| Traffic Signs           | $12           | [amazon](https://www.amazon.com/gp/product/B0006KQIX2/ref=ppx_yo_dt_b_asin_title_o04_s00?ie=UTF8&psc=1) |
| Tape (for making lanes) | $10           | [amazon](https://www.amazon.com/AmazonBasics-Masking-Tape-Inch-Rolls/dp/B07QHSKGMH/ref=sxin_3_ac_d_rm)  |

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
        -D BUILD_EXAMPLES=OFF .. && \
    make -j4 && \
    sudo make install && \
    sudo ldconfig && \
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
Note: Tensorflow folder is in .gitignore to avoid having it checked-in nto this
repo. In the future we might consider having it as submodule.

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
$ sudo bin/robocar
```
