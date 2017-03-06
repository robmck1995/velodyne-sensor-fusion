# velodyne-sensor-fusion

## Dependency Installation for PCL
There are quite a few dependencies that must be installed before PCL can be compiled. I will go through how to install all of them.

You will probably already have most of this on your system, but just to make sure you should run the following

```
sudo apt-get update
sudo apt-get install build-essential
```

# Eigen
Go to [this website](http://eigen.tuxfamily.org/index.php?title=Main_Page) and download the latest stable release with the tar.gz extension. Then go to the download directory and unzip with the following
```
tar xvzf <eigen-archive>
```

Then cd into the unzipped directory and run the following commands

```
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

# VTK
Pretty much the same as Eigen, go to [this website](http://www.vtk.org/download/) and download the source release with the tar.gz extension. Then go the download directory and run
```
tar xvzj <VTK-archive>
```

Then cd into the unzipped directory and run the following commands

```
mkdir build
cd build
cmake ..
make -j4
sudo make install
```
# Boost
Boost has a bunch of dependencies so unless you already have it installed you'llwant to get the package and all dependencies with

```
sudo apt-get install libboost-all-dev
```

Now depending on the version of Ubuntu you have the installed version of Boost may not be new enough. If you're hitting errors like that go to [this website](http://www.boost.org/users/download/). Under the current release section click "Download" and then download the version with the tar.gz extension.

Then cd into the download directory and run
```
tar xvzf <boost-archive>
```

Then cd into the unzipped folder and do the following


```
sudo ./bootstrap.sh --prefix=/usr/local
sudo ./b2 install
```

# FLANN
Go to [this website](http://www.cs.ubc.ca/research/flann/#download) and under the "Getting FLANN" section download the source zip file.

If you don't already have unzip installed run
```
sudo apt-get install unzip
```

Then cd into the download directory and run

```
unzip <FLANN-archive>
```

Then cd into the unzipped directory and run the following commands

```
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

# OpenGL
This one's easy though the package name isn't obvious. Just install the following.

```
sudo apt-get install freeglut3-dev
```
