# Surface Reconstruction
Surface Reconstruction from sparse point-clouds module developed for the U.S.B. Mechatronics Research Group.

# Requirements

This software requires the following packages:

- CMake 2.6 or higher
- PCL 1.7.0 or higher

And has been succesfully tested on Ubuntu 14.04, no GPU required.

# Installation

* **Download dependencies**

  To install CMake and Build-essentials enter your workspace and enter the following:
  
  ```
    $ sudo apt-get install build-essential
    $ sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
  ```
  
  To install pcl open terminal and enter:
  
  ```
    $ sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
    $ sudo apt-get update
    $ sudo apt-get install libpcl-all
  ```
  
* **Cloning the repository and building**

  locate the terminal in your workspace folder and enter:
  
  ```
    $ git clone https://github.com/BerAceituno/surface-reconstruction.git
  ```
  
  locate the terminal in the source code folder and build by entering the following commands:
  
  ```
    $ cd surface-reconstruction
    $ cmake . 
    $ make
  ```
  the complete installation process should take around 2 minutes to build on dual core processor.

# Use

The software is ran via terminal by locating in the build folder and entering:

```
  $ ./Surface -i input.pcd -o output.ply -a <reconstruction algorithm> -u <[Optional] process the unfiltered cloud> -d <[optional] Poisson depth> -r <[optional] Ball Pivoting radius> -m <[optional] Ball Pivoting MLS radius> -n <[optional] Ball Pivoting normal estimation radius> -c <[optional] Ball Pivoting clustering>
```

where the currently supported reconstrctions algorithms are:

* Poisson:        enter -a Poisson, poisson, p or P.
* Ball Pivoting:  enter -a Ball_Pivoting, ball_pivoting, bp or BP.

  NOTE: In the case the output directory is not created the saving process will fail.

**Distributed under an MIT License**
