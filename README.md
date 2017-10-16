# TEAM WHALERS #

### What is this repository for? ###

* Team Whalers code for NASA Space Robotics Challenge (https://bitbucket.org/osrf/srcsim)

### How do I get set up? ###

* Requirements: https://bitbucket.org/osrf/srcsim/wiki/system_requirements
* After Ubuntu 14.04 install:

```
sudo apt-get update
sudo apt-get upgrade 
sudo apt-get install build-essential
sudo apt-get install git 
```
 * Add the Gazebo and SRCSim repositories
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

```
```
wget -O - http://packages.osrfoundation.org/gazebo.key | sudo apt-key add -
```
```
sudo sh -c 'echo "deb http://srcsim.gazebosim.org/src trusty main" \ > /etc/apt/sources.list.d/src-latest.list'
```
```
wget -O - http://srcsim.gazebosim.org/src/src.key | sudo apt-key add -
```
```
wget -O - https://bintray.com/user/downloadSubjectPublicKey?username=bintray | sudo apt-key add -
```
```
sudo apt-get update
```



* Install ROS-Indigo
```
sudo apt-get install -f ros-indigo-desktop-full  ros-indigo-simulators ros-indigo-gazebo-ros-pkgs ros-indigo-gazebo-msgs ros-indigo-gazebo-plugins ros-indigo-gazebo-ros
```
* Follow the OSRF setup at https://bitbucket.org/osrf/srcsim/wiki/system_setup (skip to step 3 as you have already added the Gazebo package repository above)
